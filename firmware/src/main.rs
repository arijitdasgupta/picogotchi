#![no_std]
#![no_main]

//! Picogotchi: a Tamagotchi P1 emulator for the Raspberry Pi Pico 2 (RP2350)
//! driving a 128x64 SSD1306 I2C OLED with three buttons.
//!
//! The hardware wiring mirrors the `bike_w_friends` project:
//!   * I2C1 on GP2 (SDA) / GP3 (SCL) at 1 MHz
//!   * buttons on GP16 / GP17 / GP18 (pull-up, active low)
//!   * status LED on GP25
//!
//! There is no buzzer.

mod rom;

use defmt::{info, warn};
use defmt_rtt as _;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal::digital::{InputPin, OutputPin};
use hal::fugit::RateExtU32;
use panic_probe as _;
use rp235x_hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::{self, FunctionI2c, Pin},
    pac,
    timer::Timer,
    Watchdog,
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
use tama::{debounce::Debouncer, display, Button, ButtonState, Tama};

/// Tell the Boot ROM about our application (required to boot on RP2350).
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Program metadata for `picotool info`.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Picogotchi - Tamagotchi P1 emulator"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

pub type DisplayType = Ssd1306<
    I2CInterface<
        hal::I2C<
            pac::I2C1,
            (
                Pin<gpio::bank0::Gpio2, gpio::FunctionI2c, gpio::PullUp>,
                Pin<gpio::bank0::Gpio3, gpio::FunctionI2c, gpio::PullUp>,
            ),
        >,
    >,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

/// The Tamagotchi screen updates at ~3 fps in the reference implementation.
const FRAME_PERIOD_US: u64 = 1_000_000 / 3;

/// Switch debounce window. Mechanical switches bounce for a few milliseconds.
const DEBOUNCE_US: u64 = 20_000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // The Pico 2's external crystal runs at 12 MHz.
    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    // SSD1306 on I2C1: SDA = GP2, SCL = GP3.
    let sda_pin: Pin<_, FunctionI2c, _> = pins.gpio2.reconfigure();
    let scl_pin: Pin<_, FunctionI2c, _> = pins.gpio3.reconfigure();
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        1.MHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display: DisplayType =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
    display.init().unwrap();

    // Status LED.
    let mut led = pins.gpio25.into_push_pull_output();
    let _ = led.set_high();

    // Buttons: active low, internal pull-ups.
    let mut bt_left = pins.gpio16.into_pull_up_input();
    let mut bt_middle = pins.gpio17.into_pull_up_input();
    let mut bt_right = pins.gpio18.into_pull_up_input();

    // The 1 MHz microsecond timer paces the emulated CPU to real time.
    let timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let mut tama = Tama::new(&rom::G_PROGRAM_B12);
    let ticks_per_second = Tama::tick_frequency();

    let start = timer.get_counter().ticks();
    let mut next_frame = 0u64;
    let mut reported_pause = false;
    let mut prev_switches = (false, false, false);
    let mut deb_left = Debouncer::new(DEBOUNCE_US);
    let mut deb_middle = Debouncer::new(DEBOUNCE_US);
    let mut deb_right = Debouncer::new(DEBOUNCE_US);

    info!("picogotchi started");

    loop {
        // Sample and debounce the three switches (active low), logging any
        // change so that a wiring problem is easy to spot in the RTT log.
        let now_us = timer.get_counter().ticks();
        let elapsed_us = now_us - start;

        let left = deb_left.update(bt_left.is_low().unwrap_or(false), now_us);
        let middle = deb_middle.update(bt_middle.is_low().unwrap_or(false), now_us);
        let right = deb_right.update(bt_right.is_low().unwrap_or(false), now_us);
        if (left, middle, right) != prev_switches {
            info!("switch: left={} middle={} right={}", left, middle, right);
            prev_switches = (left, middle, right);
        }

        let target_ticks = elapsed_us * ticks_per_second / 1_000_000;

        // Run the emulator until it has caught up with the wall clock.
        while tama.tick_counter() < target_ticks {
            tama.set_button(Button::Left, button_state(left));
            tama.set_button(Button::Middle, button_state(middle));
            tama.set_button(Button::Right, button_state(right));

            if !tama.run_step() {
                if !reported_pause {
                    warn!("emulation paused (unknown opcode)");
                    reported_pause = true;
                }
                break;
            }
        }

        // Render at ~3 fps.
        if elapsed_us >= next_frame {
            next_frame = elapsed_us + FRAME_PERIOD_US;
            let _ = display.clear(BinaryColor::Off);
            let _ = display::draw(&mut display, tama.lcd());
            let _ = display.flush();
        }
    }
}

fn button_state(pressed: bool) -> ButtonState {
    if pressed {
        ButtonState::Pressed
    } else {
        ButtonState::Released
    }
}
