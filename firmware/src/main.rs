#![no_std]
#![no_main]

//! Picogotchi: a Tamagotchi P1 emulator for the Raspberry Pi Pico 2 (RP2350)
//! driving a 128x64 SSD1306 I2C OLED with three buttons.
//!
//! The hardware wiring mirrors the `bike_w_friends` project:
//!   * I2C1 on GP2 (SDA) / GP3 (SCL) at 1 MHz
//!   * buttons on GP16 / GP17 / GP18 (pull-up, active low)
//!   * the on-board LED on GP25 is driven low and unused
//!
//! There is no buzzer.

mod rom;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use critical_section::Mutex;
use cortex_m::peripheral::NVIC;
use defmt::{info, warn};
use defmt_rtt as _;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal::digital::{InputPin, OutputPin};
use hal::fugit::{MicrosDurationU32, RateExtU32};
use panic_probe as _;
use rp235x_hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    entry,
    gpio::{self, FunctionI2c, Pin},
    pac::{self, interrupt},
    timer::{Alarm, Alarm0, CopyableTimer0, Timer},
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

/// How long the panel stays lit after the last user input. Also the grace
/// period after boot, so the egg (and the mandatory clock setup) is visible.
const IDLE_TIMEOUT_US: u64 = 60 * 1_000_000;

/// How long the panel stays lit after the pet stops signalling that it needs
/// something. While the signal persists this is re-armed on every loop, so it is
/// only the tail.
const ATTENTION_HOLD_US: u64 = 60 * 1_000_000;

/// How long the core sleeps between emulator batches while the panel is lit.
/// Small enough that button sampling stays fine-grained and the ~3 fps frame
/// deadline is never missed.
const LIT_BATCH_US: u32 = 1_000;

/// How long the core sleeps between batches while the panel is blanked. The
/// pet's finest timer is 1 Hz and the switches are debounced over 20 ms, so a
/// 10 ms sample rate still registers a press.
const DARK_BATCH_US: u32 = 10_000;

/// The TIMER0 alarm that wakes the core out of `wfi`. It is shared with the
/// interrupt handler, which is the only thing that re-arms it.
static WAKE_ALARM: Mutex<RefCell<Option<Alarm0<CopyableTimer0>>>> = Mutex::new(RefCell::new(None));

/// Wake-up period in microseconds. Written by the main loop, read by the alarm
/// handler when it re-arms.
static WAKE_PERIOD_US: AtomicU32 = AtomicU32::new(LIT_BATCH_US);

/// The firmware's only interrupt: clear the alarm flag and re-arm it.
///
/// The alarm free-runs, so `wfi` always has a future wake-up armed and can never
/// sleep forever (arming once per loop would lose a wake-up whenever the alarm
/// fired between arming and sleeping). The emulator work happens in the main
/// loop, not here.
#[interrupt]
fn TIMER0_IRQ_0() {
    critical_section::with(|cs| {
        if let Some(alarm) = WAKE_ALARM.borrow(cs).borrow_mut().as_mut() {
            alarm.clear_interrupt();
            let period = WAKE_PERIOD_US.load(Ordering::Relaxed);
            let _ = alarm.schedule(MicrosDurationU32::micros(period));
        }
    });
}

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

    // The on-board LED (GP25) is left off. Driving it high would draw a couple
    // of milliamps continuously, which is more than the MCU costs when idle.
    let mut led = pins.gpio25.into_push_pull_output();
    let _ = led.set_low();

    // Buttons: active low, internal pull-ups.
    let mut bt_left = pins.gpio16.into_pull_up_input();
    let mut bt_middle = pins.gpio17.into_pull_up_input();
    let mut bt_right = pins.gpio18.into_pull_up_input();

    // The 1 MHz microsecond timer paces the emulated CPU to real time. Its
    // alarm 0 is also the wake-up source that lets the core sleep between
    // batches instead of spinning.
    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut wake_alarm = timer.alarm_0().unwrap();
    wake_alarm.enable_interrupt();
    let _ = wake_alarm.schedule(MicrosDurationU32::micros(LIT_BATCH_US));
    critical_section::with(|cs| {
        WAKE_ALARM.borrow(cs).replace(Some(wake_alarm));
    });
    unsafe {
        NVIC::unmask(pac::Interrupt::TIMER0_IRQ_0);
    }

    let mut tama = Tama::new(&rom::G_PROGRAM_B12);
    let ticks_per_second = Tama::tick_frequency();

    let start = timer.get_counter().ticks();
    let mut next_frame = 0u64;
    let mut reported_pause = false;
    let mut prev_switches = (false, false, false);
    let mut deb_left = Debouncer::new(DEBOUNCE_US);
    let mut deb_middle = Debouncer::new(DEBOUNCE_US);
    let mut deb_right = Debouncer::new(DEBOUNCE_US);

    // Display power management. The emulator always keeps running, so the pet's
    // clock and ageing are unaffected while the panel is blanked.
    let mut awake_until_us = start + IDLE_TIMEOUT_US;
    let mut display_on = true;
    // The press that wakes the panel is consumed, so you never act on a screen
    // you could not see. Input resumes once every switch is released.
    let mut suppress_until_release = false;
    let mut force_render = false;

    info!("picogotchi started");

    loop {
        // Sample and debounce the three switches (active low), logging any
        // change so that a wiring problem is easy to spot in the RTT log.
        let now_us = timer.get_counter().ticks();
        let elapsed_us = now_us - start;

        let left = deb_left.update(bt_left.is_low().unwrap_or(false), now_us);
        let middle = deb_middle.update(bt_middle.is_low().unwrap_or(false), now_us);
        let right = deb_right.update(bt_right.is_low().unwrap_or(false), now_us);
        let any_pressed = left || middle || right;
        if (left, middle, right) != prev_switches {
            info!("switch: left={} middle={} right={}", left, middle, right);
            prev_switches = (left, middle, right);
        }

        // Input and attention both extend how long the panel stays lit.
        let attention = tama.needs_attention();
        if any_pressed {
            awake_until_us = now_us + IDLE_TIMEOUT_US;
        }
        if attention {
            awake_until_us = now_us + ATTENTION_HOLD_US;
        }
        if suppress_until_release && !any_pressed {
            suppress_until_release = false;
        }

        // Wake, blanking only once nothing needs us and there has been no input.
        if !display_on && (any_pressed || attention) {
            display_on = true;
            force_render = true;
            suppress_until_release = any_pressed;
            let _ = display.set_display_on(true);
            info!("display on");
        } else if display_on && !attention && now_us >= awake_until_us {
            display_on = false;
            let _ = display.set_display_on(false);
            info!("display off");
        }

        // While the waking press is being consumed the emulator sees nothing.
        let feed = if suppress_until_release {
            (false, false, false)
        } else {
            (left, middle, right)
        };

        let target_ticks = elapsed_us * ticks_per_second / 1_000_000;

        // Run the emulator until it has caught up with the wall clock. This
        // happens whether or not the panel is lit.
        while tama.tick_counter() < target_ticks {
            tama.set_button(Button::Left, button_state(feed.0));
            tama.set_button(Button::Middle, button_state(feed.1));
            tama.set_button(Button::Right, button_state(feed.2));

            if !tama.run_step() {
                if !reported_pause {
                    warn!("emulation paused (unknown opcode)");
                    reported_pause = true;
                }
                break;
            }
        }

        // Render at ~3 fps, and immediately after the panel comes back on.
        if display_on && (force_render || elapsed_us >= next_frame) {
            force_render = false;
            next_frame = elapsed_us + FRAME_PERIOD_US;
            let _ = display.clear(BinaryColor::Off);
            let _ = display::draw(&mut display, tama.lcd());
            let _ = display.flush();
        }

        // Sleep until the next batch rather than spinning. The wake-up is
        // handled by the TIMER0 alarm (see `TIMER0_IRQ_0`), which re-arms itself
        // with this period.
        WAKE_PERIOD_US.store(
            if display_on { LIT_BATCH_US } else { DARK_BATCH_US },
            Ordering::Relaxed,
        );
        cortex_m::asm::wfi();
    }
}

fn button_state(pressed: bool) -> ButtonState {
    if pressed {
        ButtonState::Pressed
    } else {
        ButtonState::Released
    }
}
