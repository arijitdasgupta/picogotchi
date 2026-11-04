//! Tamagotchi P1 (E0C6S46) emulation core.
//!
//! This is a Rust port of the emulation core used by
//! [ArduinoGotchi](https://github.com/GaryZ88/ArduinoGotchi), which in turn is
//! a heavily optimised derivative of
//! [TamaLib](https://github.com/jcrona/tamalib).
//!
//! The crate is `no_std` and free of any hardware dependency, so it can run on
//! the RP2350 firmware *and* be exercised on the host with `cargo test`.
//!
//! Typical use:
//!
//! ```ignore
//! let mut tama = Tama::new(&PACKED_ROM);
//! // every emulated step:
//! tama.run_step();
//! // when a button changes:
//! tama.set_button(Button::Left, ButtonState::Pressed);
//! // when rendering:
//! tama::display::draw(&mut oled, tama.lcd());
//! ```

// `std` is only pulled in for the host test harness.
#![cfg_attr(not(test), no_std)]

mod cpu;
mod hw;

pub mod debounce;
pub mod display;

pub use cpu::{Tama, STATE_LEN};
pub use hw::{Button, ButtonState, Lcd, ICON_NUM, LCD_HEIGHT, LCD_WIDTH};
