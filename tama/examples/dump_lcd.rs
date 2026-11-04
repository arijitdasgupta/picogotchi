//! Diagnostic: run the ROM and print both the emulated LCD and the final
//! 128x64 image that gets pushed to the OLED.
//!
//! Usage (from the workspace root, so the host target is used):
//!     cargo run -q -p tama --example dump_lcd -- tama.bin

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*, Pixel};
use tama::{Button, ButtonState, Tama};

fn pack(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    for group in data.chunks_exact(4) {
        let (v1, v2, v3, v4) = (group[0] as u16, group[1] as u16, group[2] as u16, group[3] as u16);
        out.push(((v1 << 4) | ((v2 >> 4) & 0xF)) as u8);
        out.push((((v2 & 0xF) << 4) | v3) as u8);
        out.push(v4 as u8);
    }
    out
}

struct Fake {
    px: [[bool; 128]; 64],
}

impl OriginDimensions for Fake {
    fn size(&self) -> Size {
        Size::new(128, 64)
    }
}

impl DrawTarget for Fake {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            if (0..128).contains(&p.x) && (0..64).contains(&p.y) {
                self.px[p.y as usize][p.x as usize] = c.is_on();
            }
        }
        Ok(())
    }
}

fn dump(t: &Tama, label: &str) {
    println!("=== {label} ===");
    for y in 0..16 {
        print!("{y:>2} ");
        for x in 0..32 {
            print!("{}", if t.lcd().get(x, y) { '#' } else { '.' });
        }
        println!();
    }

    let mut fb = Fake { px: [[false; 128]; 64] };
    tama::display::draw(&mut fb, t.lcd()).unwrap();
    println!("--- 128x64 output ---");
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (128i32, -1i32, 64i32, -1i32);
    for y in 0..64 {
        let mut line = String::new();
        for x in 0..128 {
            let on = fb.px[y][x];
            if on {
                min_x = min_x.min(x as i32);
                max_x = max_x.max(x as i32);
                min_y = min_y.min(y as i32);
                max_y = max_y.max(y as i32);
            }
            line.push(if on { '#' } else { '.' });
        }
        println!("{y:>2} {line}");
    }
    println!("lit bounds: x {min_x}..{max_x}  y {min_y}..{max_y}");
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "tama.bin".into());
    let data = std::fs::read(&path).expect("read rom");
    let rom: &'static [u8] = Box::leak(pack(&data).into_boxed_slice());
    let mut t = Tama::new(rom);

    for _ in 0..200_000 {
        if !t.run_step() {
            println!("paused after an unknown opcode");
            break;
        }
    }
    dump(&t, "egg");

    for _ in 0..50_000 {
        t.set_button(Button::Middle, ButtonState::Pressed);
        t.run_step();
    }
    for _ in 0..400_000 {
        t.set_button(Button::Middle, ButtonState::Released);
        t.run_step();
    }
    dump(&t, "after the middle button");
}
