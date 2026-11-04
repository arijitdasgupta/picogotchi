//! Rendering of the emulated LCD onto an `embedded-graphics` draw target.
//!
//! This reproduces the layout used by ArduinoGotchi: the 32x16 Tamagotchi LCD
//! is scaled 3x and centred horizontally (leaving a 16 px margin), with the
//! eight status icons drawn along the bottom, each with a selection triangle
//! above it.

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
};

use crate::{Lcd, ICON_NUM, LCD_HEIGHT, LCD_WIDTH};

/// Horizontal offset of the scaled LCD (centres a 96 px wide image on 128 px).
const LCD_X_OFFSET: i32 = 16;
/// Scale factor applied to each Tamagotchi pixel.
const SCALE: i32 = 3;
/// Vertical position of the icon row.
const ICONS_Y: i32 = 49;

/// The eight 16x9 icon bitmaps, in XBM byte order (LSB is the leftmost pixel).
///
/// Transcribed verbatim from `bitmaps.h` in ArduinoGotchi.
#[rustfmt::skip]
pub const ICON_BITMAPS: [u8; 144] = [
    0x20, 0x00, 0x10, 0x00, 0xFE, 0x00, 0xFB, 0x01, 0xFD, 0x01, 0xFF, 0x01, 0xFF, 0x01, 0xFE, 0x00, 0x6C, 0x00,
    0x10, 0x00, 0x82, 0x00, 0x38, 0x00, 0x7C, 0x00, 0x7D, 0x01, 0x7C, 0x00, 0x38, 0x00, 0x82, 0x00, 0x10, 0x00,
    0x04, 0x00, 0x06, 0x00, 0xFF, 0x01, 0x06, 0x00, 0x44, 0x00, 0xC0, 0x00, 0xFF, 0x01, 0xC0, 0x00, 0x40, 0x00,
    0xEE, 0x00, 0x11, 0x01, 0x00, 0x01, 0x02, 0x01, 0x07, 0x01, 0x82, 0x00, 0x40, 0x00, 0x28, 0x00, 0x10, 0x00,
    0x20, 0x00, 0x02, 0x00, 0x98, 0x00, 0x1C, 0x00, 0x64, 0x00, 0xFE, 0x00, 0xFE, 0x00, 0x8F, 0x01, 0xFF, 0x01,
    0x1C, 0x00, 0x22, 0x00, 0x41, 0x00, 0x41, 0x00, 0x41, 0x00, 0x22, 0x00, 0x5C, 0x00, 0x80, 0x00, 0x00, 0x01,
    0x1E, 0x00, 0x3F, 0x00, 0x33, 0x00, 0x2F, 0x00, 0x7F, 0x00, 0x3F, 0x01, 0x9F, 0x00, 0x3E, 0x01, 0x0C, 0x00,
    0x7C, 0x00, 0x82, 0x00, 0x11, 0x01, 0x11, 0x01, 0x11, 0x01, 0x01, 0x01, 0x11, 0x01, 0x82, 0x00, 0x7C, 0x00,
];

/// Draw the whole Tamagotchi screen.
pub fn draw<D>(target: &mut D, lcd: &Lcd) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    // Main LCD matrix.
    for y in 0..LCD_HEIGHT {
        for x in 0..LCD_WIDTH {
            if lcd.get(x, y) {
                Rectangle::new(
                    Point::new(x as i32 * SCALE + LCD_X_OFFSET, y as i32 * SCALE),
                    Size::new(2, 2),
                )
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                .draw(target)?;
            }
        }
    }

    // Icon row: eight 16 px cells across the full width, so the row is centred.
    // Every glyph is drawn; a selection triangle marks the active ones.
    //
    // The reference skips the last glyph unless it is active, which leaves the
    // row spanning only x=4..108 and makes the whole screen look left-aligned.
    for i in 0..ICON_NUM {
        let x = i as i32 * 16;
        if lcd.icons[i] {
            draw_triangle(target, x + 5, ICONS_Y)?;
        }
        draw_icon(target, x + 4, ICONS_Y + 6, i)?;
    }

    Ok(())
}

/// The small downward selection triangle drawn above an active icon.
fn draw_triangle<D>(target: &mut D, x: i32, y: i32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    Line::new(Point::new(x + 1, y + 1), Point::new(x + 5, y + 1))
        .into_styled(style)
        .draw(target)?;
    Line::new(Point::new(x + 2, y + 2), Point::new(x + 4, y + 2))
        .into_styled(style)
        .draw(target)?;
    Pixel(Point::new(x + 3, y + 3), BinaryColor::On).draw(target)?;
    Ok(())
}

/// Blit one 16x9 XBM icon.
fn draw_icon<D>(target: &mut D, x: i32, y: i32, idx: usize) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    let bmp = &ICON_BITMAPS[idx * 18..idx * 18 + 18];
    for row in 0..9i32 {
        for col in 0..16i32 {
            let byte = bmp[(row as usize) * 2 + (col as usize) / 8];
            if (byte >> (col % 8)) & 1 != 0 {
                Pixel(Point::new(x + col, y + row), BinaryColor::On).draw(target)?;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 128x64 1-bit framebuffer used to inspect the rendered output.
    struct Fake {
        pixels: [[bool; 128]; 64],
    }

    impl Fake {
        fn new() -> Self {
            Self { pixels: [[false; 128]; 64] }
        }
        fn get(&self, x: usize, y: usize) -> bool {
            self.pixels[y][x]
        }
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
                    self.pixels[p.y as usize][p.x as usize] = c.is_on();
                }
            }
            Ok(())
        }
    }

    fn lit_x_bounds(f: &Fake) -> (i32, i32) {
        let (mut min_x, mut max_x) = (128i32, -1i32);
        for y in 0..64usize {
            for x in 0..128usize {
                if f.get(x, y) {
                    min_x = min_x.min(x as i32);
                    max_x = max_x.max(x as i32);
                }
            }
        }
        (min_x, max_x)
    }

    #[test]
    fn icon_row_is_centred() {
        let lcd = Lcd::new();
        let mut f = Fake::new();
        draw(&mut f, &lcd).unwrap();

        // With no LCD pixels and no active icons only the eight icon glyphs are
        // drawn. All eight must be present, otherwise the row stops short on the
        // right and the whole screen looks left-aligned.
        let (min_x, max_x) = lit_x_bounds(&f);
        assert!(min_x <= 5, "icon row starts too far right: x={min_x}");
        assert!(max_x >= 123, "icon row stops too early: x={max_x}");
        let skew = (min_x + max_x - 127).abs();
        assert!(skew <= 2, "icon row is not centred (skew {skew}): {min_x}..{max_x}");
    }

    #[test]
    fn lcd_pixel_is_scaled_and_offset() {
        let mut lcd = Lcd::new();
        lcd.set_pin(0, 0, 1); // pixel (0, 0)
        let mut f = Fake::new();
        draw(&mut f, &lcd).unwrap();

        // Pixel (0,0) becomes a 2x2 block at x = 16.
        assert!(f.get(16, 0) && f.get(17, 0) && f.get(16, 1) && f.get(17, 1));
        assert!(!f.get(15, 0));
        assert!(!f.get(18, 0));
    }

    #[test]
    fn last_lcd_column_fits_on_screen() {
        let mut lcd = Lcd::new();
        // SEG_POS[20] == 31, i.e. the rightmost LCD column.
        lcd.set_pin(20, 15, 1);
        let mut f = Fake::new();
        draw(&mut f, &lcd).unwrap();
        // x = 31*3 + 16 = 109, y = 45.
        assert!(f.get(109, 45) && f.get(110, 46));
    }

    #[test]
    fn icons_are_drawn_at_the_bottom() {
        let lcd = Lcd::new();
        let mut f = Fake::new();
        draw(&mut f, &lcd).unwrap();
        // Icon 0 always shows its glyph; row 0 byte 0 is 0x20 (col 5, LSB-first).
        assert!(f.get(4 + 5, 55));
        // No selection triangles while every icon is inactive.
        assert!(!f.get(6, 50));
    }

    #[test]
    fn active_icon_draws_triangle() {
        let mut lcd = Lcd::new();
        lcd.icons[0] = true;
        let mut f = Fake::new();
        draw(&mut f, &lcd).unwrap();
        assert!(f.get(6, 50) && f.get(8, 50) && f.get(7, 51) && f.get(8, 52));
    }
}
