//! Emulated Tamagotchi P1 hardware: the LCD segment matrix, the icon row and
//! the button state.
//!
//! Port of `hw.c` / `hw.h` from ArduinoGotchi.

/// Width of the Tamagotchi P1 LCD in pixels.
pub const LCD_WIDTH: usize = 32;
/// Height of the Tamagotchi P1 LCD in pixels.
pub const LCD_HEIGHT: usize = 16;
/// Number of status icons below the main LCD.
pub const ICON_NUM: usize = 8;

/// Number of bytes in the LCD matrix buffer (8 horizontal pixels per byte).
const MATRIX_BYTES: usize = LCD_HEIGHT * (LCD_WIDTH / 8);

/// SEG -> LCD position mapping (from `hw.c`).
const SEG_POS: [u8; 40] = [
    0, 1, 2, 3, 4, 5, 6, 7, 32, 8, 9, 10, 11, 12, 13, 14, 15, 33, 34, 35, 31, 30, 29, 28, 27, 26,
    25, 24, 36, 23, 22, 21, 20, 19, 18, 17, 16, 37, 38, 39,
];

/// The three Tamagotchi buttons.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Button {
    Left,
    Middle,
    Right,
}

/// Button state. Buttons are active-low on the real device.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ButtonState {
    Released,
    Pressed,
}

/// The emulated LCD: the 32x16 pixel matrix plus the 8 status icons.
pub struct Lcd {
    /// One bit per pixel, `0x80 >> (x % 8)` inside `matrix[y * 4 + x / 8]`.
    pub matrix: [u8; MATRIX_BYTES],
    /// Icon row, `true` when the icon is lit.
    pub icons: [bool; ICON_NUM],
}

impl Lcd {
    pub const fn new() -> Self {
        Self {
            matrix: [0; MATRIX_BYTES],
            icons: [false; ICON_NUM],
        }
    }

    pub fn clear(&mut self) {
        self.matrix = [0; MATRIX_BYTES];
        self.icons = [false; ICON_NUM];
    }

    /// Is pixel `(x, y)` lit?
    #[inline]
    pub fn get(&self, x: usize, y: usize) -> bool {
        self.matrix[y * (LCD_WIDTH / 8) + x / 8] & (0x80u8 >> (x % 8)) != 0
    }

    /// Port of `hw_set_lcd_pin()`: drive one LCD segment/pin.
    ///
    /// Positions `>= LCD_WIDTH` are not pixels; they are the icon row, which is
    /// wired to a couple of spare segments.
    pub fn set_pin(&mut self, seg: u8, com: u8, val: u8) {
        let pos = SEG_POS[seg as usize];
        if (pos as usize) < LCD_WIDTH {
            self.set_matrix(pos, com, val);
        } else if seg == 8 && com < 4 {
            self.icons[com as usize] = val != 0;
        } else if seg == 28 && com >= 12 {
            self.icons[(com - 8) as usize] = val != 0;
        }
    }

    #[inline]
    fn set_matrix(&mut self, x: u8, y: u8, val: u8) {
        let idx = y as usize * (LCD_WIDTH / 8) + (x as usize) / 8;
        let mask = 0x80u8 >> (x % 8);
        if val != 0 {
            self.matrix[idx] |= mask;
        } else {
            self.matrix[idx] &= !mask;
        }
    }
}

impl Default for Lcd {
    fn default() -> Self {
        Self::new()
    }
}
