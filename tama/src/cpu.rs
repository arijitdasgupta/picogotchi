//! The E0C6S46 CPU emulation.
//!
//! Port of `cpu.c` / `cpu.h` from ArduinoGotchi, which is a derivative of
//! TamaLib. The instruction table and its ordering are preserved verbatim so
//! that instruction decoding behaves exactly like the reference implementation.

use crate::hw::{Button, ButtonState, Lcd};

// ---------------------------------------------------------------------------
// Memory map
// ---------------------------------------------------------------------------

/// Size of the RAM array in bytes (each byte holds two 4-bit nibbles).
pub const MEMORY_SIZE: usize = 0x140;
/// Highest nibble address that maps into RAM.
const MEM_RAM_SIZE: u16 = 0x280;

const MEM_DISPLAY1_ADDR: u16 = 0xE00;
const MEM_DISPLAY1_SIZE: u16 = 0x050;
const MEM_DISPLAY2_ADDR: u16 = 0xE80;
const MEM_DISPLAY2_SIZE: u16 = 0x050;
const MEM_IO_ADDR: u16 = 0xF00;
const MEM_IO_SIZE: u16 = 0x080;

// ---------------------------------------------------------------------------
// Timers / interrupts
// ---------------------------------------------------------------------------

/// Tamagotchi CPU clock, in ticks per second.
const TICK_FREQUENCY: u64 = 32768;
const TIMER_1HZ_PERIOD: u64 = 32768;
const TIMER_256HZ_PERIOD: u64 = 128;

const INT_PROG_TIMER_SLOT: usize = 0;
const INT_SERIAL_SLOT: usize = 1;
const INT_K10_K13_SLOT: usize = 2;
const INT_K00_K03_SLOT: usize = 3;
const INT_STOPWATCH_SLOT: usize = 4;
const INT_CLOCK_TIMER_SLOT: usize = 5;
const INT_SLOT_NUM: usize = 6;

// Input pins (only K00-K03 are wired to buttons).
const PIN_K00: u8 = 0x0;
const PIN_K01: u8 = 0x1;
const PIN_K02: u8 = 0x2;

// ---------------------------------------------------------------------------
// Flags
// ---------------------------------------------------------------------------

const FLAG_C: u8 = 0x1 << 0;
const FLAG_Z: u8 = 0x1 << 1;
const FLAG_D: u8 = 0x1 << 2;
const FLAG_I: u8 = 0x1 << 3;

// ---------------------------------------------------------------------------
// I/O registers
// ---------------------------------------------------------------------------

const REG_CLK_INT_FACTOR_FLAGS: u16 = 0xF00;
const REG_SW_INT_FACTOR_FLAGS: u16 = 0xF01;
const REG_PROG_INT_FACTOR_FLAGS: u16 = 0xF02;
const REG_SERIAL_INT_FACTOR_FLAGS: u16 = 0xF03;
const REG_K00_K03_INT_FACTOR_FLAGS: u16 = 0xF04;
const REG_K10_K13_INT_FACTOR_FLAGS: u16 = 0xF05;
const REG_CLOCK_INT_MASKS: u16 = 0xF10;
const REG_SW_INT_MASKS: u16 = 0xF11;
const REG_PROG_INT_MASKS: u16 = 0xF12;
const REG_SERIAL_INT_MASKS: u16 = 0xF13;
const REG_K00_K03_INT_MASKS: u16 = 0xF14;
const REG_K10_K13_INT_MASKS: u16 = 0xF15;
const REG_PROG_TIMER_DATA_L: u16 = 0xF24;
const REG_PROG_TIMER_DATA_H: u16 = 0xF25;
const REG_PROG_TIMER_RELOAD_DATA_L: u16 = 0xF26;
const REG_PROG_TIMER_RELOAD_DATA_H: u16 = 0xF27;
const REG_K00_K03_INPUT_PORT: u16 = 0xF40;
const REG_K10_K13_INPUT_PORT: u16 = 0xF42;
const REG_K40_K43_BZ_OUTPUT_PORT: u16 = 0xF54;
const REG_CPU_OSC3_CTRL: u16 = 0xF70;
const REG_LCD_CTRL: u16 = 0xF71;
const REG_LCD_CONTRAST: u16 = 0xF72;
const REG_SVD_CTRL: u16 = 0xF73;
const REG_BUZZER_CTRL1: u16 = 0xF74;
const REG_BUZZER_CTRL2: u16 = 0xF75;
const REG_CLK_WD_TIMER_CTRL: u16 = 0xF76;
const REG_SW_TIMER_CTRL: u16 = 0xF77;
const REG_PROG_TIMER_CTRL: u16 = 0xF78;
const REG_PROG_TIMER_CLK_SEL: u16 = 0xF79;

// ---------------------------------------------------------------------------
// Instruction table masks
// ---------------------------------------------------------------------------

const MASK_4B: u16 = 0xF00;
const MASK_6B: u16 = 0xFC0;
const MASK_7B: u16 = 0xFE0;
const MASK_8B: u16 = 0xFF0;
const MASK_10B: u16 = 0xFFC;
const MASK_12B: u16 = 0xFFF;

// PC helpers.
const fn to_pc(bank: u16, page: u16, step: u16) -> u16 {
    (step & 0xFF) | ((page & 0xF) << 8) | ((bank & 0x1) << 12)
}

const fn to_np(bank: u16, page: u16) -> u16 {
    (page & 0xF) | ((bank & 0x1) << 4)
}

#[derive(Clone, Copy)]
struct Interrupt {
    factor_flag_reg: u8,
    mask_reg: u8,
    triggered: bool,
    vector: u8,
}

/// A single decoded instruction-table entry.
#[derive(Clone, Copy)]
struct OpEntry {
    code: u16,
    mask: u16,
    shift_arg0: u8,
    mask_arg0: u16,
    cycles: u8,
    cb: fn(&mut Tama, u8, u8),
}

/// The complete emulated machine: CPU registers, RAM, I/O and the LCD.
pub struct Tama {
    rom: &'static [u8],

    // Registers
    pc: u16,
    next_pc: u16,
    x: u16,
    y: u16,
    a: u8,
    b: u8,
    np: u8,
    sp: u8,
    flags: u8,

    memory: [u8; MEMORY_SIZE],
    inputs: [u8; 2],
    interrupts: [Interrupt; INT_SLOT_NUM],

    call_depth: u32,
    tick_counter: u64,
    clk_timer_timestamp: u64,
    prog_timer_timestamp: u64,
    prog_timer_enabled: bool,
    prog_timer_data: u8,
    prog_timer_rld: u8,
    previous_cycles: u8,

    lcd: Lcd,

    /// Set when an unknown opcode is hit; mirrors TamaLib's `EXEC_MODE_PAUSE`.
    paused: bool,
}

impl Tama {
    /// Create a new machine from a packed 12-bit ROM image.
    ///
    /// The ROM must be the output of the `TamaRomConvert` packing (two 12-bit
    /// words per three bytes), i.e. 9216 bytes for a 12288-byte P1 ROM.
    pub fn new(rom: &'static [u8]) -> Self {
        let mut tama = Self {
            rom,
            pc: 0,
            next_pc: 0,
            x: 0,
            y: 0,
            a: 0,
            b: 0,
            np: 0,
            sp: 0,
            flags: 0,
            memory: [0; MEMORY_SIZE],
            inputs: [0; 2],
            interrupts: [
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x0C },
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x0A },
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x08 },
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x06 },
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x04 },
                Interrupt { factor_flag_reg: 0, mask_reg: 0, triggered: false, vector: 0x02 },
            ],
            call_depth: 0,
            tick_counter: 0,
            clk_timer_timestamp: 0,
            prog_timer_timestamp: 0,
            prog_timer_enabled: false,
            prog_timer_data: 0,
            prog_timer_rld: 0,
            previous_cycles: 0,
            lcd: Lcd::new(),
            paused: false,
        };
        tama.reset();
        tama.hw_init();
        tama
    }

    /// Port of `cpu_reset()`.
    pub fn reset(&mut self) {
        self.pc = to_pc(0, 1, 0x00); // bank 0, page 1, step 0
        self.np = to_np(0, 1) as u8;
        self.a = 0;
        self.b = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0;
        self.flags = 0;
        self.memory = [0; MEMORY_SIZE];
        self.paused = false;
    }

    /// Port of `hw_init()`: buttons are active low, so release them all.
    fn hw_init(&mut self) {
        self.set_input_pin(PIN_K00, 1);
        self.set_input_pin(PIN_K01, 1);
        self.set_input_pin(PIN_K02, 1);
    }

    // -- Public accessors ---------------------------------------------------

    /// Emulated clock ticks elapsed. Used to pace the emulator to real time.
    pub fn tick_counter(&self) -> u64 {
        self.tick_counter
    }

    /// True once the CPU hit an opcode it does not understand.
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// The emulated LCD.
    pub fn lcd(&self) -> &Lcd {
        &self.lcd
    }

    /// True when the pet is signalling that it needs something.
    ///
    /// On the P1 the status icons are how the pet asks for attention (hungry,
    /// unhappy, wants the light, and so on), so "any icon lit" is the signal the
    /// firmware uses to keep the screen awake. Narrow this to specific icons if
    /// it turns out to keep the screen on too often.
    pub fn needs_attention(&self) -> bool {
        self.lcd.icons.iter().any(|&lit| lit)
    }

    /// Ticks per second of the emulated CPU.
    pub const fn tick_frequency() -> u64 {
        TICK_FREQUENCY
    }

    // -- Buttons ------------------------------------------------------------

    /// Port of `hw_set_button()`.
    pub fn set_button(&mut self, btn: Button, state: ButtonState) {
        let pin_state = if state == ButtonState::Pressed { 0 } else { 1 };
        let pin = match btn {
            Button::Left => PIN_K02,
            Button::Middle => PIN_K01,
            Button::Right => PIN_K00,
        };
        self.set_input_pin(pin, pin_state);
    }

    /// Port of `cpu_set_input_pin()`.
    fn set_input_pin(&mut self, pin: u8, state: u8) {
        let port = ((pin & 0x4) >> 2) as usize;
        let bit = pin & 0x3;
        self.inputs[port] = (self.inputs[port] & !(1u8 << bit)) | ((state & 1) << bit);

        if state == 0 {
            // Falling edge on a K port raises an interrupt.
            if port == 0 {
                self.generate_interrupt(INT_K00_K03_SLOT, bit);
            } else {
                self.generate_interrupt(INT_K10_K13_SLOT, bit);
            }
        }
    }

    fn generate_interrupt(&mut self, slot: usize, bit: u8) {
        self.interrupts[slot].factor_flag_reg |= 1u8 << bit;
        if self.interrupts[slot].mask_reg & (1u8 << bit) != 0 {
            self.interrupts[slot].triggered = true;
        }
    }

    // -- Register helpers ---------------------------------------------------

    #[inline]
    fn xl1(&self) -> u16 {
        self.x & 0xF
    }
    #[inline]
    fn xh1(&self) -> u16 {
        (self.x >> 4) & 0xF
    }
    #[inline]
    fn xp(&self) -> u16 {
        (self.x >> 8) & 0xF
    }
    #[inline]
    fn yl1(&self) -> u16 {
        self.y & 0xF
    }
    #[inline]
    fn yh1(&self) -> u16 {
        (self.y >> 4) & 0xF
    }
    #[inline]
    fn yp(&self) -> u16 {
        (self.y >> 8) & 0xF
    }
    #[inline]
    fn pcb(&self) -> u16 {
        (self.pc >> 12) & 0x1
    }
    #[inline]
    fn flag_c(&self) -> u8 {
        (self.flags & FLAG_C != 0) as u8
    }
    #[inline]
    fn flag_d(&self) -> bool {
        self.flags & FLAG_D != 0
    }

    #[inline]
    fn get_rq(&mut self, rq: u16) -> u8 {
        match rq & 0x3 {
            0x0 => self.a,
            0x1 => self.b,
            0x2 => self.get_memory(self.x),
            _ => self.get_memory(self.y),
        }
    }

    #[inline]
    fn set_rq(&mut self, rq: u16, v: u8) {
        match rq & 0x3 {
            0x0 => self.a = v,
            0x1 => self.b = v,
            0x2 => self.set_memory(self.x, v),
            _ => self.set_memory(self.y, v),
        }
    }

    // -- Memory / I/O -------------------------------------------------------

    /// Port of `get_memory()`. Exposed for tests.
    pub fn get_memory(&mut self, n: u16) -> u8 {
        if n < MEM_RAM_SIZE {
            let b = self.memory[(n >> 1) as usize];
            if n & 0x1 == 0 {
                b >> 4
            } else {
                b & 0x0F
            }
        } else if n >= MEM_DISPLAY1_ADDR && n < MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE {
            0
        } else if n >= MEM_DISPLAY2_ADDR && n < MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE {
            0
        } else if n >= MEM_IO_ADDR && n < MEM_IO_ADDR + MEM_IO_SIZE {
            self.get_io(n)
        } else {
            0
        }
    }

    /// Port of `set_memory()`. Exposed for tests.
    pub fn set_memory(&mut self, n: u16, v: u8) {
        if n < MEM_RAM_SIZE {
            let idx = (n >> 1) as usize;
            if n & 0x1 == 0 {
                self.memory[idx] = (self.memory[idx] & 0x0F) | (v << 4);
            } else {
                self.memory[idx] = (self.memory[idx] & 0xF0) | (v & 0x0F);
            }
        } else if n >= MEM_DISPLAY1_ADDR && n < MEM_DISPLAY1_ADDR + MEM_DISPLAY1_SIZE {
            self.set_lcd(n, v);
        } else if n >= MEM_DISPLAY2_ADDR && n < MEM_DISPLAY2_ADDR + MEM_DISPLAY2_SIZE {
            self.set_lcd(n, v);
        } else if n >= MEM_IO_ADDR && n < MEM_IO_ADDR + MEM_IO_SIZE {
            self.set_io(n, v);
        }
    }

    /// Port of `set_lcd()`: spread four pixels of a display-memory word.
    fn set_lcd(&mut self, n: u16, v: u8) {
        let seg = ((n & 0x7F) >> 1) as u8;
        let com0 = ((((n & 0x80) >> 7) * 8) + (n & 0x1) * 4) as u8;
        for i in 0..4u8 {
            self.lcd.set_pin(seg, com0 + i, (v >> i) & 0x1);
        }
    }

    /// Port of `get_io()`.
    fn get_io(&mut self, n: u16) -> u8 {
        match n {
            REG_CLK_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg;
                self.interrupts[INT_CLOCK_TIMER_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_SW_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_STOPWATCH_SLOT].factor_flag_reg;
                self.interrupts[INT_STOPWATCH_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_PROG_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg;
                self.interrupts[INT_PROG_TIMER_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_SERIAL_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_SERIAL_SLOT].factor_flag_reg;
                self.interrupts[INT_SERIAL_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_K00_K03_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_K00_K03_SLOT].factor_flag_reg;
                self.interrupts[INT_K00_K03_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_K10_K13_INT_FACTOR_FLAGS => {
                let tmp = self.interrupts[INT_K10_K13_SLOT].factor_flag_reg;
                self.interrupts[INT_K10_K13_SLOT].factor_flag_reg = 0;
                tmp
            }
            REG_CLOCK_INT_MASKS => self.interrupts[INT_CLOCK_TIMER_SLOT].mask_reg,
            REG_SW_INT_MASKS => self.interrupts[INT_STOPWATCH_SLOT].mask_reg & 0x3,
            REG_PROG_INT_MASKS => self.interrupts[INT_PROG_TIMER_SLOT].mask_reg & 0x1,
            REG_SERIAL_INT_MASKS => self.interrupts[INT_SERIAL_SLOT].mask_reg & 0x1,
            REG_K00_K03_INT_MASKS => self.interrupts[INT_K00_K03_SLOT].mask_reg,
            REG_K10_K13_INT_MASKS => self.interrupts[INT_K10_K13_SLOT].mask_reg,
            REG_PROG_TIMER_DATA_L => self.prog_timer_data & 0xF,
            REG_PROG_TIMER_DATA_H => (self.prog_timer_data >> 4) & 0xF,
            REG_PROG_TIMER_RELOAD_DATA_L => self.prog_timer_rld & 0xF,
            REG_PROG_TIMER_RELOAD_DATA_H => (self.prog_timer_rld >> 4) & 0xF,
            REG_K00_K03_INPUT_PORT => self.inputs[0],
            REG_K10_K13_INPUT_PORT => self.inputs[1],
            REG_K40_K43_BZ_OUTPUT_PORT => 0xF,
            REG_CPU_OSC3_CTRL => 0,
            REG_LCD_CTRL => 0x8,
            REG_LCD_CONTRAST => 0,
            REG_SVD_CTRL => 0,
            REG_BUZZER_CTRL1 => 0,
            REG_BUZZER_CTRL2 => 0,
            REG_CLK_WD_TIMER_CTRL => 0,
            REG_SW_TIMER_CTRL => 0,
            REG_PROG_TIMER_CTRL => self.prog_timer_enabled as u8,
            REG_PROG_TIMER_CLK_SEL => 0,
            _ => 0,
        }
    }

    /// Port of `set_io()`.
    ///
    /// Note the serial/K00-K03 mask register swap is preserved from the
    /// reference implementation so that decoding and behaviour match exactly.
    /// Buzzer registers are accepted and ignored (this build has no buzzer).
    fn set_io(&mut self, n: u16, v: u8) {
        match n {
            REG_CLOCK_INT_MASKS => self.interrupts[INT_CLOCK_TIMER_SLOT].mask_reg = v,
            REG_SW_INT_MASKS => self.interrupts[INT_STOPWATCH_SLOT].mask_reg = v,
            REG_PROG_INT_MASKS => self.interrupts[INT_PROG_TIMER_SLOT].mask_reg = v,
            REG_SERIAL_INT_MASKS => self.interrupts[INT_K10_K13_SLOT].mask_reg = v,
            REG_K00_K03_INT_MASKS => self.interrupts[INT_SERIAL_SLOT].mask_reg = v,
            REG_K10_K13_INT_MASKS => self.interrupts[INT_K10_K13_SLOT].mask_reg = v,
            REG_PROG_TIMER_RELOAD_DATA_L => {
                self.prog_timer_rld = v | (self.prog_timer_rld & 0xF0);
            }
            REG_PROG_TIMER_RELOAD_DATA_H => {
                self.prog_timer_rld = (self.prog_timer_rld & 0xF) | (v << 4);
            }
            REG_K40_K43_BZ_OUTPUT_PORT => { /* buzzer enable: ignored */ }
            REG_BUZZER_CTRL1 => { /* buzzer frequency: ignored */ }
            REG_BUZZER_CTRL2 => { /* buzzer config 2: ignored */ }
            REG_K00_K03_INPUT_PORT => { /* write not allowed */ }
            REG_CPU_OSC3_CTRL => { /* assume 32.768 kHz OSC1, OSC3 off */ }
            REG_LCD_CTRL => { /* ignored */ }
            REG_LCD_CONTRAST => { /* assume medium contrast */ }
            REG_SVD_CTRL => { /* assume battery voltage always OK */ }
            REG_CLK_WD_TIMER_CTRL => { /* watchdog ignored */ }
            REG_SW_TIMER_CTRL => { /* stopwatch ignored */ }
            REG_PROG_TIMER_CLK_SEL => { /* assume 256 Hz, output disabled */ }
            REG_PROG_TIMER_CTRL => {
                if v & 0x2 != 0 {
                    self.prog_timer_data = self.prog_timer_rld;
                }
                if (v & 0x1 != 0) && !self.prog_timer_enabled {
                    self.prog_timer_timestamp = self.tick_counter;
                }
                self.prog_timer_enabled = v & 0x1 != 0;
            }
            _ => {}
        }
    }

    // -- Execution ----------------------------------------------------------

    /// Fetch and decode the 12-bit word at `pc`.
    ///
    /// The packed ROM stores two words in three bytes, so the odd-word read
    /// picks up unused bits from the neighbouring word; they are masked off
    /// here (the reference implementation leaves them set and relies on the
    /// instruction masks to ignore them).
    fn get_program_opcode(&self, pc: u16) -> u16 {
        let i = (pc >> 1) as usize;
        if pc & 0x1 == 0 {
            ((self.rom[i * 3] as u16) << 4) | (((self.rom[i * 3 + 1] >> 4) & 0xF) as u16)
        } else {
            (((self.rom[i * 3 + 1] as u16) << 8) | (self.rom[i * 3 + 2] as u16)) & 0xFFF
        }
    }

    /// Mirrors `wait_for_cycles()` with `CPU_SPEED_RATIO == 0`: the emulation
    /// runs as fast as possible and only the tick counter advances.
    #[inline]
    fn wait_for_cycles(&mut self, cycles: u8) {
        self.tick_counter = self.tick_counter.wrapping_add(cycles as u64);
    }

    /// Execute a single instruction. Returns `true` on an unknown opcode.
    pub fn step(&mut self) -> bool {
        let op = self.get_program_opcode(self.pc);

        let mut idx = 0usize;
        while idx < OPS.len() {
            let e = OPS[idx];
            if op & e.mask == e.code {
                break;
            }
            idx += 1;
        }
        if idx >= OPS.len() {
            return true;
        }
        let entry = OPS[idx];

        self.next_pc = (self.pc + 1) & 0x1FFF;
        self.wait_for_cycles(self.previous_cycles);

        let (arg0, arg1) = if entry.mask_arg0 != 0 {
            (
                ((op & entry.mask_arg0) >> entry.shift_arg0) as u8,
                (op & !(entry.mask | entry.mask_arg0)) as u8,
            )
        } else {
            (((op & !entry.mask) >> entry.shift_arg0) as u8, 0u8)
        };
        (entry.cb)(self, arg0, arg1);

        self.pc = self.next_pc;
        self.previous_cycles = entry.cycles;

        if idx > 0 {
            // Any instruction other than PSET resets NP to the current page.
            self.np = ((self.pc >> 8) & 0x1F) as u8;
        }

        // Clock timer (1 Hz).
        if self.tick_counter.wrapping_sub(self.clk_timer_timestamp) >= TIMER_1HZ_PERIOD {
            loop {
                self.clk_timer_timestamp = self.clk_timer_timestamp.wrapping_add(TIMER_1HZ_PERIOD);
                if self.tick_counter.wrapping_sub(self.clk_timer_timestamp) < TIMER_1HZ_PERIOD {
                    break;
                }
            }
            self.generate_interrupt(INT_CLOCK_TIMER_SLOT, 3);
        }

        // Programmable timer (256 Hz).
        if self.prog_timer_enabled
            && self.tick_counter.wrapping_sub(self.prog_timer_timestamp) >= TIMER_256HZ_PERIOD
        {
            loop {
                self.prog_timer_timestamp =
                    self.prog_timer_timestamp.wrapping_add(TIMER_256HZ_PERIOD);
                self.prog_timer_data = self.prog_timer_data.wrapping_sub(1);
                if self.prog_timer_data == 0 {
                    self.prog_timer_data = self.prog_timer_rld;
                    self.generate_interrupt(INT_PROG_TIMER_SLOT, 0);
                }
                if self.tick_counter.wrapping_sub(self.prog_timer_timestamp) < TIMER_256HZ_PERIOD {
                    break;
                }
            }
        }

        // Pending interrupts (never right after a PSET).
        if (self.flags & FLAG_I) != 0 && idx > 0 {
            self.process_interrupts();
        }

        false
    }

    /// Mirrors `tamalib_mainloop_step_by_step()`'s step: runs one instruction
    /// unless execution has been paused. Returns `true` if the CPU advanced.
    pub fn run_step(&mut self) -> bool {
        if self.paused {
            return false;
        }
        if self.step() {
            self.paused = true;
            return false;
        }
        true
    }

    /// Port of `process_interrupts()`.
    fn process_interrupts(&mut self) {
        for i in 0..INT_SLOT_NUM {
            if !self.interrupts[i].triggered {
                continue;
            }
            let pcp = ((self.pc >> 8) & 0xF) as u8;
            let pcsh = ((self.pc >> 4) & 0xF) as u8;
            let pcsl = (self.pc & 0xF) as u8;
            let sp = self.sp;
            self.set_memory(sp.wrapping_sub(1) as u16, pcp);
            self.set_memory(sp.wrapping_sub(2) as u16, pcsh);
            self.set_memory(sp.wrapping_sub(3) as u16, pcsl);
            self.sp = sp.wrapping_sub(3);
            self.flags &= !FLAG_I;
            let nbp = ((self.np >> 4) & 0x1) as u16;
            self.np = to_np(nbp, 1) as u8;
            let pcb = self.pcb();
            let vector = self.interrupts[i].vector as u16;
            self.pc = to_pc(pcb, 1, vector);
            self.call_depth = self.call_depth.wrapping_add(1);

            self.wait_for_cycles(12);
            self.interrupts[i].triggered = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Instruction implementations
// ---------------------------------------------------------------------------

#[inline]
fn set_flag(t: &mut Tama, flag: u8, on: bool) {
    if on {
        t.flags |= flag;
    } else {
        t.flags &= !flag;
    }
}

fn op_pset(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.np = arg0;
}

fn op_jp(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.next_pc = arg0 as u16 | ((t.np as u16) << 8);
}

fn op_jp_c(t: &mut Tama, arg0: u8, _arg1: u8) {
    if t.flags & FLAG_C != 0 {
        t.next_pc = arg0 as u16 | ((t.np as u16) << 8);
    }
}

fn op_jp_nc(t: &mut Tama, arg0: u8, _arg1: u8) {
    if t.flags & FLAG_C == 0 {
        t.next_pc = arg0 as u16 | ((t.np as u16) << 8);
    }
}

fn op_jp_z(t: &mut Tama, arg0: u8, _arg1: u8) {
    if t.flags & FLAG_Z != 0 {
        t.next_pc = arg0 as u16 | ((t.np as u16) << 8);
    }
}

fn op_jp_nz(t: &mut Tama, arg0: u8, _arg1: u8) {
    if t.flags & FLAG_Z == 0 {
        t.next_pc = arg0 as u16 | ((t.np as u16) << 8);
    }
}

fn op_jpba(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.next_pc = (t.a as u16) | ((t.b as u16) << 4) | ((t.np as u16) << 8);
}

fn op_call(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.pc = (t.pc + 1) & 0x1FFF; // does not actually change the PC register
    let pcp = ((t.pc >> 8) & 0xF) as u8;
    let pcsh = ((t.pc >> 4) & 0xF) as u8;
    let pcsl = (t.pc & 0xF) as u8;
    let sp = t.sp;
    t.set_memory(sp.wrapping_sub(1) as u16, pcp);
    t.set_memory(sp.wrapping_sub(2) as u16, pcsh);
    t.set_memory(sp.wrapping_sub(3) as u16, pcsl);
    t.sp = sp.wrapping_sub(3);
    let pcb = t.pcb();
    let npp = (t.np & 0xF) as u16;
    t.next_pc = to_pc(pcb, npp, arg0 as u16);
    t.call_depth = t.call_depth.wrapping_add(1);
}

fn op_calz(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.pc = (t.pc + 1) & 0x1FFF;
    let pcp = ((t.pc >> 8) & 0xF) as u8;
    let pcsh = ((t.pc >> 4) & 0xF) as u8;
    let pcsl = (t.pc & 0xF) as u8;
    let sp = t.sp;
    t.set_memory(sp.wrapping_sub(1) as u16, pcp);
    t.set_memory(sp.wrapping_sub(2) as u16, pcsh);
    t.set_memory(sp.wrapping_sub(3) as u16, pcsl);
    t.sp = sp.wrapping_sub(3);
    let pcb = t.pcb();
    t.next_pc = to_pc(pcb, 0, arg0 as u16);
    t.call_depth = t.call_depth.wrapping_add(1);
}

fn pop_return(t: &mut Tama) {
    let sp = t.sp as u16;
    t.next_pc = (t.get_memory(sp) as u16)
        | ((t.get_memory(sp + 1) as u16) << 4)
        | ((t.get_memory(sp + 2) as u16) << 8)
        | (t.pcb() << 12);
    t.sp = t.sp.wrapping_add(3);
    t.call_depth = t.call_depth.wrapping_sub(1);
}

fn op_ret(t: &mut Tama, _arg0: u8, _arg1: u8) {
    pop_return(t);
}

fn op_rets(t: &mut Tama, _arg0: u8, _arg1: u8) {
    pop_return(t);
    t.next_pc = (t.pc + 1) & 0x1FFF;
}

fn op_retd(t: &mut Tama, arg0: u8, _arg1: u8) {
    pop_return(t);
    t.set_memory(t.x, arg0 & 0xF);
    t.set_memory(t.x.wrapping_add(1), ((arg0 >> 4) & 0xF) as u8);
    t.x = (t.x + 2) & 0xFFF;
}

fn op_nop5(_t: &mut Tama, _arg0: u8, _arg1: u8) {}

fn op_nop7(_t: &mut Tama, _arg0: u8, _arg1: u8) {}

fn op_halt(_t: &mut Tama, _arg0: u8, _arg1: u8) {
    // `hal_halt()` is a no-op in the reference implementation.
}

fn op_inc_x(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.x = (t.x + 1) & 0xFFF;
}

fn op_inc_y(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.y = (t.y + 1) & 0xFFF;
}

fn op_ld_x(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.x = arg0 as u16 | (t.xp() << 8);
}

fn op_ld_y(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.y = arg0 as u16 | (t.yp() << 8);
}

fn op_ld_xp_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.x = (t.x & 0xFF) | ((t.get_rq(arg0 as u16) as u16) << 8);
}

fn op_ld_xh_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.x = t.xl1() | ((t.get_rq(arg0 as u16) as u16) << 4) | (t.xp() << 8);
}

fn op_ld_xl_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.x = (t.get_rq(arg0 as u16) as u16) | (t.xh1() << 4) | (t.xp() << 8);
}

fn op_ld_yp_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.y = (t.y & 0xFF) | ((t.get_rq(arg0 as u16) as u16) << 8);
}

fn op_ld_yh_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.y = t.yl1() | ((t.get_rq(arg0 as u16) as u16) << 4) | (t.yp() << 8);
}

fn op_ld_yl_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.y = (t.get_rq(arg0 as u16) as u16) | (t.yh1() << 4) | (t.yp() << 8);
}

fn op_ld_r_xp(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.xp() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_xh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.xh1() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_xl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.xl1() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_yp(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.yp() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_yh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.yh1() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_yl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.yl1() as u8;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    t.set_rq(arg0 as u16, arg1);
}

fn op_ld_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
}

fn op_adc_xh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.xh1() as u8 + arg0 + t.flag_c();
    t.x = t.xl1() | ((tmp as u16 & 0xF) << 4) | (t.xp() << 8);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    set_flag(t, FLAG_Z, tmp & 0xF == 0);
}

fn op_adc_xl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.xl1() as u8 + arg0 + t.flag_c();
    t.x = (tmp as u16 & 0xF) | (t.xh1() << 4) | (t.xp() << 8);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    set_flag(t, FLAG_Z, tmp & 0xF == 0);
}

fn op_adc_yh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.yh1() as u8 + arg0 + t.flag_c();
    t.y = t.yl1() | ((tmp as u16 & 0xF) << 4) | (t.yp() << 8);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    set_flag(t, FLAG_Z, tmp & 0xF == 0);
}

fn op_adc_yl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.yl1() as u8 + arg0 + t.flag_c();
    t.y = (tmp as u16 & 0xF) | (t.yh1() << 4) | (t.yp() << 8);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    set_flag(t, FLAG_Z, tmp & 0xF == 0);
}

fn op_cp_xh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.xh1() as u8;
    set_flag(t, FLAG_C, v < arg0);
    set_flag(t, FLAG_Z, v == arg0);
}

fn op_cp_xl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.xl1() as u8;
    set_flag(t, FLAG_C, v < arg0);
    set_flag(t, FLAG_Z, v == arg0);
}

fn op_cp_yh(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.yh1() as u8;
    set_flag(t, FLAG_C, v < arg0);
    set_flag(t, FLAG_Z, v == arg0);
}

fn op_cp_yl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.yl1() as u8;
    set_flag(t, FLAG_C, v < arg0);
    set_flag(t, FLAG_Z, v == arg0);
}

fn op_ld_a_mn(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.a = t.get_memory(arg0 as u16);
}

fn op_ld_b_mn(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.b = t.get_memory(arg0 as u16);
}

fn op_ld_mn_a(t: &mut Tama, arg0: u8, _arg1: u8) {
    let a = t.a;
    t.set_memory(arg0 as u16, a);
}

fn op_ld_mn_b(t: &mut Tama, arg0: u8, _arg1: u8) {
    let b = t.b;
    t.set_memory(arg0 as u16, b);
}

fn op_ldpx_mx(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.set_memory(t.x, arg0);
    t.x = (t.x + 1) & 0xFFF;
}

fn op_ldpx_r(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
    t.x = (t.x + 1) & 0xFFF;
}

fn op_ldpy_my(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.set_memory(t.y, arg0);
    t.y = (t.y + 1) & 0xFFF;
}

fn op_ldpy_r(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
    t.y = (t.y + 1) & 0xFFF;
}

fn op_lbpx(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.set_memory(t.x, arg0 & 0xF);
    t.set_memory(t.x.wrapping_add(1), (arg0 >> 4) & 0xF);
    t.x = (t.x + 2) & 0xFFF;
}

fn op_set(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.flags |= arg0;
}

fn op_rst(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.flags &= arg0;
}

fn op_scf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags |= FLAG_C;
}

fn op_rcf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags &= !FLAG_C;
}

fn op_szf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags |= FLAG_Z;
}

fn op_rzf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags &= !FLAG_Z;
}

fn op_sdf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags |= FLAG_D;
}

fn op_rdf(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags &= !FLAG_D;
}

fn op_ei(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags |= FLAG_I;
}

fn op_di(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags &= !FLAG_I;
}

fn op_inc_sp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_add(1);
}

fn op_dec_sp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
}

fn op_push_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.get_rq(arg0 as u16);
    t.set_memory(t.sp as u16, v);
}

fn op_push_xp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.xp() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_xh(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.xh1() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_xl(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.xl1() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_yp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.yp() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_yh(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.yh1() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_yl(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.yl1() as u8;
    t.set_memory(t.sp as u16, v);
}

fn op_push_f(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.sp = t.sp.wrapping_sub(1);
    let v = t.flags;
    t.set_memory(t.sp as u16, v);
}

fn op_pop_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16);
    t.set_rq(arg0 as u16, v);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_xp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.x = t.xl1() | (t.xh1() << 4) | (v << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_xh(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.x = t.xl1() | (v << 4) | (t.xp() << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_xl(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.x = v | (t.xh1() << 4) | (t.xp() << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_yp(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.y = t.yl1() | (t.yh1() << 4) | (v << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_yh(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.y = t.yl1() | (v << 4) | (t.yp() << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_yl(t: &mut Tama, _arg0: u8, _arg1: u8) {
    let v = t.get_memory(t.sp as u16) as u16;
    t.y = v | (t.yh1() << 4) | (t.yp() << 8);
    t.sp = t.sp.wrapping_add(1);
}

fn op_pop_f(t: &mut Tama, _arg0: u8, _arg1: u8) {
    t.flags = t.get_memory(t.sp as u16);
    t.sp = t.sp.wrapping_add(1);
}

fn op_ld_sph_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    t.sp = (t.sp & 0xF) | (v << 4);
}

fn op_ld_spl_r(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    t.sp = v | (t.sp & 0xF0);
}

fn op_ld_r_sph(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = (t.sp >> 4) & 0xF;
    t.set_rq(arg0 as u16, v);
}

fn op_ld_r_spl(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.sp & 0xF;
    t.set_rq(arg0 as u16, v);
}

fn op_add_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t.get_rq(arg0 as u16).wrapping_add(arg1);
    if t.flag_d() {
        if tmp >= 10 {
            t.set_rq(arg0 as u16, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_rq(arg0 as u16, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_add_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t.get_rq(arg0 as u16).wrapping_add(t.get_rq(arg1 as u16));
    if t.flag_d() {
        if tmp >= 10 {
            t.set_rq(arg0 as u16, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_rq(arg0 as u16, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_adc_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t
        .get_rq(arg0 as u16)
        .wrapping_add(arg1)
        .wrapping_add(t.flag_c());
    if t.flag_d() {
        if tmp >= 10 {
            t.set_rq(arg0 as u16, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_rq(arg0 as u16, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_adc_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t
        .get_rq(arg0 as u16)
        .wrapping_add(t.get_rq(arg1 as u16))
        .wrapping_add(t.flag_c());
    if t.flag_d() {
        if tmp >= 10 {
            t.set_rq(arg0 as u16, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_rq(arg0 as u16, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_sub(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t.get_rq(arg0 as u16).wrapping_sub(t.get_rq(arg1 as u16));
    if t.flag_d() {
        if tmp >> 4 != 0 {
            t.set_rq(arg0 as u16, tmp.wrapping_sub(6) & 0xF);
        } else {
            t.set_rq(arg0 as u16, tmp);
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
    }
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_sbc_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t
        .get_rq(arg0 as u16)
        .wrapping_sub(arg1)
        .wrapping_sub(t.flag_c());
    if t.flag_d() {
        if tmp >> 4 != 0 {
            t.set_rq(arg0 as u16, tmp.wrapping_sub(6) & 0xF);
        } else {
            t.set_rq(arg0 as u16, tmp);
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
    }
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_sbc_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let tmp = t
        .get_rq(arg0 as u16)
        .wrapping_sub(t.get_rq(arg1 as u16))
        .wrapping_sub(t.flag_c());
    if t.flag_d() {
        if tmp >> 4 != 0 {
            t.set_rq(arg0 as u16, tmp.wrapping_sub(6) & 0xF);
        } else {
            t.set_rq(arg0 as u16, tmp);
        }
    } else {
        t.set_rq(arg0 as u16, tmp & 0xF);
    }
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_and_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) & arg1;
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_and_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) & t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_or_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) | arg1;
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_or_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) | t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_xor_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) ^ arg1;
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_xor_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16) ^ t.get_rq(arg1 as u16);
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_cp_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_C, v < arg1);
    set_flag(t, FLAG_Z, v == arg1);
}

fn op_cp_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    let w = t.get_rq(arg1 as u16);
    set_flag(t, FLAG_C, v < w);
    set_flag(t, FLAG_Z, v == w);
}

fn op_fan_r_i(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    set_flag(t, FLAG_Z, v & arg1 == 0);
}

fn op_fan_r_q(t: &mut Tama, arg0: u8, arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    let w = t.get_rq(arg1 as u16);
    set_flag(t, FLAG_Z, v & w == 0);
}

fn op_rlc(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    let tmp = (v << 1) | t.flag_c();
    set_flag(t, FLAG_C, v & 0x8 != 0);
    t.set_rq(arg0 as u16, tmp & 0xF);
}

fn op_rrc(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = t.get_rq(arg0 as u16);
    let tmp = (v >> 1) | (t.flag_c() << 3);
    set_flag(t, FLAG_C, v & 0x1 != 0);
    t.set_rq(arg0 as u16, tmp & 0xF);
}

fn op_inc_mn(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.get_memory(arg0 as u16).wrapping_add(1);
    t.set_memory(arg0 as u16, tmp & 0xF);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_memory(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_dec_mn(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t.get_memory(arg0 as u16).wrapping_sub(1);
    t.set_memory(arg0 as u16, tmp & 0xF);
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_memory(arg0 as u16);
    set_flag(t, FLAG_Z, v == 0);
}

fn op_acpx(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t
        .get_memory(t.x)
        .wrapping_add(t.get_rq(arg0 as u16))
        .wrapping_add(t.flag_c());
    if t.flag_d() {
        if tmp >= 10 {
            t.set_memory(t.x, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_memory(t.x, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_memory(t.x, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_memory(t.x);
    set_flag(t, FLAG_Z, v == 0);
    t.x = (t.x + 1) & 0xFFF;
}

fn op_acpy(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t
        .get_memory(t.y)
        .wrapping_add(t.get_rq(arg0 as u16))
        .wrapping_add(t.flag_c());
    if t.flag_d() {
        if tmp >= 10 {
            t.set_memory(t.y, (tmp - 10) & 0xF);
            t.flags |= FLAG_C;
        } else {
            t.set_memory(t.y, tmp);
            t.flags &= !FLAG_C;
        }
    } else {
        t.set_memory(t.y, tmp & 0xF);
        set_flag(t, FLAG_C, tmp >> 4 != 0);
    }
    let v = t.get_memory(t.y);
    set_flag(t, FLAG_Z, v == 0);
    t.y = (t.y + 1) & 0xFFF;
}

fn op_scpx(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t
        .get_memory(t.x)
        .wrapping_sub(t.get_rq(arg0 as u16))
        .wrapping_sub(t.flag_c());
    if t.flag_d() {
        if tmp >> 4 != 0 {
            t.set_memory(t.x, tmp.wrapping_sub(6) & 0xF);
        } else {
            t.set_memory(t.x, tmp);
        }
    } else {
        t.set_memory(t.x, tmp & 0xF);
    }
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_memory(t.x);
    set_flag(t, FLAG_Z, v == 0);
    t.x = (t.x + 1) & 0xFFF;
}

fn op_scpy(t: &mut Tama, arg0: u8, _arg1: u8) {
    let tmp = t
        .get_memory(t.y)
        .wrapping_sub(t.get_rq(arg0 as u16))
        .wrapping_sub(t.flag_c());
    if t.flag_d() {
        if tmp >> 4 != 0 {
            t.set_memory(t.y, tmp.wrapping_sub(6) & 0xF);
        } else {
            t.set_memory(t.y, tmp);
        }
    } else {
        t.set_memory(t.y, tmp & 0xF);
    }
    set_flag(t, FLAG_C, tmp >> 4 != 0);
    let v = t.get_memory(t.y);
    set_flag(t, FLAG_Z, v == 0);
    t.y = (t.y + 1) & 0xFFF;
}

fn op_not(t: &mut Tama, arg0: u8, _arg1: u8) {
    let v = !t.get_rq(arg0 as u16) & 0xF;
    t.set_rq(arg0 as u16, v);
    set_flag(t, FLAG_Z, v == 0);
}

// ---------------------------------------------------------------------------
// Instruction table
//
// Ordering is significant: the first entry whose `op & mask == code` wins.
// This is the ArduinoGotchi ordering. The only intentional deviation is
// SBC_R_I, which ArduinoGotchi encodes as 0xB40 (shadowed by LD_X and therefore
// unreachable); it is encoded here as the architecturally correct 0xD40.
// ---------------------------------------------------------------------------

static OPS: [OpEntry; 108] = [
    OpEntry { code: 0xE40, mask: MASK_7B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pset },
    OpEntry { code: 0x000, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jp },
    OpEntry { code: 0x200, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jp_c },
    OpEntry { code: 0x300, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jp_nc },
    OpEntry { code: 0x600, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jp_z },
    OpEntry { code: 0x700, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jp_nz },
    OpEntry { code: 0xFE8, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_jpba },
    OpEntry { code: 0x400, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_call },
    OpEntry { code: 0x500, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_calz },
    OpEntry { code: 0xFDF, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_ret },
    OpEntry { code: 0xFDE, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 12, cb: op_rets },
    OpEntry { code: 0x100, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 12, cb: op_retd },
    OpEntry { code: 0xFFB, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_nop5 },
    OpEntry { code: 0xFFF, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_nop7 },
    OpEntry { code: 0xFF8, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_halt },
    OpEntry { code: 0xEE0, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_inc_x },
    OpEntry { code: 0xEF0, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_inc_y },
    OpEntry { code: 0xB00, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_x },
    OpEntry { code: 0x800, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_y },
    OpEntry { code: 0xE80, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_xp_r },
    OpEntry { code: 0xE84, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_xh_r },
    OpEntry { code: 0xE88, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_xl_r },
    OpEntry { code: 0xE90, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_yp_r },
    OpEntry { code: 0xE94, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_yh_r },
    OpEntry { code: 0xE98, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_yl_r },
    OpEntry { code: 0xEA0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_xp },
    OpEntry { code: 0xEA4, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_xh },
    OpEntry { code: 0xEA8, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_xl },
    OpEntry { code: 0xEB0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_yp },
    OpEntry { code: 0xEB4, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_yh },
    OpEntry { code: 0xEB8, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_yl },
    OpEntry { code: 0xA00, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_adc_xh },
    OpEntry { code: 0xA10, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_adc_xl },
    OpEntry { code: 0xA20, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_adc_yh },
    OpEntry { code: 0xA30, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_adc_yl },
    OpEntry { code: 0xA40, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_cp_xh },
    OpEntry { code: 0xA50, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_cp_xl },
    OpEntry { code: 0xA60, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_cp_yh },
    OpEntry { code: 0xA70, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_cp_yl },
    OpEntry { code: 0xFA0, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_a_mn },
    OpEntry { code: 0xFB0, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_b_mn },
    OpEntry { code: 0xF80, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_mn_a },
    OpEntry { code: 0xF90, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_mn_b },
    OpEntry { code: 0xE60, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ldpx_mx },
    OpEntry { code: 0xE70, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ldpy_my },
    OpEntry { code: 0x900, mask: MASK_4B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_lbpx },
    OpEntry { code: 0xF40, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_set },
    OpEntry { code: 0xF50, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_rst },
    OpEntry { code: 0xF41, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_scf },
    OpEntry { code: 0xF5E, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_rcf },
    OpEntry { code: 0xF42, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_szf },
    OpEntry { code: 0xF5D, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_rzf },
    OpEntry { code: 0xF44, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_sdf },
    OpEntry { code: 0xF5B, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_rdf },
    OpEntry { code: 0xF48, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_ei },
    OpEntry { code: 0xF57, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_di },
    OpEntry { code: 0xFDB, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_inc_sp },
    OpEntry { code: 0xFCB, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_dec_sp },
    OpEntry { code: 0xFC0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_r },
    OpEntry { code: 0xFC4, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_xp },
    OpEntry { code: 0xFC5, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_xh },
    OpEntry { code: 0xFC6, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_xl },
    OpEntry { code: 0xFC7, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_yp },
    OpEntry { code: 0xFC8, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_yh },
    OpEntry { code: 0xFC9, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_yl },
    OpEntry { code: 0xFCA, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_push_f },
    OpEntry { code: 0xFD0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_r },
    OpEntry { code: 0xFD4, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_xp },
    OpEntry { code: 0xFD5, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_xh },
    OpEntry { code: 0xFD6, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_xl },
    OpEntry { code: 0xFD7, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_yp },
    OpEntry { code: 0xFD8, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_yh },
    OpEntry { code: 0xFD9, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_yl },
    OpEntry { code: 0xFDA, mask: MASK_12B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_pop_f },
    OpEntry { code: 0xFE0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_sph_r },
    OpEntry { code: 0xFF0, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_spl_r },
    OpEntry { code: 0xFE4, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_sph },
    OpEntry { code: 0xFF4, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_ld_r_spl },
    OpEntry { code: 0xC00, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_add_r_i },
    OpEntry { code: 0xC40, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_adc_r_i },
    OpEntry { code: 0xD40, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_sbc_r_i },
    OpEntry { code: 0xC80, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_and_r_i },
    OpEntry { code: 0xCC0, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_or_r_i },
    OpEntry { code: 0xD00, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_xor_r_i },
    OpEntry { code: 0xDC0, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_cp_r_i },
    OpEntry { code: 0xD80, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 7, cb: op_fan_r_i },
    OpEntry { code: 0xE00, mask: MASK_6B, shift_arg0: 4, mask_arg0: 0x030, cycles: 5, cb: op_ld_r_i },
    OpEntry { code: 0xA80, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_add_r_q },
    OpEntry { code: 0xA90, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_adc_r_q },
    OpEntry { code: 0xAA0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_sub },
    OpEntry { code: 0xAB0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_sbc_r_q },
    OpEntry { code: 0xAC0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_and_r_q },
    OpEntry { code: 0xAD0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_or_r_q },
    OpEntry { code: 0xAE0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_xor_r_q },
    OpEntry { code: 0xEC0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 5, cb: op_ld_r_q },
    OpEntry { code: 0xEE0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 5, cb: op_ldpx_r },
    OpEntry { code: 0xEF0, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 5, cb: op_ldpy_r },
    OpEntry { code: 0xF00, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_cp_r_q },
    OpEntry { code: 0xF10, mask: MASK_8B, shift_arg0: 2, mask_arg0: 0x00C, cycles: 7, cb: op_fan_r_q },
    OpEntry { code: 0xAF0, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_rlc },
    OpEntry { code: 0xE8C, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 5, cb: op_rrc },
    OpEntry { code: 0xF60, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_inc_mn },
    OpEntry { code: 0xF70, mask: MASK_8B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_dec_mn },
    OpEntry { code: 0xF28, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_acpx },
    OpEntry { code: 0xF2C, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_acpy },
    OpEntry { code: 0xF38, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_scpx },
    OpEntry { code: 0xF3C, mask: MASK_10B, shift_arg0: 0, mask_arg0: 0, cycles: 7, cb: op_scpy },
    OpEntry { code: 0xD0F, mask: 0xFCF, shift_arg0: 4, mask_arg0: 0, cycles: 7, cb: op_not },
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hw::{Button, ButtonState};

    /// Pack 12-bit words into the same 3-bytes-per-2-words layout produced by
    /// `TamaRomConvert` / the firmware build script.
    fn pack(words: &[u16]) -> &'static [u8] {
        let mut out: Vec<u8> = Vec::new();
        for pair in words.chunks(2) {
            let w0 = pair[0];
            let w1 = if pair.len() > 1 { pair[1] } else { 0 };
            out.push(((w0 >> 4) & 0xFF) as u8);
            out.push((((w0 & 0xF) << 4) | ((w1 >> 8) & 0xF)) as u8);
            out.push((w1 & 0xFF) as u8);
        }
        Box::leak(out.into_boxed_slice())
    }

    /// Build a 6144-word ROM with `prog` placed at word address 0x100, which is
    /// where the CPU starts (`pc = bank 0, page 1, step 0`).
    fn rom_with(prog: &[u16]) -> &'static [u8] {
        let mut words = vec![0u16; 0x1800];
        words[0x100..0x100 + prog.len()].copy_from_slice(prog);
        pack(&words)
    }

    #[test]
    fn program_opcode_packing() {
        let rom = rom_with(&[0xE05, 0xF80, 0xABC]);
        let t = Tama::new(rom);
        assert_eq!(t.get_program_opcode(0x100), 0xE05);
        assert_eq!(t.get_program_opcode(0x101), 0xF80);
        assert_eq!(t.get_program_opcode(0x102), 0xABC);
    }

    #[test]
    fn load_immediate_then_store() {
        // LD A, 5 ; LD M0, A
        let rom = rom_with(&[0xE05, 0xF80]);
        let mut t = Tama::new(rom);
        assert!(t.run_step());
        assert!(t.run_step());
        assert_eq!(t.get_memory(0), 5);
    }

    #[test]
    fn add_immediate_then_store() {
        // LD A, 5 ; ADD A, 3 ; LD M0, A
        let rom = rom_with(&[0xE05, 0xC03, 0xF80]);
        let mut t = Tama::new(rom);
        for _ in 0..3 {
            assert!(t.run_step());
        }
        assert_eq!(t.get_memory(0), 8);
    }

    #[test]
    fn jump_uses_current_page() {
        // JP 0x10 -> 0x10 | (np << 8), with np = 1 -> 0x110
        let rom = rom_with(&[0x010]);
        let mut t = Tama::new(rom);
        assert!(t.run_step());
        assert_eq!(t.pc, 0x110);
    }

    #[test]
    fn call_then_ret_round_trips() {
        // At 0x100: CALL 0x10 (to 0x110). At 0x110: RET (back to 0x101).
        let mut words = vec![0u16; 0x1800];
        words[0x100] = 0x410; // CALL 0x10
        words[0x110] = 0xFDF; // RET
        let rom = pack(&words);
        let mut t = Tama::new(rom);
        assert!(t.run_step()); // CALL
        assert_eq!(t.pc, 0x110);
        assert!(t.run_step()); // RET
        assert_eq!(t.pc, 0x101);
    }

    #[test]
    fn unknown_opcode_pauses() {
        let rom = rom_with(&[0xE9C]);
        let mut t = Tama::new(rom);
        assert!(t.step(), "expected step() to report an unknown opcode");
        assert!(!t.run_step());
        assert!(t.is_paused());
    }

    #[test]
    fn lcd_write_lights_pixels() {
        let rom = rom_with(&[0xE05]);
        let mut t = Tama::new(rom);
        // Display memory word 0xE00 -> segment 0, com 0..3.
        t.set_memory(0xE00, 0xF);
        assert!(t.lcd().get(0, 0));
        assert!(t.lcd().get(0, 1));
        assert!(t.lcd().get(0, 2));
        assert!(t.lcd().get(0, 3));
        assert!(!t.lcd().get(1, 0));
    }

    #[test]
    fn icon_segment_lights_icons() {
        let rom = rom_with(&[0xE05]);
        let mut t = Tama::new(rom);
        // Segment 8 (display word 0xE10), com 0..3 -> icons 0..3.
        t.set_memory(0xE10, 0xF);
        assert!(t.lcd().icons[0]);
        assert!(t.lcd().icons[1]);
        assert!(t.lcd().icons[2]);
        assert!(t.lcd().icons[3]);
        assert!(!t.lcd().icons[4]);
    }

    #[test]
    fn attention_follows_the_icon_row() {
        let rom = rom_with(&[0xE05]);
        let mut t = Tama::new(rom);
        assert!(!t.needs_attention());

        // Segment 8 (display word 0xE10), com 0 -> icon 0 lit.
        t.set_memory(0xE10, 0x1);
        assert!(t.needs_attention());

        // Clearing the icon clears the attention flag again.
        t.set_memory(0xE10, 0x0);
        assert!(!t.needs_attention());
    }

    #[test]
    fn buttons_are_active_low_on_port_k00_k03() {
        let rom = rom_with(&[0xE05]);
        let mut t = Tama::new(rom);
        // All released: K00..K02 read high.
        assert_eq!(t.get_memory(REG_K00_K03_INPUT_PORT) & 0x7, 0b111);
        t.set_button(Button::Left, ButtonState::Pressed); // K02
        assert_eq!(t.get_memory(REG_K00_K03_INPUT_PORT) & 0x7, 0b011);
        t.set_button(Button::Right, ButtonState::Pressed); // K00
        assert_eq!(t.get_memory(REG_K00_K03_INPUT_PORT) & 0x7, 0b010);
        t.set_button(Button::Middle, ButtonState::Pressed); // K01
        assert_eq!(t.get_memory(REG_K00_K03_INPUT_PORT) & 0x7, 0b000);
    }

    #[test]
    fn tick_counter_advances_with_execution() {
        let rom = rom_with(&[0xE05, 0xE05, 0xE05]);
        let mut t = Tama::new(rom);
        assert_eq!(t.tick_counter(), 0);
        t.run_step();
        // `wait_for_cycles` is one instruction behind, so the first step adds 0.
        assert_eq!(t.tick_counter(), 0);
        t.run_step();
        assert_eq!(t.tick_counter(), 5);
        t.run_step();
        assert_eq!(t.tick_counter(), 10);
    }
}

