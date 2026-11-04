# AGENTS.md

A `no_std` Rust Tamagotchi P1 emulator for the **Raspberry Pi Pico 2 (RP2350)**
driving a 128x64 SSD1306 I2C OLED with three buttons. There is no buzzer.

It is a port of [ArduinoGotchi](https://github.com/GaryZ88/ArduinoGotchi), which
is an optimised derivative of [TamaLib](https://github.com/jcrona/tamalib). The
CPU, instruction table, LCD/icon mapping and button handling follow the reference
implementation closely; see "Deviations from the reference" below.

## Commands

```bash
# Emulation core (host, real tests) - run from the repository root
cargo test

# Firmware (RP2350) - run from firmware/
cd firmware
rustup target add thumbv8m.main-none-eabihf   # once
cargo build                                    # or `cargo run` to flash + stream defmt
cargo run                                      # flashes over SWD and streams defmt/RTT

# Release UF2 for drag-and-drop flashing
./make-uf2.sh
```

`cargo test` at the root only exercises `tama` (see "Workspace" below). There is
no lint/CI config.

## Workspace

The repository is a Cargo workspace:

- `tama/` - the portable emulation core. `no_std` (except under `cfg(test)`,
  where `std` is enabled so the test harness works), no hardware dependencies.
- `firmware/` - the RP2350 binary, `#![no_std]`/`#![no_main]`.

`default-members = ["tama"]` and the profiles live in the root `Cargo.toml`.
`firmware/.cargo/config.toml` is what selects the `thumbv8m.main-none-eabihf`
target and the `probe-rs` runner.

**Important:** Cargo discovers `.cargo/config.toml` by walking *up* from the
current directory. That is why `firmware/` has its own config and the root does
not: if the target were set at the root, `cd tama && cargo test` would try to
build the core for the MCU and fail. Do not add a root `.cargo/config.toml` with
a `[build] target`.

## Hardware

| Function    | Pin  |
| ----------- | ---- |
| I2C1 SDA    | GP2  |
| I2C1 SCL    | GP3  |
| Left switch | GP16 |
| Mid switch  | GP17 |
| Right switch| GP18 |
| LED (unused)| GP25 |

I2C1 runs at 1 MHz. Switches are active low with internal pull-ups. Pin
assignments are repeated in `firmware/src/main.rs`.

## The ROM

The Tamagotchi P1 ROM (12288 bytes) is not included. `firmware/build.rs` looks
for `rom.bin` or `tama.bin` next to `firmware/Cargo.toml` or at the workspace
root, or at `$ROM_PATH`, and packs it into the 12-bit format the CPU expects
(9216 bytes; two 12-bit words per three bytes, matching `TamaRomConvert.java`).

The packed image is emitted as `rom.rs` in `OUT_DIR` and pulled in by
`firmware/src/rom.rs` via `include!`. If no ROM is found the build warns and uses
an all-zero image so the firmware still compiles.

`tama/src/cpu.rs`'s `get_program_opcode` reads the packed words: even PC uses the
high 12 bits of a 3-byte group, odd PC the low 12 bits. The odd read picks up
unused bits from the neighbouring word, so it is masked with `0xFFF` (the
reference leaves them set and relies on the instruction masks to ignore them).

## Emulation core (`tama`)

- **`cpu.rs`** - the E0C6S46 CPU. Registers, the 320-byte nibble RAM, I/O
  registers, interrupts and timers, and a `[OpEntry; 108]` instruction table.
  `Tama::new(rom)` resets the machine; `Tama::run_step()` executes one
  instruction (mirroring `tamalib_mainloop_step_by_step`), returning `false`
  once an unknown opcode has paused execution.
- **`hw.rs`** - the LCD. `Lcd::set_pin(seg, com, val)` is `hw_set_lcd_pin`: the
  `SEG_POS` table maps segments to pixels; segments past `LCD_WIDTH` drive the
  icon row.
- **`display.rs`** - renders an `Lcd` onto any `embedded-graphics` target.
- **`debounce.rs`** - time-based switch debouncing.

### Instruction table ordering is significant

`OPS` is searched in order and the first `op & mask == code` wins, exactly like
the reference. Several entries rely on this shadowing (e.g. `SET`/`RST` cover the
specific `SCF`/`RCF`/`EI`/`DI` opcodes, and `INC_X` precedes `LDPX_R`). Do not
reorder the table without checking what is shadowed.

### Memory / I/O

`get_memory`/`set_memory` decode nibble addresses: RAM below `0x280`, display
memory at `0xE00`/`0xE80` (which drives the LCD), and I/O at `0xF00`. The I/O
handlers are ports of `get_io`/`set_io`.

### Timers and pacing

`tick_counter` advances by the instruction cycle count, but one instruction
behind (this is the reference behaviour of `wait_for_cycles`). The clock timer
(1 Hz) and programmable timer (256 Hz) fire off it. Timestamps are `u64` here
rather than the reference's `u32`, so no wrapping logic is needed.

`firmware/src/main.rs` paces the emulator to the real 32768 Hz by comparing
`tick_counter` against `elapsed_us * 32768 / 1_000_000`, running steps until it
catches up, then rendering at ~3 fps.

### Deviations from the reference

- `SBC_R_I` is encoded as `0xD40` (the architecturally correct value, as in
  upstream TamaLib). ArduinoGotchi encodes it as `0xB40`, where it is shadowed by
  `LD_X` and therefore unreachable. The P1 ROM does not appear to use it, so
  behaviour is unchanged in practice.
- The interrupt-mask register swap in `set_io` (`REG_SERIAL_INT_MASKS` ->
  `INT_K10_K13_SLOT`, `REG_K00_K03_INT_MASKS` -> `INT_SERIAL_SLOT`) is preserved
  because it is present in both the reference and upstream TamaLib. Buttons work
  by polling the `K00-K03` input port, not via those interrupts.

## Rendering

`display::draw(target, lcd)` reproduces `drawTamaSelection`/`drawTamaRow`: the
32x16 LCD is scaled 3x (2x2 blocks on a 3 px pitch) and offset 16 px to centre
it, and the eight 16x9 icons are drawn along the bottom at `y = 49` with a
selection triangle above each active icon. Icon bitmaps are XBM (LSB is the
leftmost pixel). All eight glyphs are always drawn; the reference skips the last
one unless it is active, which stops the row at x=108 and makes the whole screen
look left-aligned. With all eight drawn the row spans x=4..124, i.e. centred.

`cargo run -p tama --example dump_lcd -- <rom>` prints the emulated LCD and the
final 128x64 image as ASCII, which is the quickest way to check layout changes.

## RP2350 boot requirements (easy to break)

The Boot ROM will not boot the image without an image definition in the
`.start_block` section; `firmware/src/main.rs` provides it:

```rust
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();
```

`firmware/memory.x` places `.start_block` right after `.vector_table` so it stays
in the first 4K of flash the ROM scans (it ends up at offset `0x110`). The
`binary-info` feature on `rp235x-hal` enables the `PICOTOOL_ENTRIES` block read by
`picotool info`.

Because the firmware crate is no longer the linker's working directory, `build.rs`
copies `memory.x` into `OUT_DIR` and adds that directory to the linker search
path. Edit `firmware/memory.x`, not `build.rs`, to change the memory layout.

## Flashing

- The runner is `probe-rs run --chip RP235x`. **Pin probe-rs to 0.31.0**: 0.30.0
  loses the debug connection on RP2350 (reset sequence), and 0.32.0 has a
  flashing regression.
- `probe-rs run` never exits: it streams `defmt` over RTT until stopped.
- `firmware/make-uf2.sh` builds the release image and converts the ELF to UF2
  with `picotool` when available, otherwise with `firmware/tools/elf2uf2.py`
  (python3 only, no dependencies). The UF2 family ID for an Arm Secure image is
  `0xe48bff59`.

## Logging

`defmt` over RTT. `DEFMT_LOG = "debug"` is set in `firmware/.cargo/config.toml`.
Use `defmt::info!`, not `println!`. The firmware logs every debounced switch
transition, which is handy for ruling out wiring problems.

## Conventions

- The core crate is hardware-free; keep it that way so `cargo test` keeps working.
- Behaviour-affecting constants live at the top of the relevant module.
- `firmware/src/main.rs` is a single `#[entry]` on core0; there is no multicore.
