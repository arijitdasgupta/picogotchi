# Picogotchi

## A Tamagotchi P1 emulator for the Raspberry Pi Pico 2

### Running on RP2350 (Raspberry Pi Pico 2) with an SSD1306 128 x 64 display

A Rust port of [ArduinoGotchi](https://github.com/GaryZ88/ArduinoGotchi), which
is itself a heavily optimised derivative of
[TamaLib](https://github.com/jcrona/tamalib). The E0C6S46 CPU, its instruction
set, the LCD/icon mapping and the button handling are all ported, so it runs the
same Tamagotchi P1 ROM as the Arduino version.

## Hardware

| Function    | Pico 2 pin |
| ----------- | ---------- |
| OLED SDA    | GP2        |
| OLED SCL    | GP3        |
| Left switch | GP16       |
| Mid switch  | GP17       |
| Right switch| GP18       |
| LED (unused)| GP25       |

The OLED is an I2C SSD1306 128x64 running on `I2C1` at 1 MHz. The three switches
are active low (internal pull-ups): wire each one between its pin and GND. There
is **no buzzer**.

## The ROM

The emulator needs a Tamagotchi P1 ROM (12288 bytes). It is not included. Put it
at the repository root as `tama.bin` (or `rom.bin`), or next to
`firmware/Cargo.toml`, or point `ROM_PATH` at it:

```bash
cp /path/to/rom.bin tama.bin      # repository root
# or
ROM_PATH=/path/to/rom.bin cargo build
```

`firmware/build.rs` packs it into the 12-bit format the CPU expects (two 12-bit
words per three bytes, exactly like ArduinoGotchi's `TamaRomConvert`). If no ROM
is found the build still succeeds with an empty ROM, so the firmware compiles but
the pet will not run.

## Development

The emulation core is a separate, host-testable crate, so there is a real test
suite:

```bash
cargo test                      # from the repository root: core crate, on the host
```

Flash and run over a debug probe (e.g. a Picoprobe), streaming `defmt` logs over
RTT:

```bash
cd firmware
rustup target add thumbv8m.main-none-eabihf
cargo install probe-rs-tools --version 0.31.0 --locked   # see note below
cargo run
```

`cargo run` uses `probe-rs run --chip RP235x`. **Pin probe-rs to 0.31.0**:
0.30.0 has a flawed RP2350 reset sequence and 0.32.0 has a flashing regression.
Wire a Picoprobe with Probe GP2 -> target SWCLK, Probe GP3 -> target SWDIO, plus
a shared GND.

## Production build (UF2)

To flash the final board by drag-and-drop, build a UF2:

```bash
cd firmware
./make-uf2.sh
```

That does a release build and converts the ELF to a UF2 at
`target/thumbv8m.main-none-eabihf/release/picogotchi.uf2`. It uses `picotool`
when available (`brew install picotool`) and otherwise falls back to the bundled,
dependency-free converter `firmware/tools/elf2uf2.py` (needs only `python3`).

To flash: hold **BOOTSEL** while plugging the board in, then copy the `.uf2` onto
the `RPI-RP2` mass storage device. The board reboots and starts the emulator.

Unlike a plain `cargo build --release`, this build compiles the defmt log calls
out entirely (`DEFMT_LOG=off`) and reboots on panic instead of halting, so a
fault self-heals rather than leaving the pet dead until it is power-cycled. The
pet's state is RAM-only, so a panic costs a fresh egg.

## Saved state

The pet is saved to flash so it survives a power cycle. Production writes once
an hour; development writes every two minutes, so save and restore can be
exercised in a single session.

The last 32 KiB of flash is reserved for it, carved out of the linker's `FLASH`
region in `firmware/memory.x` so code can never be placed in a sector the
storage layer erases. It is a ring of eight 4 KiB sectors: each save writes one
512-byte record (magic, sequence, version, CRC, then the state) to the next
sector, so a sector is erased once per eight saves. At an hourly cadence that is
~91 years of flash life; even at the development cadence it is years.

**The save survives a firmware update.** A UF2 only carries the application
sectors (`0x10000000`-`0x1000B200`), so the bootloader never touches the save
region, and updating the firmware keeps your pet.

To start fresh in development, use `firmware/flash-clean.sh`, which erases the
flash before flashing so no stale pet is restored.

## Notes

- The screen refreshes at ~3 fps, matching the reference implementation.
- The emulated CPU is paced to its real 32 768 Hz clock using the 1 MHz timer, so
  the pet ages at the correct rate.
- Inputs are debounced with a 20 ms window (`firmware/src/main.rs`).
- The panel blanks after a minute with no input, and only while the pet is not
  signalling that it needs something. Press any switch to wake it; that press is
  consumed, so the next one acts. The pet keeps running while blanked.
- The core sleeps between emulator batches rather than spinning, and the on-board
  LED is off.
- In the egg state only the **middle** switch does anything: press it to enter
  clock setup, then use left/right to set the time. Until the clock is set the
  pet does not hatch, which can look like the other switches doing nothing.

## Layout

```
Cargo.toml            workspace (profiles live here)
tama/                 portable emulation core (no_std, host-testable)
  cpu.rs              E0C6S46 CPU and instruction table
  hw.rs               LCD matrix, icons, buttons
  display.rs          rendering of the LCD onto an embedded-graphics target
  debounce.rs         switch debouncing
firmware/             RP2350 binary
  src/main.rs         hardware bring-up, pacing loop, rendering
  build.rs            memory.x staging + ROM packing
  tools/elf2uf2.py    dependency-free ELF -> UF2 converter
  make-uf2.sh         release build + UF2
```

## License

Picogotchi is a derivative of ArduinoGotchi and TamaLib, both distributed under
the GNU General Public License v2, and is therefore also distributed under the
GPLv2. See `LICENSE`.
