//! Build script.
//!
//! Two jobs:
//!
//! 1. Stage `memory.x` so the linker can always find it. The linker searches
//!    the crate root by default, which is no longer the current directory now
//!    that the firmware lives in a subdirectory of the workspace, so the file
//!    is copied into `OUT_DIR` and that directory is added to the linker search
//!    path.
//!
//! 2. Convert a Tamagotchi P1 ROM into a packed 12-bit image. Drop a
//!    12288-byte ROM named `rom.bin`, `tama.bin` or `test.bin` next to this
//!    crate's `Cargo.toml` or at the workspace root (or point `ROM_PATH` at
//!    one) and it is packed into two 12-bit words per three bytes, exactly like
//!    ArduinoGotchi's `TamaRomConvert` tool, then emitted as `rom.rs`.
//!
//! If no ROM is found the build still succeeds with an empty (all-zero) ROM so
//! that the firmware compiles; the emulator simply will not run a program.

use std::{env, fs, path::PathBuf};

const ROM_BYTES: usize = 12288;
const PACKED_BYTES: usize = 9216;
/// Candidate ROM locations, relative to the firmware crate, in priority order.
/// The `../` entries let the ROM live next to the workspace root `Cargo.toml`.
/// `ROM_PATH` overrides the whole list.
const ROM_NAMES: [&str; 6] = [
    "rom.bin",
    "tama.bin",
    "test.bin",
    "../rom.bin",
    "../tama.bin",
    "../test.bin",
];

fn main() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));

    // --- memory.x ---------------------------------------------------------
    fs::write(out_dir.join("memory.x"), include_bytes!("memory.x"))
        .expect("failed to copy memory.x into OUT_DIR");
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");

    // --- ROM --------------------------------------------------------------
    for name in ROM_NAMES {
        println!("cargo:rerun-if-changed={name}");
    }
    println!("cargo:rerun-if-env-changed=ROM_PATH");

    // An explicit ROM_PATH wins; otherwise use the first candidate that exists.
    let rom_path = env::var("ROM_PATH")
        .ok()
        .or_else(|| ROM_NAMES.iter().find(|n| fs::metadata(n).is_ok()).map(|n| n.to_string()));

    let packed = match rom_path.as_deref().map(fs::read) {
        Some(Ok(data)) if data.len() == ROM_BYTES => pack(&data),
        Some(Ok(data)) => {
            println!(
                "cargo:warning=ROM '{}' is {} bytes, expected {ROM_BYTES}; building with an empty ROM",
                rom_path.as_deref().unwrap_or(""),
                data.len()
            );
            vec![0u8; PACKED_BYTES]
        }
        _ => {
            println!(
                "cargo:warning=no Tamagotchi ROM found (looked for {ROM_NAMES:?}); building with an empty ROM. Put a 12288-byte P1 ROM next to Cargo.toml or set ROM_PATH."
            );
            vec![0u8; PACKED_BYTES]
        }
    };

    let mut src = String::with_capacity(packed.len() * 6 + 64);
    src.push_str("/// Packed Tamagotchi P1 ROM: two 12-bit words per three bytes.\n");
    src.push_str("pub static G_PROGRAM_B12: [u8; ");
    src.push_str(&packed.len().to_string());
    src.push_str("] = [\n");
    for chunk in packed.chunks(12) {
        src.push_str("    ");
        for b in chunk {
            src.push_str(&format!("0x{b:02X}, "));
        }
        src.push('\n');
    }
    src.push_str("];\n");

    fs::write(out_dir.join("rom.rs"), src).expect("failed to write rom.rs");
}

/// Repack a raw P1 ROM exactly like `TamaRomConvert.java`.
fn pack(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(PACKED_BYTES);
    for group in data.chunks_exact(4) {
        let v1 = group[0] as u16;
        let v2 = group[1] as u16;
        let v3 = group[2] as u16;
        let v4 = group[3] as u16;
        out.push(((v1 << 4) | ((v2 >> 4) & 0xF)) as u8);
        out.push((((v2 & 0xF) << 4) | v3) as u8);
        out.push(v4 as u8);
    }
    out
}
