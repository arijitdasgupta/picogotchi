#!/bin/sh
# Build a release UF2 image for drag-and-drop flashing onto the RP2350.
#
# Put the board into USB boot mode (hold BOOTSEL while plugging it in), then
# copy the resulting .uf2 onto the RPI-RP2 mass storage device.
#
# Uses `picotool` when it is on PATH, otherwise falls back to the bundled,
# dependency-free converter in tools/elf2uf2.py.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/.." && pwd)
out_dir="$root/target/thumbv8m.main-none-eabihf/release"
elf="$out_dir/picogotchi"
uf2="$out_dir/picogotchi.uf2"

cd "$here"
# Production build: DEFMT_LOG=off compiles the defmt log calls out entirely
# (cargo's [env] is not profile-scoped, so it has to be set here), and the
# `production` feature reboots on panic instead of halting.
DEFMT_LOG=off cargo build --release --features production

if command -v picotool >/dev/null 2>&1; then
    picotool uf2 convert -t elf "$elf" "$uf2"
else
    echo "picotool not found; using tools/elf2uf2.py" >&2
    python3 "$here/tools/elf2uf2.py" "$elf" "$uf2"
fi

echo
echo "UF2 ready: $uf2"
