#!/usr/bin/env python3
"""Convert an RP2350 ELF into a UF2 image, with no external dependencies.

This exists so that a drag-and-drop UF2 can be produced without installing
`picotool`. The output matches what `picotool uf2 convert -t elf` produces for
a plain flash image: 256-byte pages emitted in ascending address order, each
wrapped in a UF2 block carrying the RP2350 family ID.

Usage:
    elf2uf2.py <input.elf> <output.uf2> [--family arm-s|absolute|riscv|arm-ns]
"""

import argparse
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
UF2_PAGE_SIZE = 256
UF2_DATA_SIZE = 476  # 512-byte block minus the 36-byte header/footer

# Family IDs, from pico-sdk's boot/uf2.h.
FAMILIES = {
    "absolute": 0xE48BFF57,
    "arm-s": 0xE48BFF59,  # Arm Secure, the default for rp235x-hal images
    "riscv": 0xE48BFF5A,
    "arm-ns": 0xE48BFF5B,
}

FLASH_BASE = 0x10000000
FLASH_SIZE = 0x200000  # 2 MiB

PT_LOAD = 1


def load_flash_segments(path):
    """Return [(paddr, bytes)] for every PT_LOAD segment."""
    with open(path, "rb") as handle:
        data = handle.read()

    if data[:4] != b"\x7fELF":
        raise SystemExit(f"{path}: not an ELF file")
    if data[4] != 1:
        raise SystemExit(f"{path}: only ELF32 is supported")
    if data[5] != 1:
        raise SystemExit(f"{path}: only little-endian ELF is supported")

    e_phoff, = struct.unpack_from("<I", data, 28)
    e_phentsize, e_phnum = struct.unpack_from("<HH", data, 42)

    segments = []
    for index in range(e_phnum):
        offset = e_phoff + index * e_phentsize
        p_type, p_offset, _p_vaddr, p_paddr, p_filesz, _p_memsz, _p_flags, _p_align = (
            struct.unpack_from("<IIIIIIII", data, offset)
        )
        if p_type == PT_LOAD and p_filesz:
            segments.append((p_paddr, data[p_offset : p_offset + p_filesz]))
    return segments


def build_pages(segments):
    """Map 256-byte page addresses to their contents."""
    pages = {}
    for addr, blob in segments:
        pos = 0
        while pos < len(blob):
            current = addr + pos
            if not (FLASH_BASE <= current < FLASH_BASE + FLASH_SIZE):
                raise SystemExit(f"segment byte at 0x{current:08X} is outside flash")
            page = current & ~(UF2_PAGE_SIZE - 1)
            offset = current - page
            count = min(UF2_PAGE_SIZE - offset, len(blob) - pos)
            buf = pages.setdefault(page, bytearray(UF2_PAGE_SIZE))
            buf[offset : offset + count] = blob[pos : pos + count]
            pos += count
    return pages


def write_uf2(pages, family, out_path):
    ordered = sorted(pages.items())
    num_blocks = len(ordered)
    with open(out_path, "wb") as handle:
        for block_no, (addr, buf) in enumerate(ordered):
            header = struct.pack(
                "<IIIIIIII",
                UF2_MAGIC_START0,
                UF2_MAGIC_START1,
                UF2_FLAG_FAMILY_ID_PRESENT,
                addr,
                UF2_PAGE_SIZE,
                block_no,
                num_blocks,
                family,
            )
            payload = bytes(buf) + b"\x00" * (UF2_DATA_SIZE - UF2_PAGE_SIZE)
            handle.write(header + payload + struct.pack("<I", UF2_MAGIC_END))


def main():
    parser = argparse.ArgumentParser(description="Convert an RP2350 ELF to UF2.")
    parser.add_argument("input", help="input ELF")
    parser.add_argument("output", help="output UF2")
    parser.add_argument(
        "--family",
        default="arm-s",
        choices=sorted(FAMILIES),
        help="UF2 family ID (default: arm-s, 0x%08X)" % FAMILIES["arm-s"],
    )
    args = parser.parse_args()

    segments = load_flash_segments(args.input)
    if not segments:
        raise SystemExit(f"{args.input}: no loadable flash segments found")

    pages = build_pages(segments)
    write_uf2(pages, FAMILIES[args.family], args.output)

    low = min(pages)
    high = max(pages) + UF2_PAGE_SIZE
    print(
        f"{args.output}: {len(pages)} blocks, "
        f"0x{low:08X}..0x{high:08X}, family {args.family} (0x{FAMILIES[args.family]:08X})"
    )


if __name__ == "__main__":
    sys.exit(main())
