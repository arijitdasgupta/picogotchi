//! Wear-levelled persistence of the emulator state in a reserved flash region.
//!
//! `memory.x` holds back the last 32 KiB of flash for this module, so the linker
//! can never place code in a sector that gets erased.
//!
//! Layout: a ring of 4 KiB sectors. Each save writes one 512-byte record (two
//! 256-byte pages) at the start of the next sector, so a sector is erased once
//! every `NUM_SECTORS` saves. On boot every sector is scanned and the record with
//! the highest sequence number wins; anything that fails its magic, version,
//! length or CRC check is ignored, which is what makes a save interrupted by a
//! power cut harmless.
//!
//! Reading is a plain XIP access (the region is memory-mapped). Writing is the
//! delicate part — see [`write_record`].

use rp235x_hal::rom_data;
use tama::{Tama, STATE_LEN};

// Region reserved by `memory.x`.
extern "C" {
    static __save_start: u8;
    static __save_end: u8;
}

/// Non-zero so that erased (`0xFF`) and zeroed flash can never validate, which
/// is also what makes the dev wipe in `flash-clean.sh` work.
const MAGIC: u32 = 0x5047_5331; // "PGS1"
/// Bump when the record layout changes; older records are then ignored.
const VERSION: u16 = 1;

const SECTOR_SIZE: u32 = 4096;
/// Two 256-byte pages: 16 bytes of header, then the state.
const RECORD_SIZE: usize = 512;
const PAYLOAD_OFFSET: usize = 16;
const CRC_OFFSET: usize = RECORD_SIZE - 4;
/// Sector erase (4 KiB) command.
const SECTOR_ERASE_CMD: u8 = 0x20;

/// The flash `flash_range_*` calls take an offset from the start of flash.
const FLASH_BASE: u32 = 0x1000_0000;

fn region_start() -> u32 {
    unsafe { &__save_start as *const u8 as u32 }
}

fn region_end() -> u32 {
    unsafe { &__save_end as *const u8 as u32 }
}

fn num_sectors() -> u32 {
    (region_end() - region_start()) / SECTOR_SIZE
}

// -- record encoding --------------------------------------------------------

fn rd_u16(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

fn rd_u32(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

/// CRC-32 (IEEE 802.3), bitwise so it needs no table.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Decode a record, returning `(sequence, state)` or `None` if it is not valid.
fn parse(record: &[u8; RECORD_SIZE]) -> Option<(u32, [u8; STATE_LEN])> {
    if rd_u32(record, 0) != MAGIC || rd_u16(record, 8) != VERSION {
        return None;
    }
    if rd_u16(record, 10) as usize != STATE_LEN {
        return None;
    }
    if rd_u32(record, CRC_OFFSET) != crc32(&record[..CRC_OFFSET]) {
        return None;
    }
    let mut state = [0u8; STATE_LEN];
    state.copy_from_slice(&record[PAYLOAD_OFFSET..PAYLOAD_OFFSET + STATE_LEN]);
    Some((rd_u32(record, 4), state))
}

/// Build the record for `state` at sequence number `sequence`.
fn encode(sequence: u32, state: &[u8; STATE_LEN]) -> [u8; RECORD_SIZE] {
    let mut record = [0xFFu8; RECORD_SIZE];
    record[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    record[4..8].copy_from_slice(&sequence.to_le_bytes());
    record[8..10].copy_from_slice(&VERSION.to_le_bytes());
    record[10..12].copy_from_slice(&(STATE_LEN as u16).to_le_bytes());
    record[PAYLOAD_OFFSET..PAYLOAD_OFFSET + STATE_LEN].copy_from_slice(state);
    let crc = crc32(&record[..CRC_OFFSET]);
    record[CRC_OFFSET..CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
    record
}

// -- reading ----------------------------------------------------------------

/// Read a record out of the memory-mapped flash region.
fn read_record(addr: u32) -> [u8; RECORD_SIZE] {
    let src = addr as *const u8;
    let mut out = [0u8; RECORD_SIZE];
    for (i, byte) in out.iter_mut().enumerate() {
        // SAFETY: `addr` is inside the region reserved by memory.x.
        *byte = unsafe { core::ptr::read_volatile(src.add(i)) };
    }
    out
}

/// The newest valid save, as `(sequence, sector, state)`.
pub fn load() -> Option<(u32, u32, [u8; STATE_LEN])> {
    let mut best: Option<(u32, u32, [u8; STATE_LEN])> = None;
    for sector in 0..num_sectors() {
        let record = read_record(region_start() + sector * SECTOR_SIZE);
        if let Some((sequence, state)) = parse(&record) {
            if best.is_none_or(|(best_seq, _, _)| sequence > best_seq) {
                best = Some((sequence, sector, state));
            }
        }
    }
    best
}

/// Restore `tama` from flash, if there is a valid save.
pub fn restore(tama: &mut Tama) -> Option<u32> {
    let (sequence, _, state) = load()?;
    tama.restore_state(&state);
    Some(sequence)
}

// -- writing ----------------------------------------------------------------

/// ROM entry points, resolved while XIP is still available.
struct RomFlash {
    connect: unsafe extern "C" fn(),
    exit_xip: unsafe extern "C" fn(),
    erase: unsafe extern "C" fn(u32, usize, u32, u8),
    program: unsafe extern "C" fn(u32, *const u8, usize),
    flush: unsafe extern "C" fn(),
}

impl RomFlash {
    fn resolve() -> Self {
        Self {
            connect: rom_data::connect_internal_flash::ptr(),
            exit_xip: rom_data::flash_exit_xip::ptr(),
            erase: rom_data::flash_range_erase::ptr(),
            program: rom_data::flash_range_program::ptr(),
            flush: rom_data::flash_flush_cache::ptr(),
        }
    }
}

/// Erase `addr`'s sector and program `record` into it.
///
/// XIP is unavailable for the duration, so this runs from RAM: it lives in
/// `.data`, which cortex-m-rt copies into SRAM at startup. Everything it calls
/// is a ROM entry point passed in as a pointer — resolving them here, or calling
/// any other function that lives in flash, would fault mid-erase.
///
/// The caller must have masked interrupts, since the TIMER0 handler also lives
/// in flash.
///
/// Mirrors the sequence in the pico-sdk's `flash_range_erase` /
/// `flash_range_program`: `connect_internal_flash`, `flash_exit_xip`, the
/// operation, then `flash_flush_cache` (which also releases the CSn IO force).
/// Unlike RP2040 there is no XIP-mode restore afterwards; the RP2350 ROM
/// restores it as part of the erase/program calls.
#[inline(never)]
#[link_section = ".data.flash_ops"]
unsafe fn write_record(rom: &RomFlash, addr: u32, record: &[u8; RECORD_SIZE]) {
    let offset = addr - FLASH_BASE;

    (rom.connect)();
    (rom.exit_xip)();
    (rom.erase)(offset, SECTOR_SIZE as usize, SECTOR_SIZE, SECTOR_ERASE_CMD);
    (rom.flush)();

    (rom.connect)();
    (rom.exit_xip)();
    (rom.program)(offset, record.as_ptr(), RECORD_SIZE);
    (rom.flush)();
}

/// Save `state` as the next record in the ring. Returns its sequence number.
pub fn save(state: &[u8; STATE_LEN]) -> u32 {
    let (sequence, sector) = match load() {
        Some((sequence, sector, _)) => (sequence.wrapping_add(1), (sector + 1) % num_sectors()),
        None => (1, 0),
    };
    let record = encode(sequence, state);
    let addr = region_start() + sector * SECTOR_SIZE;

    let rom = RomFlash::resolve();
    // Interrupts are masked across the whole operation: the TIMER0 handler lives
    // in flash and would fault while XIP is down. The erase also stalls XIP for
    // tens of milliseconds, so the emulator briefly falls behind afterwards and
    // catches up on the next pass of the main loop.
    cortex_m::interrupt::free(|_| {
        // SAFETY: `addr` is sector-aligned inside the reserved region, and
        // `record` outlives the call.
        unsafe { write_record(&rom, addr, &record) };
    });

    sequence
}
