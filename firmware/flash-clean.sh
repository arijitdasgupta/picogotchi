#!/bin/sh
# Development flash that also clears the saved pet.
#
# A save left over from a previous firmware build would otherwise be restored on
# the next boot, which makes development confusing. This erases the whole flash
# first — the simplest way to be sure the save is gone, and it needs no knowledge
# of the region's layout — then builds, flashes and runs as usual.
#
# Not needed for production: a UF2 only writes the application sectors, so a
# saved pet survives a firmware update, and a change to the record format is
# caught by the record's version field.
set -eu

# Cargo finds .cargo/config.toml (and therefore the RP2350 target) by walking up
# from the current directory, so the script has to run from the firmware dir.
here=$(cd "$(dirname "$0")" && pwd)
cd "$here"

echo "erasing flash (this clears the saved state)"
# probe-rs 0.31.0 erases RP235x flash but never returns from `erase`, so it is
# run in the background and reaped after the erase has had time to finish.
probe-rs erase --chip RP235x >/dev/null 2>&1 &
erase_pid=$!
sleep 12
kill "$erase_pid" 2>/dev/null || true
wait "$erase_pid" 2>/dev/null || true

exec cargo run
