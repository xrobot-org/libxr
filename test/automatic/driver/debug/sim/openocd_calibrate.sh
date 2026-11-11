#!/usr/bin/env bash
# openocd_calibrate.sh <build-dir>: calibrate the simulated SWD target against real
# OpenOCD. Starts swd_target_bitbang, runs the sessions below and prints PASS or FAIL
# for each; exits 1 on any failure, 3 when it could not run (no openocd in PATH, or
# swd_target_bitbang not built).
set -u
BUILD=$(realpath "${1:?build dir}")
cd "$(dirname "$0")"
command -v openocd >/dev/null 2>&1 || { echo "COULD NOT RUN: openocd is not installed or not in PATH"; exit 3; }
[ -x "$BUILD/test/swd_target_bitbang" ] || { echo "COULD NOT RUN: $BUILD/test/swd_target_bitbang is not built"; exit 3; }
WORK=$(mktemp -d)
SERVER_PID=""
trap '[ -n "$SERVER_PID" ] && kill "$SERVER_PID" 2>/dev/null; rm -rf "$WORK"' EXIT
fail=0
server() {
  if [ -n "$SERVER_PID" ]; then kill "$SERVER_PID" 2>/dev/null; wait "$SERVER_PID" 2>/dev/null; fi
  env "$@" "$BUILD/test/swd_target_bitbang" 44853 > "$WORK/server.log" 2>&1 &
  SERVER_PID=$!
  sleep 0.5
  grep -q "remote_bitbang on 127.0.0.1:44853" "$WORK/server.log" || { echo "FAIL: swd_target_bitbang did not start (port 44853 busy?)"; exit 1; }
}
ocd() { timeout 300 openocd -f openocd_sim.cfg -c init -c "sleep 100" -c poll -c halt -c "sim.cpu cortex_m maskisr on" "$@" -c shutdown > "$WORK/ocd.log" 2>&1; }
check() {
  local name=$1; shift
  if "$@"; then echo "PASS $name"; else
    echo "FAIL $name"; fail=1
    grep -E "Error|Warn|halted" "$WORK/ocd.log" | sed "s/^/    /"
  fi
  grep -h "session:" "$WORK/server.log" | tail -1 | grep -q "protocol_errors=0 contention=0" || { echo "FAIL $name: protocol errors or contention"; fail=1; }
}
head -c 65536 /dev/urandom > "$WORK/img.bin"

server
ocd -c "dap info 0"
check "examine and ROM table" grep -q "Cortex-M4 SCS" "$WORK/ocd.log"

server
ocd -c "reg r3 0x55aa55aa" -c "reg r3"
check "core registers" grep -q "r3 (/32): 0x55aa55aa" "$WORK/ocd.log"

server
ocd -c "load_image $WORK/img.bin 0x20000000 bin" -c "dump_image $WORK/back.bin 0x20000000 65536"
check "64 KiB RAM download" cmp -s "$WORK/img.bin" "$WORK/back.bin"

server
ocd -c "reset halt" -c step -c step -c resume -c "sleep 100" -c poll -c halt
check "reset, step, resume" bash -c "grep -c 'single-step' '$WORK/ocd.log' | grep -q 2 && ! grep -q Error '$WORK/ocd.log'"

server
ocd -c "bp 0x08000100 2 hw" -c "rbp 0x08000100"
check "hardware breakpoint" grep -q "breakpoint set at 0x08000100" "$WORK/ocd.log"

server SWD_WAIT_EVERY=7 SWD_WAIT_BURST=3
ocd -c "load_image $WORK/img.bin 0x20000000 bin" -c "dump_image $WORK/back.bin 0x20000000 65536"
check "download under injected WAITs" cmp -s "$WORK/img.bin" "$WORK/back.bin"

kill "$SERVER_PID" 2>/dev/null
rm -rf "$WORK"
exit $fail
