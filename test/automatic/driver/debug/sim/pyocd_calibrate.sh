#!/usr/bin/env bash
# pyocd_calibrate.sh <build-dir> [pyocd]: calibrate the simulated SWD target against real
# pyOCD with the pyocd-bitbang probe plugin. Starts swd_target_bitbang, runs the sessions
# below and prints PASS or FAIL for each; exits non-zero on any failure.
set -u
BUILD=$(realpath "${1:?build dir}")
PYOCD=${2:-pyocd}
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
py() {
  timeout 600 "$PYOCD" commander -u remote_bitbang:127.0.0.1:44853 -t cortex_m \
    -O connect_mode=attach -c halt "$@" > "$WORK/py.log" 2>&1
}
check() {
  local name=$1; shift
  if "$@"; then echo "PASS $name"; else
    echo "FAIL $name"; fail=1
    tail -5 "$WORK/py.log" | sed "s/^/    /"
  fi
  grep -h "session:" "$WORK/server.log" | tail -1 | grep -q "protocol_errors=0 contention=0" || { echo "FAIL $name: protocol errors or contention"; fail=1; }
}
head -c 16384 /dev/urandom > "$WORK/img.bin"

server
py -c "wreg r3 0x55aa55aa" -c "reg r3"
check "core registers" grep -q "r3 = 0x55aa55aa" "$WORK/py.log"

server
py -c "loadmem 0x20000000 $WORK/img.bin" -c "savemem 0x20000000 16384 $WORK/back.bin"
check "16 KiB RAM download" cmp -s "$WORK/img.bin" "$WORK/back.bin"

server
py -c "reset halt" -c step -c step -c go -c halt -c status
check "reset, step, go" bash -c "! grep -qiE 'error|exception' '$WORK/py.log'"

# WAIT/FAULT are calibrated with OpenOCD only: pyocd-bitbang 1.0.2 clocks a data phase
# after WAIT although pyOCD leaves ORUNDETECT off, and skips the turnaround after a FAULT
# on a read; pyOCD recovers by reconnecting, so the data is right but the wire is not.

kill "$SERVER_PID" 2>/dev/null
rm -rf "$WORK"
exit $fail
