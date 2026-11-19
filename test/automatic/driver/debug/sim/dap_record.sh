#!/usr/bin/env bash
# dap_record.sh <build-dir> <out-dir> [pyocd] [probe-rs]: record the DAP v2 sessions of
# real host tools for the replay test. Starts dap_sim_usbip with USBIP_RECORD, attaches
# it through vhci and runs one OpenOCD session (with and without injected WAITs), and
# optionally one pyOCD and one probe-rs session, each checked against the image it
# wrote. The probe is selected by VID:PID 1209:0002 and serial 0002 only. Needs sudo,
# vhci-hcd, usbip and openocd.
set -u
BUILD=$(realpath "${1:?build dir}")
OUT=$(realpath "${2:?out dir}")
PYOCD=${3:-}
PROBERS=${4:-}
cd "$(dirname "$0")"
. ../../usb/sim/usbip_lib.sh
usbsim_lock
usbsim_modules
WORK=$(mktemp -d)
fail=0
server=""
python3 -c "import sys; sys.stdout.buffer.write(bytes((i * 37 + 11) & 0xFF for i in range(4096)))" \
  > "$WORK/img.bin"

stop_server() {
  usbsim_detach
  [ -n "$server" ] && kill "$server" 2>/dev/null && wait "$server" 2>/dev/null
  server=""
}
start() {
  local name=$1; shift
  stop_server
  env USBIP_RECORD="$WORK/$name.raw" "$@" "$BUILD/test/dap_sim_usbip" "$USBSIM_PORT" \
    > "$WORK/dap.log" 2>&1 &
  server=$!
  sleep 1
  usbsim_attach || { echo "FAIL attach"; exit 1; }
  for _ in $(seq 1 50); do lsusb -d 1209:0002 > /dev/null && break; sleep 0.1; done
  sleep 1
}
finish() {
  local name=$1 header=$2 ok=$3
  stop_server
  if [ "$ok" = 1 ] && grep -q "^target:.*protocol_errors=0 contention=0" "$WORK/dap.log"; then
    { echo "# $header"; cat "$WORK/$name.raw"; } > "$OUT/$name.txt"
    echo "PASS $name ($(wc -l < "$OUT/$name.txt") lines)"
  else
    echo "FAIL $name"; fail=1; tail -3 "$WORK/tool.log" | sed "s/^/    /"
  fi
}
ok() { if "$@"; then echo 1; else echo 0; fi; }

for wait in "" "SWD_WAIT_EVERY=7 SWD_WAIT_BURST=3"; do
  name=openocd${wait:+_wait}
  start "$name" $wait
  sudo timeout 300 openocd -f openocd_dap_sim.cfg -c init -c "sleep 100" -c poll -c halt \
    -c "sim.cpu cortex_m maskisr on" -c "reg r3 0x55aa55aa" -c "reg r3" \
    -c "load_image $WORK/img.bin 0x20000000 bin" \
    -c "dump_image $WORK/back.bin 0x20000000 4096" -c "reset halt" -c step -c resume \
    -c "sleep 100" -c poll -c halt -c shutdown > "$WORK/tool.log" 2>&1
  if [ -n "$wait" ]; then header="target wait_every=7 wait_burst=3"; else header="target wait_every=0"; fi
  finish "$name" "$header" \
    "$(ok bash -c "grep -q 'r3 (/32): 0x55aa55aa' '$WORK/tool.log' && sudo cmp -s '$WORK/img.bin' '$WORK/back.bin'")"
  sudo rm -f "$WORK/back.bin"
done

if [ -n "$PYOCD" ]; then
  start pyocd
  sudo timeout 600 "$PYOCD" commander -u 0002 -t cortex_m -O connect_mode=attach -c halt \
    -c "wreg r3 0x12345678" -c "reg r3" -c "loadmem 0x20000000 $WORK/img.bin" \
    -c "savemem 0x20000000 4096 $WORK/back.bin" -c "reset halt" -c step -c go -c halt \
    > "$WORK/tool.log" 2>&1
  finish pyocd "target wait_every=0" \
    "$(ok bash -c "grep -q 'r3 = 0x12345678' '$WORK/tool.log' && sudo cmp -s '$WORK/img.bin' '$WORK/back.bin'")"
fi

if [ -n "$PROBERS" ]; then
  start probe_rs
  P="--chip STM32G431RBTx --probe 1209:0002:0002 --protocol swd"
  {
    sudo timeout 60 "$PROBERS" write $P b32 0x20000000 0x12345678 0xCAFEF00D
    sudo timeout 60 "$PROBERS" read $P b32 0x20000000 2
    sudo timeout 60 "$PROBERS" reset $P
  } > "$WORK/tool.log" 2>&1
  finish probe_rs "target wait_every=0" \
    "$(ok grep -qi "12345678 cafef00d" "$WORK/tool.log")"
fi

usbsim_unload
sudo rm -rf "$WORK"
exit $fail
