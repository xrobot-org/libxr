#!/usr/bin/env bash
# dap_record.sh <build-dir> <out-dir> [pyocd] [probe-rs]: record the DAP v2 sessions of
# real host tools for the replay test. Starts dap_sim_usbip with USBIP_RECORD, attaches
# it through vhci and runs one OpenOCD session (with and without injected WAITs), and
# optionally one pyOCD and one probe-rs session, each checked against the image it
# wrote. Needs vhci-hcd, usbip and openocd. The device is selected as in dap_e2e.sh
# (VID:PID unique and under vhci, serial and bus-port location; DAP_PRIV="sudo -n" for
# the debugger only; pyOCD and probe-rs only with DAP_TOOLS_SCAN_OK=1).
# 录下真实主机工具的 DAP v2 会话供回放测试使用：启动带 USBIP_RECORD 的 dap_sim_usbip，
# 经 vhci 挂接，跑一次 OpenOCD 会话（带和不带注入的 WAIT），可选再跑 pyOCD、probe-rs，
# 每次都对照它写入的镜像检查。设备的选择同 dap_e2e.sh。
set -u
BUILD=$(realpath "${1:?build dir}")
OUT=$(realpath "${2:?out dir}")
PYOCD=${3:-}
PROBERS=${4:-}
cd "$(dirname "$0")"
. ../../usb/sim/usbip_lib.sh
PRIV=${DAP_PRIV:-}
WORK=$(mktemp -d)
SERVER_PID=""
DEV=""
fail=0
python3 -c "import sys; sys.stdout.buffer.write(bytes((i * 37 + 11) & 0xFF for i in range(4096)))" \
  > "$WORK/img.bin"

stop_server() {
  sim_detach
  if [ -n "$SERVER_PID" ]; then kill "$SERVER_PID" 2>/dev/null; wait "$SERVER_PID" 2>/dev/null; fi
  SERVER_PID=""
}
cleanup() {
  stop_server
  rm -rf "$WORK"
}
trap cleanup EXIT

start() {
  local name=$1; shift
  stop_server
  env USBIP_RECORD="$WORK/$name.raw" "$@" "$BUILD/test/dap_sim_usbip" "$SIM_TCP_PORT" \
    > "$WORK/dap.log" 2>&1 &
  SERVER_PID=$!
  sleep 1
  sim_attach || { echo "FAIL attach"; exit 1; }
  DEV=""
  for _ in $(seq 1 50); do
    DEV=$(sim_usb_device_unique 1209 0002) && break
    sleep 0.1
  done
  [ -n "$DEV" ] || { echo "FAIL: 1209:0002 is not a single device under vhci"; exit 1; }
  sleep 1
}
finish() {
  local name=$1 header=$2 ok=$3
  sleep 1.5
  local clean=0
  grep "^target:" "$WORK/dap.log" | tail -1 | grep -q "protocol_errors=0 contention=0" && clean=1
  stop_server
  if [ "$ok" = 1 ] && [ "$clean" = 1 ]; then
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
  timeout 300 $PRIV openocd -c "set SIM_SERIAL $(cat "$DEV/serial")" \
    -c "set SIM_LOCATION $(basename "$DEV")" -f openocd_dap_sim.cfg -c init \
    -c "sleep 100" -c poll -c halt -c "sim.cpu cortex_m maskisr on" -c "reg r3 0x55aa55aa" \
    -c "reg r3" -c "load_image $WORK/img.bin 0x20000000 bin" \
    -c "dump_image $WORK/back.bin 0x20000000 4096" -c "reset halt" -c step -c resume \
    -c "sleep 100" -c poll -c halt -c shutdown > "$WORK/tool.log" 2>&1
  if [ -n "$wait" ]; then header="target wait_every=7 wait_burst=3"; else header="target wait_every=0"; fi
  finish "$name" "$header" \
    "$(ok bash -c "grep -q 'r3 (/32): 0x55aa55aa' '$WORK/tool.log' && cmp -s '$WORK/img.bin' '$WORK/back.bin'")"
  rm -f "$WORK/back.bin"
done

if [ "${DAP_TOOLS_SCAN_OK:-0}" != 1 ] && { [ -n "$PYOCD" ] || [ -n "$PROBERS" ]; }; then
  echo "SKIP pyOCD/probe-rs: they open every CMSIS-DAP probe they find; set DAP_TOOLS_SCAN_OK=1 on a host without other probes"
  PYOCD=""
  PROBERS=""
fi

if [ -n "$PYOCD" ]; then
  start pyocd
  timeout 600 $PRIV "$PYOCD" commander -u "$(cat "$DEV/serial")" -t cortex_m \
    -O connect_mode=attach -c halt -c "wreg r3 0x12345678" -c "reg r3" \
    -c "loadmem 0x20000000 $WORK/img.bin" -c "savemem 0x20000000 4096 $WORK/back.bin" \
    -c "reset halt" -c step -c go -c halt > "$WORK/tool.log" 2>&1
  finish pyocd "target wait_every=0" \
    "$(ok bash -c "grep -q 'r3 = 0x12345678' '$WORK/tool.log' && cmp -s '$WORK/img.bin' '$WORK/back.bin'")"
fi

if [ -n "$PROBERS" ]; then
  start probe_rs
  P="--chip STM32G431RBTx --probe 1209:0002:$(cat "$DEV/serial") --protocol swd"
  {
    timeout 60 $PRIV "$PROBERS" write $P b32 0x20000000 0x12345678 0xCAFEF00D
    timeout 60 $PRIV "$PROBERS" read $P b32 0x20000000 2
    timeout 60 $PRIV "$PROBERS" reset $P
  } > "$WORK/tool.log" 2>&1
  finish probe_rs "target wait_every=0" \
    "$(ok grep -qi "12345678 cafef00d" "$WORK/tool.log")"
fi

exit $fail
