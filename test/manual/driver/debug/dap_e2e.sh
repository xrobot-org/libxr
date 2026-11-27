#!/usr/bin/env bash
# dap_e2e.sh <build-dir> [pyocd] [probe-rs]: end-to-end test of DAP v2 and v1 classes
# across SWD and JTAG. Starts dap_sim_usbip (--mode=v2-swd|v1-swd|v2-jtag|v1-jtag),
# attaches it through vhci, and runs OpenOCD, pyOCD and probe-rs sessions against it,
# with and without injected WAITs. The probe is always selected by VID:PID 1209:0002 and
# serial 0002, never a real probe. Needs sudo, vhci-hcd, usbip, openocd and optionally
# pyOCD and probe-rs.
set -u
BUILD=$(realpath "${1:?build dir}")
PYOCD=${2:-}
PROBERS=${3:-}
cd "$(dirname "$0")"
. ../usb/usbip_lib.sh
usbsim_lock
usbsim_modules
WORK=$(mktemp -d)
fail=0
server=""
stop_server() {
  usbsim_detach
  [ -n "$server" ] && kill "$server" 2>/dev/null && wait "$server" 2>/dev/null
  server=""
}
start_mode() {
  local mode=$1; shift
  stop_server
  env "$@" "$BUILD/test/dap_sim_usbip" "$USBSIM_PORT" "--mode=$mode" 9>&- > "$WORK/dap.log" 2>&1 &
  server=$!
  sleep 1
  usbsim_attach || { echo "FAIL attach ($mode)"; fail=1; return 1; }
  for _ in $(seq 1 50); do lsusb -d 1209:0002 > /dev/null && break; sleep 0.1; done
  case "$mode" in
    v1-*)
      for _ in $(seq 1 50); do
        grep -ql "HID_ID=0003:00001209:00000002" /sys/class/hidraw/hidraw*/device/uevent 2>/dev/null && break
        sleep 0.1
      done
      ;;
  esac
  sleep 1
}
target_clean() {
  sleep 1.5
  grep "^target:" "$WORK/dap.log" | tail -1 | grep -q "protocol_errors=0 contention=0"
}
check() {
  local name=$1; shift
  if "$@" && target_clean; then echo "PASS $name"; else
    echo "FAIL $name"; fail=1
    tail -10 "$WORK/tool.log" | sed "s/^/    /"
    grep "^target:" "$WORK/dap.log" | tail -1 | sed "s/^/    /"
  fi
}
ocd() {
  local cfg=$1; shift
  sudo timeout 300 openocd -f "$cfg" -c "gdb port disabled" -c "tcl port disabled" \
    -c "telnet port disabled" -c init -c "sleep 100" -c poll -c halt \
    -c "sim.cpu cortex_m maskisr on" "$@" -c shutdown > "$WORK/tool.log" 2>&1
}
py() {
  sudo timeout 600 "$PYOCD" commander -u 0002 -t cortex_m -O connect_mode=attach -c halt \
    "$@" > "$WORK/tool.log" 2>&1
}
prs() {
  local proto=$1; shift
  sudo timeout 60 "$PROBERS" "$@" --chip STM32G431RBTx --probe 1209:0002:0002 \
    --protocol "$proto"
}
same_image() { sudo cmp -s "$WORK/img.bin" "$WORK/back.bin"; }
ocd_ok() {
  grep -q 'r3 (/32): 0x55aa55aa' "$WORK/tool.log" && same_image &&
    grep -q single-step "$WORK/tool.log"
}
py_ok() {
  grep -q 'r3 = 0x12345678' "$WORK/tool.log" && same_image &&
    ! grep -qiE 'error|exception' "$WORK/tool.log"
}
prs_ok() {
  grep -qi '12345678 cafef00d 0badbeef' "$WORK/tool.log" &&
    grep -qi '410fc241' "$WORK/tool.log"
}
run_ocd_suite() {
  local cfg=$1 label=$2
  ocd "$cfg" -c "reg r3 0x55aa55aa" -c "reg r3" -c "load_image $WORK/img.bin 0x20000000 bin" \
    -c "dump_image $WORK/back.bin 0x20000000 65536" -c "reset halt" -c step -c resume \
    -c "sleep 100" -c poll -c halt
  check "OpenOCD registers, 64 KiB download, reset, step ($label)" ocd_ok
  sudo rm -f "$WORK/back.bin"
}
run_pyocd_suite() {
  local label=$1
  if [ -n "$PYOCD" ]; then
    py -c "wreg r3 0x12345678" -c "reg r3" -c "loadmem 0x20000000 $WORK/img.bin" \
      -c "savemem 0x20000000 65536 $WORK/back.bin" -c "reset halt" -c step -c go -c halt
    check "pyOCD registers, 64 KiB download, reset, step ($label)" py_ok
    sudo rm -f "$WORK/back.bin"
  fi
}
run_prs_suite() {
  local proto=$1 label=$2
  if [ -n "$PROBERS" ]; then
    {
      prs "$proto" write b32 0x20000100 0x12345678 0xCAFEF00D 0x0BADBEEF
      prs "$proto" read b32 0x20000100 3
      prs "$proto" reset
      prs "$proto" read b32 0xE000ED00 1
    } > "$WORK/tool.log" 2>&1
    check "probe-rs write, read, reset, CPUID ($label)" prs_ok
  fi
}

head -c 65536 /dev/urandom > "$WORK/img.bin"

for wait in "" "SWD_WAIT_EVERY=7 SWD_WAIT_BURST=3"; do
  wtag=${wait:+, injected WAITs}
  start_mode v2-swd $wait || continue
  run_ocd_suite openocd_dap_sim.cfg "v2-swd$wtag"
  run_pyocd_suite "v2-swd$wtag"
  run_prs_suite swd "v2-swd$wtag"
done

if start_mode v1-swd; then
  run_ocd_suite openocd_dap_sim_v1.cfg "v1-swd"
  run_pyocd_suite "v1-swd"
  run_prs_suite swd "v1-swd"
fi

for wait in "" "SWD_WAIT_EVERY=7 SWD_WAIT_BURST=3"; do
  wtag=${wait:+, injected WAITs}
  start_mode v2-jtag $wait || continue
  run_ocd_suite openocd_dap_sim_jtag.cfg "v2-jtag$wtag"
  run_prs_suite jtag "v2-jtag$wtag"
done

if start_mode v1-jtag; then
  run_ocd_suite openocd_dap_sim_v1_jtag.cfg "v1-jtag"
  run_prs_suite jtag "v1-jtag"
fi

stop_server
usbsim_unload
sudo rm -rf "$WORK"
exit $fail
