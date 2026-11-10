#!/usr/bin/env bash
# dap_e2e.sh <build-dir> [pyocd] [probe-rs]: end-to-end test of the DAP v2 class. Starts
# dap_sim_usbip (DAP v2 class + SwdGeneralGPIO + simulated SWD target over USB/IP),
# attaches it through vhci, and runs OpenOCD (and pyOCD and probe-rs when their paths are
# given) against it, with and without injected WAITs. Needs vhci-hcd, usbip and openocd.
# 端到端测试 DAP v2 类：启动 dap_sim_usbip（DAP v2 类 + SwdGeneralGPIO + SWD 目标仿真，经
# USB/IP），经 vhci 挂接，用 OpenOCD（给了路径时还有 pyOCD、probe-rs）跑会话，带和不带
# 注入的 WAIT。
#
# The host has real CMSIS-DAP probes, so the simulated one is selected in several ways at
# once: its VID:PID 1209:0002 must belong to exactly one device on the host and that
# device must lie under vhci (usbip_lib.sh, checked before every tool starts), OpenOCD
# gets its serial string and its bus-port location, and the vhci port is found from the
# URL used at import. Note that the OpenOCD build tested (0.12.0+dev-02421) ignores
# "adapter usb location" in the cmsis-dap usb_bulk backend (it connected with a wrong
# location), so there the selection is VID:PID + serial, made unambiguous by the check
# above; OpenOCD opens only devices that match both. DAP_PRIV="sudo -n" runs only the
# debugger (OpenOCD and, if asked for, pyOCD/probe-rs) as root, which usbfs needs; the
# script itself and the simulated probe run as the calling user. pyOCD and probe-rs have
# no bus-port option and open every CMSIS-DAP probe they find to read its serial, so they
# run only with DAP_TOOLS_SCAN_OK=1 on a host that has no other probe.
# 主机上有真实的 CMSIS-DAP 探针，所以仿真探针同时用几种方式选：VID:PID 1209:0002 在主机上
# 只属于一个设备且该设备在 vhci 之下（usbip_lib.sh，每个工具启动前检查）；OpenOCD 拿到它
# 的序列号和总线端口位置；vhci 端口按导入时的 URL 找。注意测试用的 OpenOCD 版本
# （0.12.0+dev-02421）在 cmsis-dap usb_bulk 后端忽略 "adapter usb location"（给错误位置
# 也连上了），所以那里的选择是 VID:PID + 序列号，由上面的检查保证无歧义；OpenOCD 只打开
# 两者都匹配的设备。DAP_PRIV="sudo -n" 只让调试器以 root 运行（usbfs 需要），脚本本身和
# 仿真探针以调用者身份运行。pyOCD 和 probe-rs 没有总线端口选项，会打开找到的每个
# CMSIS-DAP 探针读序列号，所以只有在设置 DAP_TOOLS_SCAN_OK=1 且主机上没有其他探针时才运行。
# Exit status: 0 all passed, 1 a check failed.
set -u
BUILD=$(realpath "${1:?build dir}")
PYOCD=${2:-}
PROBERS=${3:-}
cd "$(dirname "$0")"
. ../../usb/sim/usbip_lib.sh
PRIV=${DAP_PRIV:-}
VIDPID_VID=1209
VIDPID_PID=0002
WORK=$(mktemp -d)
SERVER_PID=""
DEV=""
fail=0

cleanup() {
  sim_detach
  [ -n "$SERVER_PID" ] && kill "$SERVER_PID" 2>/dev/null
  rm -rf "$WORK"
}
trap cleanup EXIT

start() {
  sim_detach
  if [ -n "$SERVER_PID" ]; then kill "$SERVER_PID" 2>/dev/null; wait "$SERVER_PID" 2>/dev/null; fi
  env "$@" "$BUILD/test/dap_sim_usbip" "$SIM_TCP_PORT" > "$WORK/dap.log" 2>&1 &
  SERVER_PID=$!
  sleep 1
  sim_attach || { echo "FAIL attach"; exit 1; }
  DEV=""
  for _ in $(seq 1 50); do
    DEV=$(sim_usb_device_unique $VIDPID_VID $VIDPID_PID) && break
    sleep 0.1
  done
  [ -n "$DEV" ] || { echo "FAIL: 1209:0002 is not a single device under vhci"; exit 1; }
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
    tail -3 "$WORK/tool.log" | sed "s/^/    /"; grep "^target:" "$WORK/dap.log" | tail -1 | sed "s/^/    /"
  fi
}
ocd() {
  timeout 300 $PRIV openocd -c "set SIM_SERIAL $(cat "$DEV/serial")" \
    -c "set SIM_LOCATION $(basename "$DEV")" -f openocd_dap_sim.cfg -c init \
    -c "sleep 100" -c poll -c halt -c "sim.cpu cortex_m maskisr on" "$@" -c shutdown \
    > "$WORK/tool.log" 2>&1
}
head -c 65536 /dev/urandom > "$WORK/img.bin"

for wait in "" "SWD_WAIT_EVERY=7 SWD_WAIT_BURST=3"; do
  label=${wait:+ (injected WAITs)}
  start $wait
  ocd -c "reg r3 0x55aa55aa" -c "reg r3" -c "load_image $WORK/img.bin 0x20000000 bin" \
      -c "dump_image $WORK/back.bin 0x20000000 65536" -c "reset halt" -c step -c resume \
      -c "sleep 100" -c poll -c halt
  check "OpenOCD registers, 64 KiB download, reset, step$label" \
    bash -c "grep -q 'r3 (/32): 0x55aa55aa' '$WORK/tool.log' && cmp -s '$WORK/img.bin' '$WORK/back.bin' && grep -q single-step '$WORK/tool.log'"
  rm -f "$WORK/back.bin"
  if [ -n "$PYOCD" ] && [ "${DAP_TOOLS_SCAN_OK:-0}" = 1 ]; then
    timeout 600 $PRIV "$PYOCD" commander -u "$(cat "$DEV/serial")" -t cortex_m \
      -O connect_mode=attach -c halt -c "wreg r3 0x12345678" -c "reg r3" \
      -c "loadmem 0x20000000 $WORK/img.bin" -c "savemem 0x20000000 65536 $WORK/back.bin" \
      -c "reset halt" -c step -c go -c halt > "$WORK/tool.log" 2>&1
    check "pyOCD registers, 64 KiB download, reset, step$label" \
      bash -c "grep -q 'r3 = 0x12345678' '$WORK/tool.log' && cmp -s '$WORK/img.bin' '$WORK/back.bin' && ! grep -qiE 'error|exception' '$WORK/tool.log'"
    rm -f "$WORK/back.bin"
  elif [ -n "$PYOCD" ]; then
    echo "SKIP pyOCD: it opens every CMSIS-DAP probe it finds; set DAP_TOOLS_SCAN_OK=1 on a host without other probes"
  fi
done

if [ -n "$PROBERS" ]; then
  if [ "${DAP_TOOLS_SCAN_OK:-0}" = 1 ]; then
    start
    P="--chip STM32G431RBTx --probe $VIDPID_VID:$VIDPID_PID:$(cat "$DEV/serial") --protocol swd"
    {
      timeout 60 $PRIV "$PROBERS" write $P b32 0x20000000 0x12345678 0xCAFEF00D
      timeout 60 $PRIV "$PROBERS" read $P b32 0x20000000 2
      timeout 60 $PRIV "$PROBERS" reset $P
    } > "$WORK/tool.log" 2>&1
    check "probe-rs write, read, reset" grep -qi "12345678 cafef00d" "$WORK/tool.log"
  else
    echo "SKIP probe-rs: it opens every CMSIS-DAP probe it finds; set DAP_TOOLS_SCAN_OK=1 on a host without other probes"
  fi
fi

exit $fail
