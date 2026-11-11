#!/usr/bin/env bash
# dap_e2e.sh <build-dir> [pyocd] [probe-rs]: end-to-end test of the DAP v2 class. Starts
# dap_sim_usbip (DAP v2 class + SwdGeneralGPIO + simulated SWD target over USB/IP),
# attaches it through vhci, and runs OpenOCD (and pyOCD and probe-rs when their paths are
# given) against it, with and without injected WAITs. Needs vhci-hcd, usbip and openocd.
# The server runs on a free TCP port with a random busid and is attached only after the
# script has checked that it is the server it started (usbip_lib.sh); a system usbipd on
# the stock port is never reached.
# 端到端测试 DAP v2 类：启动 dap_sim_usbip（DAP v2 类 + SwdGeneralGPIO + SWD 目标仿真，经
# USB/IP），经 vhci 挂接，用 OpenOCD（给了路径时还有 pyOCD、probe-rs）跑会话，带和不带
# 注入的 WAIT。服务端用空闲的 TCP 端口和随机 busid，脚本确认它就是自己启动的那个之后才挂接
# （usbip_lib.sh），不会连到系统 usbipd 的默认端口。
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
# A missing tool (usbip, sudo -n, openocd) or kernel module (vhci_hcd), and a debugger that
# cannot open the device through usbfs, are COULD NOT RUN (exit status 3), not a failure.
# 缺工具（usbip、sudo -n、openocd）或内核模块（vhci_hcd），以及调试器经 usbfs 打不开设备，
# 是 COULD NOT RUN（退出码 3），不算失败。
# Exit status: 0 all passed, 1 a check failed, 3 nothing failed but something could not run.
set -u
BUILD=$(realpath "${1:?build dir}")
PYOCD=${2:-}
PROBERS=${3:-}
cd "$(dirname "$0")"
. ../../usb/sim/usbip_lib.sh
PRIV=${DAP_PRIV:-}
sim_check_host || exit $?
sim_need_tools openocd || exit $?
[ -x "$BUILD/test/dap_sim_usbip" ] || { echo "COULD NOT RUN: $BUILD/test/dap_sim_usbip is not built"; exit 3; }
if [ -n "$PRIV" ] && ! $PRIV true 2>/dev/null; then
  echo "COULD NOT RUN: DAP_PRIV ($PRIV) does not work here"
  exit 3
fi
VIDPID_VID=1209
VIDPID_PID=0002
WORK=$(mktemp -d)
DEV=""
fail=0
skipped=0

cleanup() {
  sim_detach
  sim_stop_server
  rm -rf "$WORK"
}
trap cleanup EXIT

start() {
  sim_detach
  sim_start_server "$WORK/dap.log" env "$@" "$BUILD/test/dap_sim_usbip" @PORT@ ||
    { echo "FAIL: the server did not start"; exit 1; }
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
  if "$@" && target_clean; then echo "PASS $name"
  elif grep -qE "LIBUSB_ERROR_ACCESS|Permission denied|Access denied" "$WORK/tool.log"; then
    echo "COULD NOT RUN $name (the debugger cannot open the device; set DAP_PRIV=\"sudo -n\")"
    skipped=1
  else
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

[ "$fail" -ne 0 ] && exit 1
[ "$skipped" -ne 0 ] && exit 3
exit 0
