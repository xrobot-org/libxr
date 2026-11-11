#!/usr/bin/env bash
# plug_cycle.sh <build-dir> [cycles]: start usb_sim_usbip on a free port, attach the
# simulated CDC device, start a loopback, detach it mid-transfer after a random delay, and
# check that every reattach enumerates and loops back cleanly. Needs vhci-hcd, usbip and
# the cdc_acm driver; acm_echo.py and usbip_lib.sh are in the same directory. sudo -n is
# used for the usbip tool only; the loopback runs as the calling user (dialout group).
# 启动 usb_sim_usbip（空闲端口），挂接仿真的 CDC 设备，开始回环，在随机延迟后传输中途摘除，
# 检查每次重新挂接都能枚举并回环。sudo -n 只用于 usbip 工具，回环以调用者身份运行（dialout
# 组）。
# Exit status: 0 passed, 1 a check failed, 3 it could not run (a missing tool, privilege,
# kernel module or device node).
set -u
BUILD=$(realpath "${1:?build dir}")
CYCLES=${2:-20}
cd "$(dirname "$0")"
. ./usbip_lib.sh
sim_check_host || exit $?
[ -x "$BUILD/test/usb_sim_usbip" ] || { echo "COULD NOT RUN: $BUILD/test/usb_sim_usbip is not built"; exit 3; }
WORK=$(mktemp -d)
cleanup() {
  sim_detach
  sim_stop_server
  rm -rf "$WORK"
}
trap cleanup EXIT

# The ttyACM node whose device lies under the vhci host controller.
find_dev() {
  for d in /sys/class/tty/ttyACM*; do
    [ -e "$d" ] || continue
    sim_is_vhci "$d/device" && basename "$d" && return 0
  done
  return 1
}

sim_start_server "$WORK/server.log" "$BUILD/test/usb_sim_usbip" @PORT@ ||
  { echo "FAIL: the server did not start"; exit 1; }
fail=0
i=0
for i in $(seq 1 "$CYCLES"); do
  sim_attach || { echo "cycle $i: attach failed"; fail=1; break; }
  dev=""
  for _ in $(seq 1 50); do dev=$(find_dev) && break; sleep 0.1; done
  if [ -z "$dev" ]; then
    if [ ! -d /sys/module/cdc_acm ]; then
      echo "COULD NOT RUN: the cdc_acm kernel module is not loaded, so there is no ttyACM node"
      exit 3
    fi
    echo "cycle $i: no ttyACM"; fail=1; break
  fi
  # clean loopback first: the device must be fully usable after the previous unplug
  DRAIN=1 python3 acm_echo.py "/dev/$dev" 65536 >"$WORK/echo.log" 2>&1
  status=$?
  if [ "$status" -eq 3 ]; then
    echo "COULD NOT RUN: $(cat "$WORK/echo.log")"
    exit 3
  elif [ "$status" -ne 0 ]; then
    echo "cycle $i: loopback failed: $(cat "$WORK/echo.log")"; fail=1; break
  fi
  grep -o "stale=[0-9]*" "$WORK/echo.log" | grep -v "stale=0" | sed "s/^/cycle $i: /"
  # unplug in the middle of a transfer
  timeout 20 python3 acm_echo.py "/dev/$dev" 8388608 >/dev/null 2>&1 &
  echo_pid=$!
  sleep "0.$((RANDOM % 9 + 1))"
  sim_detach
  wait "$echo_pid"
  for _ in $(seq 1 50); do find_dev >/dev/null || break; sleep 0.1; done
  sim_server_alive || { echo "cycle $i: server died"; fail=1; break; }
done
sim_detach
echo "cycles=$i fail=$fail"
exit $fail
