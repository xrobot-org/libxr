#!/usr/bin/env bash
# plug_cycle.sh <cycles>: attach the simulated CDC device, start a loopback, detach it
# mid-transfer after a random delay, and check that every reattach enumerates and loops
# back cleanly. Runs on the Linux host next to a running usb_sim_usbip (port 3240), with
# acm_echo.py and usbip_lib.sh in the same directory. sudo -n is used for the usbip tool
# only; the loopback runs as the calling user (dialout group).
# 挂接仿真的 CDC 设备，开始回环，在随机延迟后传输中途摘除，检查每次重新挂接都能枚举并
# 回环。在 Linux 主机上、已运行的 usb_sim_usbip 旁边运行。sudo -n 只用于 usbip 工具，
# 回环以调用者身份运行（dialout 组）。
set -u
CYCLES=${1:-20}
cd "$(dirname "$0")"
. ./usbip_lib.sh

# The ttyACM node whose device lies under the vhci host controller.
find_dev() {
  for d in /sys/class/tty/ttyACM*; do
    [ -e "$d" ] || continue
    sim_is_vhci "$d/device" && basename "$d" && return 0
  done
  return 1
}

fail=0
i=0
for i in $(seq 1 "$CYCLES"); do
  sim_attach || { echo "cycle $i: attach failed"; fail=1; break; }
  dev=""
  for _ in $(seq 1 50); do dev=$(find_dev) && break; sleep 0.1; done
  [ -n "$dev" ] || { echo "cycle $i: no ttyACM"; fail=1; break; }
  # clean loopback first: the device must be fully usable after the previous unplug
  if ! DRAIN=1 python3 acm_echo.py "/dev/$dev" 65536 >echo.log 2>&1; then
    echo "cycle $i: loopback failed: $(cat echo.log)"; fail=1; break
  fi
  grep -o "stale=[0-9]*" echo.log | grep -v "stale=0" | sed "s/^/cycle $i: /"
  # unplug in the middle of a transfer
  timeout 20 python3 acm_echo.py "/dev/$dev" 8388608 >/dev/null 2>&1 &
  sleep "0.$((RANDOM % 9 + 1))"
  sim_detach
  wait
  for _ in $(seq 1 50); do find_dev >/dev/null || break; sleep 0.1; done
  pgrep -x usb_sim_usbip >/dev/null || { echo "cycle $i: server died"; fail=1; break; }
done
sim_detach
echo "cycles=$i fail=$fail"
exit $fail
