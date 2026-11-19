#!/usr/bin/env bash
# plug_cycle.sh <build-dir> [cycles]: attach the simulated CDC device, check a loopback,
# detach it in the middle of a transfer after a random delay, and check that every
# reattach enumerates and loops back cleanly. Starts usb_sim_usbip itself; needs sudo,
# vhci-hcd and usbip.
set -u
BUILD=$(realpath "${1:?build dir}")
CYCLES=${2:-20}
cd "$(dirname "$0")"
. ./usbip_lib.sh
usbsim_lock
usbsim_modules
WORK=$(mktemp -d)
"$BUILD/test/usb_sim_usbip" "$USBSIM_PORT" > "$WORK/server.log" 2>&1 &
server=$!
sleep 1
find_dev() {
  for d in /sys/class/tty/ttyACM*; do
    readlink -f "$d/device" | grep -q vhci && basename "$d" && return 0
  done
  return 1
}
fail=0
for i in $(seq 1 "$CYCLES"); do
  usbsim_attach || { echo "cycle $i: attach failed"; fail=1; break; }
  dev=""
  for _ in $(seq 1 50); do dev=$(find_dev) && break; sleep 0.1; done
  [ -n "$dev" ] || { echo "cycle $i: no ttyACM"; fail=1; break; }
  # 先做一次干净的回环：上一次拔出之后设备必须完全可用 / A clean loopback first: the
  # device must be fully usable after the previous unplug.
  if ! sudo DRAIN=1 python3 acm_echo.py "/dev/$dev" 65536 > "$WORK/echo.log" 2>&1; then
    echo "cycle $i: loopback failed: $(cat "$WORK/echo.log")"; fail=1; break
  fi
  grep -o "stale=[0-9]*" "$WORK/echo.log" | grep -v "stale=0" | sed "s/^/cycle $i: /"
  # 传输中途拔出 / Unplug in the middle of a transfer.
  sudo timeout 20 python3 acm_echo.py "/dev/$dev" 8388608 > /dev/null 2>&1 &
  sleep "0.$((RANDOM % 9 + 1))"
  usbsim_detach
  wait $!
  for _ in $(seq 1 50); do find_dev > /dev/null || break; sleep 0.1; done
  kill -0 "$server" 2>/dev/null || { echo "cycle $i: server died"; fail=1; break; }
done
echo "cycles=$i fail=$fail"
usbsim_detach
kill "$server" 2>/dev/null
wait "$server" 2>/dev/null
[ $fail -eq 0 ] || tail -5 "$WORK/server.log"
usbsim_unload
sudo rm -rf "$WORK"
exit $fail
