#!/usr/bin/env bash
# classes_e2e.sh <build-dir>: HID, gs_usb, DFU and UAC device classes through the kernel
# drivers. Starts usb_sim_classes for each class, attaches it through vhci and checks it
# from the host side (hidraw, SocketCAN, dfu-util, arecord). Needs vhci-hcd, usbip,
# dfu-util and arecord; USBIP_SOF=stops|control|always selects the frame interrupt policy
# of the simulated controller (default stops, see SofPolicy).
# 经内核驱动检查 HID、gs_usb、DFU 和 UAC 设备类：每个类启动一个 usb_sim_classes，经 vhci
# 挂接，从主机侧检查（hidraw、SocketCAN、dfu-util、arecord）。USBIP_SOF 选择仿真控制器的
# 帧中断策略。
#
# Privileges: the script runs as the calling user, and sudo -n is used for the usbip tool
# and for ip link on the simulated adapter's CAN interface only (usbip_lib.sh,
# class_checks.py). hidraw, usbfs (dfu-util) and ALSA capture need more than that on most
# hosts (root or the audio group); when the open fails the class is reported as
# "COULD NOT RUN" together with what the kernel did show (the driver bound, the
# descriptors it read), never as a pass.
# 权限：脚本以调用者身份运行，sudo -n 只用于 usbip 工具和仿真适配器 CAN 接口上的 ip link。
# hidraw、usbfs（dfu-util）和 ALSA 采集在多数主机上需要更高权限（root 或 audio 组）；打开
# 失败时该类报告为"COULD NOT RUN"，并附上内核能看到的部分（驱动绑定、读到的描述符），
# 不算通过。
# Exit status: 0 everything passed, 1 a check failed, 3 nothing failed but a class could not run.
set -u
BUILD=${1:?build dir}
cd "$(dirname "$0")"
. ./usbip_lib.sh
WORK=$(mktemp -d)
SERVER_PID=""
fail=0
skipped=0
summary=()

cleanup() {
  sim_detach
  [ -n "$SERVER_PID" ] && kill "$SERVER_PID" 2>/dev/null
  rm -rf "$WORK"
}
trap cleanup EXIT

start() {
  sim_detach
  if [ -n "$SERVER_PID" ]; then kill "$SERVER_PID" 2>/dev/null; wait "$SERVER_PID" 2>/dev/null; fi
  "$BUILD/test/usb_sim_classes" "$1" "$SIM_TCP_PORT" > "$WORK/server.log" 2>&1 &
  SERVER_PID=$!
  sleep 1
  sim_attach || { echo "FAIL $1: attach"; summary+=("FAIL $1: attach"); fail=1; return 1; }
  sleep 3
}
result() {  # result <label> <status>: 0 pass, 3 could not run, else fail
  case "$2" in
    0) echo "PASS $1"; summary+=("PASS $1");;
    3) echo "COULD NOT RUN $1"; summary+=("COULD NOT RUN $1"); skipped=1;;
    *) echo "FAIL $1"; summary+=("FAIL $1"); fail=1;;
  esac
}
note() { echo "  $*"; summary+=("  $*"); }

# The sysfs directory of the simulated device (under vhci) with this VID:PID, if any.
sim_usb_device() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$1" ] && [ "$(cat "$d/idProduct")" = "$2" ] || continue
    sim_is_vhci "$d" && { echo "$d"; return 0; }
  done
  return 1
}

# HID ---------------------------------------------------------------------------------
if start hid; then
  python3 class_checks.py hid 1000
  status=$?
  result "hid (hidraw, 1000 reports)" "$status"
  if [ "$status" -eq 3 ]; then
    node=$(for h in /sys/class/hidraw/hidraw*; do sim_is_vhci "$h/device" && echo "$h"; done | head -1)
    if [ -n "$node" ]; then
      note "kernel: $(basename "$node") bound by $(basename "$(readlink -f "$node/device/driver")"), report descriptor $(cat "$node/device/report_descriptor" | wc -c) bytes"
    else
      note "kernel: no hidraw node under vhci"
    fi
  fi
fi

# gs_usb -------------------------------------------------------------------------------
if start gsusb; then
  python3 class_checks.py gsusb 1000
  result "gs_usb (SocketCAN, 1000 frames)" $?
fi

# DFU ----------------------------------------------------------------------------------
if start dfu; then
  head -c 100000 /dev/urandom > "$WORK/img.bin"
  timeout 120 dfu-util -d 1209:0005 -a 0 -D "$WORK/img.bin" > "$WORK/dfu.log" 2>&1
  if grep -q "LIBUSB_ERROR_ACCESS" "$WORK/dfu.log"; then
    result "dfu (dfu-util download and upload, 100000 bytes)" 3
    dev=$(sim_usb_device 1209 0005)
    [ -n "$dev" ] && note "kernel: enumerated as $(basename "$dev"), $(cat "$dev/descriptors" | wc -c) descriptor bytes read, interface class $(cat "$dev"/*:1.0/bInterfaceClass 2>/dev/null)/$(cat "$dev"/*:1.0/bInterfaceSubClass 2>/dev/null)"
  else
    timeout 120 dfu-util -d 1209:0005 -a 0 -U "$WORK/back.bin" >> "$WORK/dfu.log" 2>&1
    cmp -s -n 100000 "$WORK/img.bin" "$WORK/back.bin"
    result "dfu (dfu-util download and upload, 100000 bytes)" $?
  fi
fi

# UAC ----------------------------------------------------------------------------------
if start uac; then
  card=""
  for c in /proc/asound/card[0-9]*; do
    [ "$(cat "$c/usbid" 2>/dev/null)" = "1209:0006" ] && card=${c##*/card}
  done
  if [ -z "$card" ]; then
    result "uac (arecord, 3 s at 48 kHz)" 1
    note "kernel: no sound card with USB id 1209:0006"
  else
    timeout 30 arecord -D "hw:$card,0" -f S16_LE -r 48000 -c 1 -d 3 "$WORK/uac.wav" > "$WORK/arecord.log" 2>&1
    if [ ! -s "$WORK/uac.wav" ] && grep -qiE "permission denied|audio open error|cannot get card index" "$WORK/arecord.log"; then
      result "uac (arecord, 3 s at 48 kHz)" 3
      note "kernel: card $card bound, $(grep -E 'Rates|Format|Endpoint' /proc/asound/card$card/stream0 | tr -s ' ' | tr '\n' ';')"
    else
      python3 uac_check.py "$WORK/uac.wav"
      result "uac (arecord, 3 s at 48 kHz)" $?
    fi
  fi
fi

echo "---- summary (USBIP_SOF=${USBIP_SOF:-stops})"
printf '%s\n' "${summary[@]}"
[ "$fail" -ne 0 ] && exit 1
[ "$skipped" -ne 0 ] && exit 3
exit 0
