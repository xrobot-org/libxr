#!/usr/bin/env bash
# classes_e2e.sh <build-dir>: HID, gs_usb, DFU and UAC device classes through the kernel
# drivers. Starts usb_sim_classes for each class, attaches it through vhci and checks it
# from the host side (hidraw, SocketCAN, dfu-util, arecord). Needs vhci-hcd, usbip,
# dfu-util and arecord. Each server runs on a free TCP port with a random busid and is
# attached only after the script has checked that it is the server it started
# (usbip_lib.sh); a system usbipd on the stock port is never reached.
# 经内核驱动检查 HID、gs_usb、DFU 和 UAC 设备类：每个类启动一个 usb_sim_classes，经 vhci
# 挂接，从主机侧检查（hidraw、SocketCAN、dfu-util、arecord）。每个服务端用空闲的 TCP 端口
# 和随机 busid，脚本确认它就是自己启动的那个之后才挂接（usbip_lib.sh），不会连到系统
# usbipd 的默认端口。
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
# CLASS_PRIV="sudo -n" runs exactly three programs with that prefix, each already limited
# to the simulated device: class_checks.py hid (opens only a hidraw node under vhci),
# dfu-util (-d 1209:0005 -S <serial> -p <bus-port of the vhci device>) and arecord (the
# sound card whose sysfs path lies under vhci). CLASSES="hid gsusb dfu uac" selects classes.
# CLASS_PRIV="sudo -n" 只给三个程序加前缀，且每个都只指向仿真设备：class_checks.py hid
# （只打开 vhci 下的 hidraw 节点）、dfu-util（-d 1209:0005 -S 序列号 -p vhci 设备的
# 总线端口）和 arecord（sysfs 路径在 vhci 之下的声卡）。CLASSES 选择要跑的类。
# A missing tool (usbip, sudo -n, dfu-util, arecord), a missing vhci_hcd, gs_usb or
# snd_usb_audio kernel module or hidraw support, and a device node the user may not open
# are COULD NOT RUN too. Nothing is installed or loaded.
# 缺工具（usbip、sudo -n、dfu-util、arecord）、缺 vhci_hcd、gs_usb 或 snd_usb_audio 内核模块
# 或 hidraw 支持、用户无权打开的设备节点，同样是 COULD NOT RUN。不安装也不加载任何东西。
# Exit status: 0 everything passed, 1 a check failed, 3 nothing failed but a class could not run.
set -u
BUILD=${1:?build dir}
cd "$(dirname "$0")"
. ./usbip_lib.sh
PRIV=${CLASS_PRIV:-}
sim_check_host || exit $?
[ -x "$BUILD/test/usb_sim_classes" ] || { echo "COULD NOT RUN: $BUILD/test/usb_sim_classes is not built"; exit 3; }
if [ -n "$PRIV" ] && ! $PRIV true 2>/dev/null; then
  echo "COULD NOT RUN: CLASS_PRIV ($PRIV) does not work here"
  exit 3
fi
CLASSES=${CLASSES:-hid gsusb dfu uac}
want() { case " $CLASSES " in *" $1 "*) return 0;; esac; return 1; }
WORK=$(mktemp -d)
fail=0
skipped=0
summary=()

cleanup() {
  sim_detach
  sim_stop_server
  rm -rf "$WORK"
}
trap cleanup EXIT

start() {
  sim_detach
  sim_start_server "$WORK/server.log" "$BUILD/test/usb_sim_classes" "$1" @PORT@ ||
    { echo "FAIL $1: server did not start"; summary+=("FAIL $1: server did not start"); fail=1; return 1; }
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

# HID ---------------------------------------------------------------------------------
if want hid && start hid; then
  dev=$(sim_our_device)
  $PRIV python3 class_checks.py hid 1000 "$dev"
  status=$?
  result "hid (hidraw, 1000 reports)" "$status"
  if [ "$status" -eq 3 ]; then
    node=$(for h in /sys/class/hidraw/hidraw*; do [ -n "$dev" ] && sim_is_under "$h/device" "$dev" && echo "$h"; done | head -1)
    if [ -n "$node" ]; then
      note "kernel: $(basename "$node") bound by $(basename "$(readlink -f "$node/device/driver")"), report descriptor $(cat "$node/device/report_descriptor" | wc -c) bytes"
    else
      note "kernel: no hidraw node under the simulated device"
    fi
  fi
fi

# gs_usb -------------------------------------------------------------------------------
if want gsusb && start gsusb; then
  python3 class_checks.py gsusb 1000 "$(sim_our_device)"
  result "gs_usb (SocketCAN, 1000 frames)" $?
fi

# DFU ----------------------------------------------------------------------------------
if want dfu && ! reason=$(sim_need_tools dfu-util); then
  result "dfu (dfu-util download and upload, 100000 bytes)" 3
  note "$reason"
elif want dfu && start dfu; then
  head -c 100000 /dev/urandom > "$WORK/img.bin"
  # dfu-util only gets the device imported from this run's server: its bus-port path and
  # serial string, in addition to VID:PID. 只给 dfu-util 本次运行的服务端导入的设备：总线端口
  # 路径和序列号。
  dev=$(sim_our_device)
  if [ -z "$dev" ] || [ -z "$(cat "$dev/serial" 2>/dev/null)" ]; then
    result "dfu (dfu-util download and upload, 100000 bytes)" 1
    note "kernel: no DFU device with a serial string imported from the simulated server"
  else
    DFU_SEL=(-d 1209:0005 -S "$(cat "$dev/serial")" -p "$(basename "$dev")" -a 0)
    timeout 120 $PRIV dfu-util "${DFU_SEL[@]}" -D "$WORK/img.bin" > "$WORK/dfu.log" 2>&1
    if grep -q "LIBUSB_ERROR_ACCESS" "$WORK/dfu.log"; then
      result "dfu (dfu-util download and upload, 100000 bytes)" 3
      note "kernel: enumerated as $(basename "$dev"), $(cat "$dev/descriptors" | wc -c) descriptor bytes read, interface class $(cat "$dev"/*:1.0/bInterfaceClass 2>/dev/null)/$(cat "$dev"/*:1.0/bInterfaceSubClass 2>/dev/null)"
    else
      timeout 120 $PRIV dfu-util "${DFU_SEL[@]}" -U "$WORK/back.bin" >> "$WORK/dfu.log" 2>&1
      cmp -s -n 100000 "$WORK/img.bin" "$WORK/back.bin"
      result "dfu (dfu-util download and upload, 100000 bytes)" $?
    fi
  fi
fi

# UAC ----------------------------------------------------------------------------------
if want uac && ! reason=$(sim_need_tools arecord); then
  result "uac (arecord, 3 s at 48 kHz)" 3
  note "$reason"
elif want uac && start uac; then
  card=""
  # The card must be USB id 1209:0006 and its sysfs device must lie below the device
  # imported from this run's server. 声卡必须是 USB id 1209:0006，且 sysfs 设备在本次运行的
  # 服务端导入的设备之下。
  dev=$(sim_our_device)
  for c in /proc/asound/card[0-9]*; do
    n=${c##*/card}
    [ -n "$dev" ] && [ "$(cat "$c/usbid" 2>/dev/null)" = "1209:0006" ] &&
      sim_is_under "/sys/class/sound/card$n/device" "$dev" && card=$n
  done
  if [ -z "$card" ] && [ ! -d /sys/module/snd_usb_audio ]; then
    result "uac (arecord, 3 s at 48 kHz)" 3
    note "the snd_usb_audio kernel module is not loaded, so the kernel made no sound card"
  elif [ -z "$card" ]; then
    result "uac (arecord, 3 s at 48 kHz)" 1
    note "kernel: no sound card with USB id 1209:0006 under the simulated device"
  else
    timeout 30 $PRIV arecord -D "hw:$card,0" -f S16_LE -r 48000 -c 1 -d 3 "$WORK/uac.wav" > "$WORK/arecord.log" 2>&1
    arecord_status=$?
    [ "$arecord_status" -ne 0 ] && note "arecord exit status $arecord_status: $(tr '\n' ' ' < "$WORK/arecord.log" | cut -c1-200)"
    if [ ! -s "$WORK/uac.wav" ] && grep -qiE "permission denied|audio open error|cannot get card index" "$WORK/arecord.log"; then
      result "uac (arecord, 3 s at 48 kHz)" 3
      note "kernel: card $card bound, $(grep -E 'Rates|Format|Endpoint' /proc/asound/card$card/stream0 | tr -s ' ' | tr '\n' ';')"
    else
      python3 uac_check.py "$WORK/uac.wav"
      result "uac (arecord, 3 s at 48 kHz)" $?
    fi
  fi
fi

echo "---- summary"
printf '%s\n' "${summary[@]}"
[ "$fail" -ne 0 ] && exit 1
[ "$skipped" -ne 0 ] && exit 3
exit 0
