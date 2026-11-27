#!/usr/bin/env bash
# classes_e2e.sh <build-dir>: CDC, CDCReadTest, CDCWriteTest, CDCToUart, HID, composite
# standard HID, gs_usb, DFU, DfuRuntimeClass and UAC device classes through the kernel
# drivers. Starts usb_sim_usbip (CDC) or usb_sim_classes for each class, attaches it
# through vhci and checks it from the host side. USBSIM_STOP / USBSIM_PEND pick the stop
# mode and PendInterrupt (see usbip_server.hpp); USBSIM_ONLY="dfu uac" runs only those
# classes. Needs sudo, vhci-hcd, usbip, dfu-util, python3-usb and arecord.
set -u
BUILD=$(realpath "${1:?build dir}")
cd "$(dirname "$0")"
. ./usbip_lib.sh
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
start() {
  stop_server
  "$@" "$USBSIM_PORT" 9>&- > "$WORK/server.log" 2>&1 &
  server=$!
  sleep 1
  usbsim_attach || { echo "FAIL $*: attach"; fail=1; return 1; }
  sleep 3
}
result() {
  if [ "$2" -eq 0 ]; then echo "PASS $1"; else
    echo "FAIL $1"; fail=1; tail -5 "$WORK/server.log" | sed "s/^/    /"
  fi
}
want() { case " ${USBSIM_ONLY:-cdc cdc_read cdc_write cdc_to_uart hid hid_std gsusb dfu dfu_runtime uac} " in *" $1 "*) return 0 ;; esac; return 1; }
find_acm() {
  for d in /sys/class/tty/ttyACM*; do
    readlink -f "$d/device" | grep -q vhci && basename "$d" && return 0
  done
  return 1
}

if want cdc && start "$BUILD/test/usb_sim_usbip"; then
  dev=""
  for _ in $(seq 1 50); do dev=$(find_acm) && break; sleep 0.1; done
  if [ -n "$dev" ]; then
    sudo python3 acm_echo.py "/dev/$dev" 1048576 > "$WORK/acm.log" 2>&1
    rc=$?
    sed "s/^/    /" "$WORK/acm.log"
    result "cdc (ttyACM loopback, 1 MiB)" $rc
  else
    result "cdc (no vhci ttyACM)" 1
  fi
fi

want cdc_read && start "$BUILD/test/usb_sim_classes" cdc_read && {
  sudo python3 class_checks.py cdc_read 262144
  result "cdc_read (CDCReadTest, 256 KiB OUT)" $?
}
want cdc_write && start "$BUILD/test/usb_sim_classes" cdc_write && {
  sudo python3 class_checks.py cdc_write 2048
  result "cdc_write (CDCWriteTest, DTR stream + stop)" $?
}
want cdc_to_uart && start "$BUILD/test/usb_sim_classes" cdc_to_uart && {
  sudo python3 class_checks.py cdc_to_uart 1000 &&
    grep -q "cdc_to_uart: line coding baud=57600" "$WORK/server.log" &&
    grep -q "cdc_to_uart: line coding baud=115200" "$WORK/server.log"
  result "cdc_to_uart (CDCToUart loopback + line coding)" $?
}

want hid && start "$BUILD/test/usb_sim_classes" hid && {
  sudo python3 class_checks.py hid 1000
  result "hid (hidraw, 1000 reports)" $?
}
want hid_std && start "$BUILD/test/usb_sim_classes" hid_std && {
  sudo python3 class_checks.py hid_std 1
  result "hid_std (composite Keyboard+Mouse+Gamepad+Bipolar)" $?
}
want gsusb && start "$BUILD/test/usb_sim_classes" gsusb && {
  sudo python3 class_checks.py gsusb 1000
  result "gs_usb (SocketCAN, 1000 frames)" $?
}

if want dfu && start "$BUILD/test/usb_sim_classes" dfu; then
  head -c 100000 /dev/urandom > "$WORK/img.bin"
  # 本类不是 manifestation tolerant：下载后由主机复位（-R），才能离开 MANIFEST-WAIT-RESET。
  # The class is not manifestation tolerant: the host resets after the download (-R)
  # so that it leaves MANIFEST-WAIT-RESET.
  sudo timeout 120 dfu-util -d 1209:0005 -a 0 -D "$WORK/img.bin" -R > "$WORK/dfu.log" 2>&1
  sleep 2
  sudo timeout 120 dfu-util -d 1209:0005 -a 0 -U "$WORK/back.bin" >> "$WORK/dfu.log" 2>&1
  sudo cmp -s -n 100000 "$WORK/img.bin" "$WORK/back.bin"
  rc=$?
  [ $rc -eq 0 ] || tail -25 "$WORK/dfu.log" | sed "s/^/    /"
  grep -c "bus reset" "$WORK/server.log" | sed "s/^/    server bus resets: /"
  result "dfu (dfu-util download and upload, 100000 bytes)" $rc
fi

if want dfu_runtime && start "$BUILD/test/usb_sim_classes" dfu_runtime; then
  sudo python3 class_checks.py dfu_runtime 1 > "$WORK/dfu_rt.log" 2>&1
  rc=$?
  if [ $rc -eq 0 ]; then
    sudo timeout 30 dfu-util -d 1209:000b -e >> "$WORK/dfu_rt.log" 2>&1
    rc=$?
  fi
  if [ $rc -eq 0 ]; then
    rc=1
    for _ in $(seq 1 30); do
      if grep -q "dfu: jump to application requested" "$WORK/server.log"; then
        rc=0
        break
      fi
      sleep 0.1
    done
  fi
  sed "s/^/    /" "$WORK/dfu_rt.log"
  result "dfu_runtime (BOS+WebUSB+WinUSB & dfu-util -e)" $rc
fi

if want uac && start "$BUILD/test/usb_sim_classes" uac; then
  card=$(awk '/USB sim device/ {print $1; exit}' /proc/asound/cards)
  if [ -n "$card" ]; then
    sudo timeout 30 arecord -D "hw:$card,0" -f S16_LE -r 48000 -c 1 -d 3 "$WORK/uac.wav" \
      > /dev/null 2>&1
    python3 uac_check.py "$WORK/uac.wav"
    result "uac (arecord, 3 s at 48 kHz)" $?
  else
    result "uac (no sound card)" 1
  fi
fi

stop_server
usbsim_unload
sudo rm -rf "$WORK"
exit $fail
