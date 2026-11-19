#!/usr/bin/env bash
# usbip_lib.sh: helpers for the USB/IP scripts (sourced, not run). The simulated device
# is served on 127.0.0.1:$USBSIM_PORT (default 3249) and attached through vhci; only the
# vhci port whose remote is that address is ever detached, so ports of other users stay
# alone. Every script takes /tmp/usb-measure.lock (flock) for the whole time a USB/IP
# device is attached.

USBSIM_PORT=${USBSIM_PORT:-3249}

# usbsim_lock: wait (up to 30 min) for /tmp/usb-measure.lock and keep it on fd 9;
# USBSIM_LOCK_HELD=1 means a calling script already holds it.
usbsim_lock() {
  [ "${USBSIM_LOCK_HELD:-0}" = 1 ] && return 0
  exec 9>>/tmp/usb-measure.lock
  flock -w 1800 9 || { echo "FAIL: /tmp/usb-measure.lock busy"; exit 1; }
}

# usbsim_modules: load vhci-hcd if needed; remember whether this script loaded it.
usbsim_modules() {
  USBSIM_LOADED=0
  if ! lsmod | grep -q '^vhci_hcd'; then
    sudo modprobe vhci-hcd && USBSIM_LOADED=1
  fi
}

# usbsim_port: the vhci port number whose remote is our server, empty if none.
usbsim_port() {
  sudo usbip port 2>/dev/null | awk -v want="usbip://127.0.0.1:$USBSIM_PORT/" '
    /^Port [0-9]+:/ { port = $2; sub(":", "", port); sub("^0*", "", port);
                      if (port == "") port = 0 }
    index($0, want) { print port; exit }'
}

# usbsim_attach: attach the server's device 1-1.
usbsim_attach() {
  sudo usbip --tcp-port "$USBSIM_PORT" attach -r 127.0.0.1 -b 1-1 >/dev/null 2>&1
}

# usbsim_detach: detach our port, if attached, and wait until it is gone.
usbsim_detach() {
  local p
  p=$(usbsim_port)
  [ -n "$p" ] || return 0
  sudo usbip detach -p "$p" >/dev/null 2>&1
  for _ in $(seq 1 50); do [ -z "$(usbsim_port)" ] && return 0; sleep 0.1; done
  return 1
}

# usbsim_unload: unload vhci-hcd again if this script loaded it and no port is in use.
usbsim_unload() {
  [ "${USBSIM_LOADED:-0}" = 1 ] || return 0
  if ! sudo usbip port 2>/dev/null | grep -q 'Port in Use'; then
    sudo modprobe -r vhci-hcd usbip-core 2>/dev/null
  fi
}
