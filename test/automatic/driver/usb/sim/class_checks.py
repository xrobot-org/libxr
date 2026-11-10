"""Host-side checks for usb_sim_classes through the kernel drivers.

hid:   write output reports to the hidraw node and expect them back bitwise inverted.
gsusb: bring the simulated adapter's CAN interface up through SocketCAN, send frames and
       expect each back with ID+1.

The host also has real USB devices, among them a gs_usb adapter with the same VID:PID as
the simulated one. A device or interface is therefore only ever accepted when its sysfs
path lies under the vhci host controller, and nothing else is opened or reconfigured.
The script runs as the calling user; `sudo -n` is used for `ip link` on the simulated
adapter's interface only. A node the user cannot open (hidraw is root-only) is reported
as "could not run" (exit status 3), not as a pass.
Exit status: 0 pass, 1 fail, 3 could not run.
"""

import glob
import os
import random
import select
import socket
import struct
import subprocess
import sys
import time

COULD_NOT_RUN = 3


def on_vhci(path):
    return "/vhci_hcd." in os.path.realpath(path)


def find_hidraw(vid, pid):
    want = f"HID_ID=0003:{vid:08X}:{pid:08X}"
    found = []
    for node in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        device = os.path.join(node, "device")
        if not on_vhci(device):
            continue
        with open(os.path.join(device, "uevent")) as f:
            if want in f.read():
                found.append("/dev/" + os.path.basename(node))
    return found[0] if len(found) == 1 else None


def check_hid(count):
    dev = None
    for _ in range(50):
        dev = find_hidraw(0x1209, 0x0004)
        if dev:
            break
        time.sleep(0.1)
    if not dev:
        print("FAIL hid: no hidraw node under vhci")
        return 1
    try:
        fd = os.open(dev, os.O_RDWR)
    except PermissionError as e:
        print(f"COULD NOT RUN hid: {dev}: {e}")
        return COULD_NOT_RUN
    rng = random.Random(7)
    start = time.time()
    for i in range(count):
        report = bytes(rng.getrandbits(8) for _ in range(8))
        os.write(fd, b"\x00" + report)  # report number 0: no report IDs
        r, _, _ = select.select([fd], [], [], 2.0)
        if not r:
            print(f"FAIL hid: no input report for #{i}")
            return 1
        back = os.read(fd, 64)
        if back != bytes((~b) & 0xFF for b in report):
            print(f"FAIL hid: #{i} got {back.hex()} for {report.hex()}")
            return 1
    os.close(fd)
    print(f"PASS hid: {count} reports in {time.time() - start:.2f}s via {dev}")
    return 0


def find_can(vid, pid):
    found = []
    for net in sorted(glob.glob("/sys/class/net/*")):
        dev = os.path.join(net, "device")
        if not os.path.exists(dev) or not on_vhci(dev):
            continue
        usb = os.path.realpath(os.path.join(dev, ".."))
        try:
            with open(os.path.join(usb, "idVendor")) as f:
                v = f.read().strip()
            with open(os.path.join(usb, "idProduct")) as f:
                p = f.read().strip()
        except OSError:
            continue
        if int(v, 16) == vid and int(p, 16) == pid:
            found.append(os.path.basename(net))
    return found[0] if len(found) == 1 else None


def ip_link(*args):
    return subprocess.run(["sudo", "-n", "ip", "link", *args], check=False).returncode


def check_gsusb(count):
    ifname = None
    for _ in range(50):
        ifname = find_can(0x1D50, 0x606F)
        if ifname:
            break
        time.sleep(0.1)
    if not ifname:
        print("FAIL gsusb: no CAN interface under vhci")
        return 1
    ip_link("set", ifname, "down")
    if ip_link("set", ifname, "type", "can", "bitrate", "500000") != 0:
        print(f"FAIL gsusb: cannot configure {ifname}")
        return 1
    if ip_link("set", ifname, "up") != 0:
        print(f"FAIL gsusb: cannot bring {ifname} up")
        return 1
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((ifname,))
    s.settimeout(2.0)
    fmt = "=IB3x8s"
    rng = random.Random(11)
    start = time.time()
    ok = True
    for i in range(count):
        extended = rng.random() < 0.3
        can_id = rng.getrandbits(29 if extended else 11)
        if not extended:
            can_id = min(can_id, 0x7FE)
        dlc = rng.randint(0, 8)
        data = bytes(rng.getrandbits(8) for _ in range(dlc))
        flags = socket.CAN_EFF_FLAG if extended else 0
        s.send(struct.pack(fmt, can_id | flags, dlc, data.ljust(8, b"\x00")))
        try:
            frame = s.recv(16)
        except TimeoutError:
            print(f"FAIL gsusb: no frame back for #{i}")
            ok = False
            break
        rid, rdlc, rdata = struct.unpack(fmt, frame)
        mask = 0x1FFFFFFF if extended else 0x7FF
        if (rid & mask) != ((can_id + 1) & mask) or rdlc != dlc or rdata[:dlc] != data:
            print(
                f"FAIL gsusb: #{i} got id {rid:#x} dlc {rdlc} for id {can_id:#x} dlc {dlc}"
            )
            ok = False
            break
    s.close()
    ip_link("set", ifname, "down")
    if ok:
        print(f"PASS gsusb: {count} frames in {time.time() - start:.2f}s via {ifname}")
    return 0 if ok else 1


if __name__ == "__main__":
    kind = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
    sys.exit({"hid": check_hid, "gsusb": check_gsusb}[kind](count))
