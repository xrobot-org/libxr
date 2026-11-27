"""Host-side checks for usb_sim_classes through the kernel drivers (run with sudo).

cdc_read:    stream host-to-device bulk writes to ttyACM without stalling or spuriously echoing.
cdc_write:   assert DTR, trigger streaming zeros from ttyACM, clear DTR, verify stop.
cdc_to_uart: exercise termios line-coding changes and loopback data over CDCToUart.
hid:         write output reports to the hidraw node and expect them back bitwise inverted.
hid_std:     EVIOCGRAB all 4 evdev nodes of the composite HID device, trigger CapsLock LED,
             and verify Keyboard, Mouse, Gamepad and GamepadBipolar input_event payloads.
gsusb:       bring can0 up through SocketCAN, send frames and expect each back with ID+1.
dfu_runtime: verify BOS, WebUSB URL and MS OS 2.0 WinUSB descriptors via pyusb.
"""

import fcntl
import glob
import os
import random
import select
import socket
import struct
import subprocess
import sys
import termios
import time
import tty


def find_hidraw(vid, pid):
    want = f"HID_ID=0003:{vid:08X}:{pid:08X}"
    for node in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        with open(os.path.join(node, "device", "uevent")) as f:
            if want in f.read():
                return "/dev/" + os.path.basename(node)
    return None


def check_hid(count):
    dev = None
    for _ in range(50):
        dev = find_hidraw(0x1209, 0x0004)
        if dev:
            break
        time.sleep(0.1)
    if not dev:
        print("FAIL hid: no hidraw node")
        return False
    fd = os.open(dev, os.O_RDWR)
    rng = random.Random(7)
    start = time.time()
    for i in range(count):
        report = bytes(rng.getrandbits(8) for _ in range(8))
        os.write(fd, b"\x00" + report)  # report number 0: no report IDs
        r, _, _ = select.select([fd], [], [], 2.0)
        if not r:
            print(f"FAIL hid: no input report for #{i}")
            return False
        back = os.read(fd, 64)
        if back != bytes((~b) & 0xFF for b in report):
            print(f"FAIL hid: #{i} got {back.hex()} for {report.hex()}")
            return False
    os.close(fd)
    print(f"PASS hid: {count} reports in {time.time() - start:.2f}s via {dev}")
    return True


def find_can(vid, pid):
    for net in sorted(glob.glob("/sys/class/net/*")):
        dev = os.path.join(net, "device")
        if not os.path.exists(dev):
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
            return os.path.basename(net)
    return None


def check_gsusb(count):
    ifname = None
    for _ in range(50):
        ifname = find_can(0x1D50, 0x606F)
        if ifname:
            break
        time.sleep(0.1)
    if not ifname:
        print("FAIL gsusb: no CAN interface")
        return False
    subprocess.run(["ip", "link", "set", ifname, "down"], check=False)
    subprocess.run(
        ["ip", "link", "set", ifname, "type", "can", "bitrate", "500000"], check=True
    )
    subprocess.run(["ip", "link", "set", ifname, "up"], check=True)
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
    subprocess.run(["ip", "link", "set", ifname, "down"], check=False)
    if ok:
        print(f"PASS gsusb: {count} frames in {time.time() - start:.2f}s via {ifname}")
    return ok


def find_vhci_tty(vid, pid):
    want_v = f"{vid:04x}"
    want_p = f"{pid:04x}"
    for tty_dir in sorted(glob.glob("/sys/class/tty/ttyACM*")):
        dev_link = os.path.join(tty_dir, "device")
        if not os.path.exists(dev_link):
            continue
        real = os.path.realpath(dev_link)
        if "vhci" not in real:
            continue
        usb_dev = os.path.realpath(os.path.join(real, ".."))
        try:
            with open(os.path.join(usb_dev, "idVendor")) as f:
                v = f.read().strip().lower()
            with open(os.path.join(usb_dev, "idProduct")) as f:
                p = f.read().strip().lower()
        except OSError:
            continue
        if v == want_v and p == want_p:
            return "/dev/" + os.path.basename(tty_dir)
    return None


def wait_vhci_tty(vid, pid):
    for _ in range(50):
        node = find_vhci_tty(vid, pid)
        if node:
            return node
        time.sleep(0.1)
    return None


def open_raw_tty(dev):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def check_cdc_read(count):
    dev = wait_vhci_tty(0x1209, 0x0008)
    if not dev:
        print("FAIL cdc_read: no vhci ttyACM for 1209:0008")
        return False
    fd = open_raw_tty(dev)
    try:
        total = max(65536, count)
        rng = random.Random(19)
        payload = bytes(rng.getrandbits(8) for _ in range(total))
        chunks = [1, 7, 63, 64, 65, 256, 1024]
        sent = 0
        start = time.time()
        deadline = start + 30.0
        while sent < total and time.time() < deadline:
            r, w, _ = select.select([fd], [fd], [], 0.5)
            if r:
                unexpected = os.read(fd, 4096)
                if unexpected:
                    print(f"FAIL cdc_read: unexpected IN bytes ({len(unexpected)})")
                    return False
            if w:
                n = min(rng.choice(chunks), total - sent)
                try:
                    sent += os.write(fd, payload[sent : sent + n])
                except BlockingIOError:
                    pass
        if sent != total:
            print(f"FAIL cdc_read: sent {sent}/{total} bytes before timeout")
            return False
        r, _, _ = select.select([fd], [], [], 0.25)
        if r and os.read(fd, 4096):
            print("FAIL cdc_read: spurious IN data after drain")
            return False
        elapsed = time.time() - start
        print(
            f"PASS cdc_read: drained {sent} bytes in {elapsed:.2f}s via {dev} "
            f"({sent / 1024 / max(elapsed, 1e-6):.1f} KiB/s)"
        )
        return True
    finally:
        os.close(fd)


def check_cdc_write(count):
    dev = wait_vhci_tty(0x1209, 0x0009)
    if not dev:
        print("FAIL cdc_write: no vhci ttyACM for 1209:0009")
        return False
    fd = open_raw_tty(dev)
    try:
        # Assert DTR explicitly and send one byte so CDCWriteTest::OnDataOutComplete starts
        # continuous IN pumping.
        tiocmbis = getattr(termios, "TIOCMBIS", 0x5416)
        tiocmbic = getattr(termios, "TIOCMBIC", 0x5417)
        tiocm_dtr = getattr(termios, "TIOCM_DTR", 0x002)
        fcntl.ioctl(fd, tiocmbis, struct.pack("i", tiocm_dtr))
        time.sleep(0.05)
        os.write(fd, b"X")

        target = max(131072, count * 64)
        received = 0
        start = time.time()
        deadline = start + 30.0
        while received < target and time.time() < deadline:
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if any(b != 0 for b in chunk):
                print("FAIL cdc_write: received non-zero byte in zero stream")
                return False
            received += len(chunk)
        if received < target:
            print(f"FAIL cdc_write: read {received}/{target} bytes")
            return False

        # Clear DTR, drain any already-queued IN slots, and verify no new IN starts even
        # when another OUT packet arrives.
        fcntl.ioctl(fd, tiocmbic, struct.pack("i", tiocm_dtr))
        time.sleep(0.05)
        quiet_until = time.time() + 0.2
        drain_deadline = time.time() + 2.0
        drained = 0
        while time.time() < quiet_until and time.time() < drain_deadline:
            r, _, _ = select.select([fd], [], [], 0.05)
            if r:
                try:
                    data = os.read(fd, 4096)
                    if data:
                        drained += len(data)
                        quiet_until = time.time() + 0.15
                except BlockingIOError:
                    pass
        os.write(fd, b"Y")
        r, _, _ = select.select([fd], [], [], 0.3)
        if r:
            extra = os.read(fd, 4096)
            if extra:
                print(f"FAIL cdc_write: still streaming {len(extra)} bytes after DTR clear")
                return False
        elapsed = time.time() - start
        print(
            f"PASS cdc_write: {received} zero bytes (+{drained} drained) in "
            f"{elapsed:.2f}s via {dev}"
        )
        return True
    finally:
        os.close(fd)


def run_tty_loopback(fd, total, seed):
    rng = random.Random(seed)
    data = bytes(rng.getrandbits(8) for _ in range(total))
    sent = 0
    received = bytearray()
    chunk_sizes = [1, 7, 63, 64, 128, 500]
    deadline = time.time() + 30.0
    while len(received) < total and time.time() < deadline:
        w = [fd] if sent < total else []
        r, wr, _ = select.select([fd], w, [], 0.5)
        if wr:
            n = min(rng.choice(chunk_sizes), total - sent)
            try:
                sent += os.write(fd, data[sent : sent + n])
            except BlockingIOError:
                pass
        if r:
            try:
                received += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return bytes(received) == data


def check_cdc_to_uart(count):
    dev = wait_vhci_tty(0x1209, 0x000A)
    if not dev:
        print("FAIL cdc_to_uart: no vhci ttyACM for 1209:000A")
        return False
    fd = open_raw_tty(dev)
    try:
        start = time.time()
        # Exercise SET_LINE_CODING: 57600 8E1 then 115200 8N1.
        attrs = termios.tcgetattr(fd)
        attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.CSTOPB | termios.PARODD)) | (
            termios.CS8 | termios.PARENB
        )
        attrs[4] = termios.B57600
        attrs[5] = termios.B57600
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
        time.sleep(0.05)
        if not run_tty_loopback(fd, max(16384, count * 16), 31):
            print("FAIL cdc_to_uart: loopback mismatch at 57600 8E1")
            return False

        attrs[2] = (attrs[2] & ~(termios.CSIZE | termios.PARENB | termios.PARODD | termios.CSTOPB)) | termios.CS8
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
        time.sleep(0.05)
        if not run_tty_loopback(fd, max(32768, count * 32), 32):
            print("FAIL cdc_to_uart: loopback mismatch at 115200 8N1")
            return False

        print(f"PASS cdc_to_uart: 57600 8E1 + 115200 8N1 loopback in {time.time() - start:.2f}s via {dev}")
        return True
    finally:
        os.close(fd)


def check_dfu_runtime(_count):
    import usb.core
    import usb.util

    dev = None
    for _ in range(50):
        dev = usb.core.find(idVendor=0x1209, idProduct=0x000B)
        if dev is not None:
            break
        time.sleep(0.1)
    if dev is None:
        print("FAIL dfu_runtime: USB device 1209:000b not found")
        return False
    try:
        if dev.bcdUSB < 0x0210:
            print(f"FAIL dfu_runtime: bcdUSB {dev.bcdUSB:#06x} < 0x0210")
            return False
        bos = bytes(dev.ctrl_transfer(0x80, 0x06, 0x0F00, 0, 255))
        if len(bos) < 5 or bos[1] != 0x0F:
            print(f"FAIL dfu_runtime: invalid BOS length={len(bos)}")
            return False

        webusb_uuid = bytes.fromhex("38b60834a909a0478bfda0768815b665")
        msos20_uuid = bytes.fromhex("df60ddd88945c74c9cd2659d9e648a9f")
        web_vendor = None
        landing_idx = None
        ms_vendor = None
        off = 5
        while off < len(bos):
            cap_len = bos[off]
            if cap_len == 0 or off + cap_len > len(bos):
                print("FAIL dfu_runtime: malformed BOS capability header")
                return False
            if bos[off + 2] == 0x05 and cap_len >= 20:
                uuid = bos[off + 4 : off + 20]
                if uuid == webusb_uuid and cap_len >= 24:
                    web_vendor = bos[off + 22]
                    landing_idx = bos[off + 23]
                elif uuid == msos20_uuid and cap_len >= 28:
                    ms_vendor = bos[off + 26]
            off += cap_len

        if not web_vendor or not landing_idx or not ms_vendor:
            print(
                f"FAIL dfu_runtime: missing BOS caps web={web_vendor} "
                f"landing={landing_idx} ms={ms_vendor}"
            )
            return False

        url_desc = bytes(dev.ctrl_transfer(0xC0, web_vendor, landing_idx, 0x0002, 255))
        if len(url_desc) < 4 or url_desc[1] != 0x03:
            print(f"FAIL dfu_runtime: bad WebUSB URL desc {url_desc.hex()}")
            return False
        scheme = {0: "http://", 1: "https://"}.get(url_desc[2], "")
        landing = scheme + url_desc[3:].decode("utf-8", errors="replace")
        if landing != "https://example.com/dfu":
            print(f"FAIL dfu_runtime: unexpected WebUSB landing page {landing!r}")
            return False

        ms_set = bytes(dev.ctrl_transfer(0xC0, ms_vendor, 0, 7, 1024))
        if b"WINUSB\x00" not in ms_set or "DeviceInterfaceGUIDs".encode("utf-16-le") not in ms_set:
            print(f"FAIL dfu_runtime: MS OS 2.0 set missing WINUSB/GUID ({len(ms_set)} B)")
            return False

        print(
            f"PASS dfu_runtime: bcdUSB={dev.bcdUSB:#06x} WebUSB={landing} "
            f"MS_OS_2.0={len(ms_set)}B"
        )
        return True
    finally:
        usb.util.dispose_resources(dev)


def find_hid_std_events():
    found = {}
    for ev_path in sorted(glob.glob("/sys/class/input/event*")):
        ev_name = os.path.basename(ev_path)
        real = os.path.realpath(os.path.join(ev_path, "device"))
        if "vhci" not in real:
            continue
        cur = real
        iface_dir = None
        for _ in range(8):
            if os.path.exists(os.path.join(cur, "bInterfaceNumber")):
                iface_dir = cur
                break
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
        if not iface_dir:
            continue
        usb_dev = os.path.dirname(iface_dir)
        try:
            with open(os.path.join(usb_dev, "idVendor")) as f:
                v = f.read().strip().lower()
            with open(os.path.join(usb_dev, "idProduct")) as f:
                p = f.read().strip().lower()
            with open(os.path.join(iface_dir, "bInterfaceNumber")) as f:
                ifnum = int(f.read().strip(), 16)
        except OSError:
            continue
        if v == "1209" and p == "000c":
            found[ifnum] = "/dev/input/" + ev_name
    return found


def check_hid_std(_count):
    nodes = {}
    for _ in range(50):
        nodes = find_hid_std_events()
        if all(i in nodes for i in (0, 1, 2, 3)):
            break
        time.sleep(0.1)
    if not all(i in nodes for i in (0, 1, 2, 3)):
        print(f"FAIL hid_std: expected 4 input event nodes, got {nodes}")
        return False

    EVIOCGRAB = 0x40044590
    EVIOCGABS_X = 0x80184540
    fds = {}
    try:
        for idx in (0, 1, 2, 3):
            fd = os.open(nodes[idx], os.O_RDWR | os.O_NONBLOCK)
            fcntl.ioctl(fd, EVIOCGRAB, 1)
            fds[idx] = fd

        abs_uni = struct.unpack("6i", fcntl.ioctl(fds[2], EVIOCGABS_X, b"\x00" * 24))
        abs_bi = struct.unpack("6i", fcntl.ioctl(fds[3], EVIOCGABS_X, b"\x00" * 24))
        if (abs_uni[1], abs_uni[2]) != (0, 2047) or (abs_bi[1], abs_bi[2]) != (-2048, 2047):
            print(f"FAIL hid_std: unexpected ABS_X ranges uni={abs_uni[1:3]} bi={abs_bi[1:3]}")
            return False

        # Trigger deterministic report emission on all 4 grabbed interfaces by turning
        # CapsLock LED on.
        os.write(fds[0], struct.pack("qqHHi", 0, 0, 0x11, 1, 1))

        events = {0: [], 1: [], 2: [], 3: []}
        ev_fmt = "qqHHi"
        ev_size = struct.calcsize(ev_fmt)
        deadline = time.time() + 4.0

        def have_all():
            kbd_evs = events[0]
            mse_evs = events[1]
            uni_evs = events[2]
            bi_evs = events[3]
            kbd_ok = (1, 30, 1) in kbd_evs and (1, 30, 0) in kbd_evs
            mse_ok = (
                (2, 0, 12) in mse_evs
                and (2, 1, -7) in mse_evs
                and (2, 8, 2) in mse_evs
                and (1, 0x110, 1) in mse_evs
                and (1, 0x110, 0) in mse_evs
            )
            uni_ok = (
                (3, 0, 100) in uni_evs
                and (3, 1, 1900) in uni_evs
                and (3, 2, 500) in uni_evs
                and (3, 3, 1500) in uni_evs
                and sum(1 for t, _, v in uni_evs if t == 1 and v == 1) >= 2
                and sum(1 for t, _, v in uni_evs if t == 1 and v == 0) >= 2
            )
            bi_ok = (
                (3, 0, -1500) in bi_evs
                and (3, 1, 1500) in bi_evs
                and (3, 2, -500) in bi_evs
                and (3, 3, 500) in bi_evs
                and sum(1 for t, _, v in bi_evs if t == 1 and v == 1) >= 1
                and sum(1 for t, _, v in bi_evs if t == 1 and v == 0) >= 1
            )
            return kbd_ok and mse_ok and uni_ok and bi_ok

        fd_to_idx = {fd: idx for idx, fd in fds.items()}
        while not have_all() and time.time() < deadline:
            r, _, _ = select.select(list(fds.values()), [], [], 0.2)
            for fd in r:
                idx = fd_to_idx[fd]
                while True:
                    try:
                        buf = os.read(fd, ev_size * 32)
                    except BlockingIOError:
                        break
                    if not buf:
                        break
                    for pos in range(0, len(buf) - ev_size + 1, ev_size):
                        _, _, etype, ecode, eval_ = struct.unpack(
                            ev_fmt, buf[pos : pos + ev_size]
                        )
                        events[idx].append((etype, ecode, eval_))

        if not have_all():
            print(f"FAIL hid_std: incomplete events={events}")
            return False

        print(
            f"PASS hid_std: kbd={nodes[0]} mouse={nodes[1]} "
            f"gamepad={nodes[2]} gamepad_bi={nodes[3]}"
        )
        return True
    finally:
        if 0 in fds:
            try:
                os.write(fds[0], struct.pack("qqHHi", 0, 0, 0x11, 1, 0))
                time.sleep(0.05)
            except OSError:
                pass
        for fd in fds.values():
            try:
                fcntl.ioctl(fd, EVIOCGRAB, 0)
            except OSError:
                pass
            os.close(fd)


if __name__ == "__main__":
    kind = sys.argv[1]
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
    checks = {
        "cdc_read": check_cdc_read,
        "cdc_write": check_cdc_write,
        "cdc_to_uart": check_cdc_to_uart,
        "hid": check_hid,
        "hid_std": check_hid_std,
        "gsusb": check_gsusb,
        "dfu_runtime": check_dfu_runtime,
    }
    ok = checks[kind](count)
    sys.exit(0 if ok else 1)
