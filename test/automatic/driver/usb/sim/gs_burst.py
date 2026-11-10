"""gs_usb burst experiment against `usb_sim_classes gsusb2` through the kernel gs_usb driver.

Sends a burst of classic CAN frames on channel 0 as fast as the adapter accepts them
(blocking sends) while the simulated bus loops them back as received frames on channel 1,
at a limited rate. Reports what the kernel saw on channel 1: the frames that arrived,
the missing sequence numbers, rx_over_errors and the overflow error frames.

usage: gs_burst.py [frames] [seconds to wait after the burst]

The host has a real gs_usb adapter with the same VID:PID; only interfaces whose sysfs
path lies under the vhci host controller are used. sudo -n is used for `ip link` on those
two interfaces only.
"""

import errno
import glob
import os
import socket
import struct
import subprocess
import sys
import threading
import time

FMT = "=IB3x8s"
SOL_CAN_RAW = 101
CAN_RAW_ERR_FILTER = 2
CAN_ERR_FLAG = 0x20000000
CAN_ERR_CRTL = 0x04
CAN_ERR_CRTL_RX_OVERFLOW = 0x01


def sim_interfaces():
    found = []
    for net in sorted(glob.glob("/sys/class/net/*")):
        dev = os.path.join(net, "device")
        if not os.path.exists(dev) or "/vhci_hcd." not in os.path.realpath(dev):
            continue
        usb = os.path.realpath(os.path.join(dev, ".."))
        try:
            with open(os.path.join(usb, "idVendor")) as f:
                vid = f.read().strip()
            with open(os.path.join(usb, "idProduct")) as f:
                pid = f.read().strip()
            with open(os.path.join(net, "dev_port")) as f:
                port = int(f.read().strip(), 0)
        except (OSError, ValueError):
            continue
        if (vid, pid) == ("1d50", "606f"):
            found.append((port, os.path.basename(net)))
    return [name for _, name in sorted(found)]


def ip_link(*args):
    subprocess.run(["sudo", "-n", "ip", "link", *args], check=True)


def counter(ifname, name):
    with open(f"/sys/class/net/{ifname}/statistics/{name}") as f:
        return int(f.read())


def snapshot(ifname):
    names = ("rx_packets", "rx_errors", "rx_over_errors", "rx_dropped", "tx_packets", "tx_errors")
    return {n: counter(ifname, n) for n in names}


def send_retry(sock, frame, limit=5.0):
    begin = time.time()
    while True:
        try:
            sock.send(frame)
            return True
        except OSError as e:
            if e.errno != errno.ENOBUFS or time.time() - begin > limit:
                print(f"send failed: {e}")
                return False


def main():
    frames =int(sys.argv[1]) if len(sys.argv) > 1 else 600
    settle = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    names = None
    for _ in range(50):
        names = sim_interfaces()
        if len(names) == 2:
            break
        time.sleep(0.1)
    if names is None or len(names) != 2:
        print(f"FAIL: expected two simulated CAN interfaces under vhci, found {names}")
        return 1
    ch0, ch1 = names
    for name in names:
        ip_link("set", name, "down")
        ip_link("set", name, "type", "can", "bitrate", "500000")
        ip_link("set", name, "up")

    rx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    rx.setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, 0x1FFFFFFF)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    rx.bind((ch1,))
    rx.settimeout(0.2)
    tx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    tx.bind((ch0,))

    before0, before1 = snapshot(ch0), snapshot(ch1)
    received = []
    overflow_frames = [0]
    other_errors = [0]
    last_rx = [time.time()]
    stop = threading.Event()

    def receiver():
        while not stop.is_set():
            try:
                frame = rx.recv(16)
            except (TimeoutError, socket.timeout):
                continue
            can_id, dlc, data = struct.unpack(FMT, frame)
            last_rx[0] = time.time()
            if can_id & CAN_ERR_FLAG:
                if (can_id & CAN_ERR_CRTL) and data[1] & CAN_ERR_CRTL_RX_OVERFLOW:
                    overflow_frames[0] += 1
                else:
                    other_errors[0] += 1
                continue
            if dlc == 4:
                received.append(struct.unpack("<I", data[:4])[0])

    thread = threading.Thread(target=receiver)
    thread.start()
    start = time.time()
    # The CAN interface has a short transmit queue and send() fails with ENOBUFS while the
    # adapter has not taken the earlier frames: retry at once, which is as fast as the
    # adapter accepts them. 5 s without progress counts as stalled.
    sent = 0
    for seq in range(frames):
        frame = struct.pack(FMT, 0x123, 4, struct.pack("<I", seq).ljust(8, b"\x00"))
        if not send_retry(tx, frame):
            print(f"send stalled after {sent} frames")
            break
        sent += 1
    send_time = time.time() - start
    deadline = time.time() + settle
    while time.time() < deadline:
        time.sleep(0.05)
        if time.time() - last_rx[0] > 1.0 and len(received) >= sent:
            break
    time.sleep(0.3)
    stop.set()
    thread.join()
    after0, after1 = snapshot(ch0), snapshot(ch1)
    for name in names:
        ip_link("set", name, "down")

    missing = sorted(set(range(sent)) - set(received))
    delta0 = {k: after0[k] - before0[k] for k in after0}
    delta1 = {k: after1[k] - before1[k] for k in after1}
    print(f"channel 0 {ch0}: sent {sent} of {frames} frames in {send_time:.3f}s "
          f"({sent / max(send_time, 1e-9):.0f}/s), kernel counters delta {delta0}")
    print(f"channel 1 {ch1}: received {len(received)} of {sent} frames, "
          f"{len(missing)} missing, overflow error frames {overflow_frames[0]}, "
          f"other error frames {other_errors[0]}, kernel counters delta {delta1}")
    if missing:
        runs = []
        run_start = prev = missing[0]
        for seq in missing[1:]:
            if seq != prev + 1:
                runs.append((run_start, prev))
                run_start = seq
            prev = seq
        runs.append((run_start, prev))
        print(f"missing sequence ranges (first 10): {runs[:10]}")
    print(f"RESULT rx_over_errors={delta1["rx_over_errors"]} missing={len(missing)} "
          f"received={len(received)} sent={sent}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
