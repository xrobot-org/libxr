"""Loopback test for the simulated CDC device: write random data to a ttyACM, read it back."""

import os
import random
import select
import sys
import termios
import time
import tty

dev = sys.argv[1]
total = int(sys.argv[2]) if len(sys.argv) > 2 else 65536
chunk_sizes = [1, 7, 63, 64, 65, 128, 500, 1000]

fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd)
attrs = termios.tcgetattr(fd)
attrs[3] &= ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, attrs)
termios.tcflush(fd, termios.TCIOFLUSH)
# Optional: drop bytes left over from a previous session (CDCUart keeps queued data across a
# reconfiguration) before the loopback starts.
stale = 0
if os.environ.get("DRAIN"):
    quiet_until = time.time() + 0.3
    while time.time() < quiet_until:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                stale += len(os.read(fd, 4096))
                quiet_until = time.time() + 0.3
            except BlockingIOError:
                pass

rng = random.Random(1234)
data = bytes(rng.getrandbits(8) for _ in range(total))
sent = 0
received = bytearray()
start = time.time()
deadline = start + 60
while len(received) < total and time.time() < deadline:
    w = [fd] if sent < total else []
    r, wr, _ = select.select([fd], w, [], 0.5)
    if wr:
        n = rng.choice(chunk_sizes)
        try:
            sent += os.write(fd, data[sent : sent + n])
        except BlockingIOError:
            pass
    if r:
        try:
            received += os.read(fd, 4096)
        except BlockingIOError:
            pass
elapsed = time.time() - start
os.close(fd)
ok = bytes(received) == data
print(
    f"stale={stale} sent={sent} received={len(received)} match={ok} elapsed={elapsed:.2f}s "
    f"rate={len(received) / 1024 / max(elapsed, 1e-6):.1f} KiB/s"
)
if not ok:
    for i, (a, b) in enumerate(zip(received, data)):
        if a != b:
            print(f"first mismatch at {i}: got {a:#x} want {b:#x}")
            break
sys.exit(0 if ok else 1)
