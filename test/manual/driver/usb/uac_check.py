"""Check a capture from the simulated UAC microphone: the device sends an incrementing
16-bit sample (a sawtooth), so consecutive samples differ by one (mod 2^16) except where
the stream started or the host dropped data.

usage: uac_check.py <capture.wav>
"""

import itertools
import struct
import sys
import wave

with wave.open(sys.argv[1], "rb") as w:
    rate, channels, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
    frames = w.readframes(w.getnframes())

samples = struct.unpack(f"<{len(frames) // 2}h", frames)
# Skip leading silence before the stream started.
start = next((i for i, s in enumerate(samples) if s != 0), len(samples))
body = samples[start:]
steps = [((b - a) & 0xFFFF) for a, b in itertools.pairwise(body)]
good = sum(1 for d in steps if d == 1)
breaks = [i for i, d in enumerate(steps) if d != 1]
seconds = len(samples) / rate if rate else 0
ratio = good / len(steps) if steps else 0.0
print(
    f"rate={rate} channels={channels} width={width} samples={len(samples)} "
    f"({seconds:.2f}s) leading_zero={start} continuous={ratio:.4%} breaks={len(breaks)} "
    f"first_breaks={breaks[:5]}"
)
ok = (
    rate == 48000
    and channels == 1
    and width == 2
    and len(body) > rate
    and ratio > 0.999
)
print("PASS uac" if ok else "FAIL uac")
sys.exit(0 if ok else 1)
