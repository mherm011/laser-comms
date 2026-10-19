"""Desktop tests for linkproto: RS correctness and a noisy-channel simulation.

Run from the laserlink folder:  python3 tests/sim_test.py
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "pico"))
import linkproto as lp  # noqa: E402

rng = random.Random(1234)


def test_rs():
    for trial in range(3000):
        data = bytes(rng.randrange(256) for _ in range(lp.K))
        cw = bytearray(lp.rs_encode(data))
        nerr = rng.randrange(0, 5)
        for p in rng.sample(range(lp.N), nerr):
            cw[p] ^= rng.randrange(1, 256)
        out, nfix = lp.rs_decode(cw)
        assert out == data and nfix == nerr, (trial, nerr, nfix)
    # 5+ errors must not silently produce wrong data that also passes CRC
    bad = 0
    for trial in range(3000):
        d = bytearray(lp.K)
        d[:22] = bytes(rng.randrange(256) for _ in range(22))
        c = lp.crc16(d, 22)
        d[22], d[23] = c >> 8, c & 0xFF
        cw = bytearray(lp.rs_encode(d))
        for p in rng.sample(range(lp.N), rng.randrange(5, 12)):
            cw[p] ^= rng.randrange(1, 256)
        out, _ = lp.rs_decode(cw)
        if out is not None and out != bytes(d):
            c2 = lp.crc16(out, 22)
            if (c2 >> 8) == out[22] and (c2 & 0xFF) == out[23]:
                bad += 1
    assert bad == 0, bad
    print("RS: 0-4 byte errors always corrected; no undetected bad frames in 3000 overloaded trials")


def channel(stream, byte_err, drop, insert):
    out = bytearray()
    for b in stream:
        r = rng.random()
        if r < drop:
            continue
        if r < drop + insert:
            out.append(rng.randrange(256))
        if rng.random() < byte_err:
            b ^= rng.randrange(1, 256)
        out.append(b)
    return bytes(out)


def test_link(byte_err, drop=0.0, insert=0.0, nmsg=200):
    msgs = ["msg %d: " % i + "the quick brown fox jumps over the lazy dog"[: rng.randrange(1, 44)]
            for i in range(nmsg)]
    stream = bytearray(rng.randrange(256) for _ in range(50))  # leading junk
    for mid, m in enumerate(msgs):
        for seq, (frag, chunk) in enumerate(lp.split_message(m.encode())):
            stream += lp.build_frame(lp.T_MSG, seq, mid, frag, chunk)
    rx = channel(stream, byte_err, drop, insert)
    dec = lp.FrameDecoder()
    asm = lp.MessageAssembler()
    got = {}
    t = 0
    for i in range(0, len(rx), 7):          # feed in small chunks like the UART does
        for fr in dec.feed(rx[i:i + 7]):
            t += 1
            r = asm.add(fr, t)
            if r:
                got[r[0] % 256] = r
    for r in asm.service(10 ** 9):
        got[r[0]] = r
    good = sum(1 for mid, m in enumerate(msgs) if mid in got and got[mid][1] == m and not got[mid][2])
    wrong = sum(1 for mid, m in enumerate(msgs) if mid in got and not got[mid][2] and got[mid][1] != m)
    print("byte_err=%.3f drop=%.3f insert=%.3f -> %d/%d messages perfect, %d wrong, "
          "frames ok=%d corrected_bytes=%d rs_fail=%d"
          % (byte_err, drop, insert, good, nmsg, wrong, dec.ok, dec.corrected, dec.rs_fail))
    assert wrong == 0
    return good / nmsg


if __name__ == "__main__":
    test_rs()
    assert test_link(0.0) == 1.0
    assert test_link(0.02) == 1.0
    assert test_link(0.05) > 0.9
    test_link(0.10)
    test_link(0.01, drop=0.002, insert=0.002)
    print("all tests passed")
