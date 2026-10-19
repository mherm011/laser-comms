# linkproto.py - framing + Reed-Solomon FEC for the laser link.
# Runs unchanged on MicroPython (Pico) and CPython (desktop tests).
#
# Physical layer: hardware UART, 8N1, 1000 baud, inverted so that
# idle = laser OFF and a start bit = laser ON.
#
# Frame on the wire (34 bytes = 340 ms at 1000 baud):
#   SYNC0 SYNC1 | 32-byte Reed-Solomon codeword
# Codeword = 24 data bytes + 8 parity bytes (corrects any 4 bad bytes).
# Data bytes:
#   [0] type   1=MSG 2=TEST 3=TEST_END
#   [1] seq    frame counter (low 8 bits)
#   [2] mid    message id (MSG) / run id (TEST)
#   [3] frag   fragment index, bit7 = last fragment (MSG)
#              / high bits of the test index (TEST)
#   [4] len    payload length 0..17
#   [5:22]     payload (zero padded)
#   [22:24]    CRC-16/CCITT over bytes 0..21

try:
    from micropython import const
except ImportError:
    def const(x):
        return x

try:
    from time import ticks_diff
except ImportError:
    def ticks_diff(a, b):
        return a - b

SYNC0 = const(0xD3)
SYNC1 = const(0x91)
NSYM = const(8)
K = const(24)
N = const(32)
FRAME_LEN = const(34)
PAYLOAD_MAX = const(17)
SYNC_TOL_BITS = const(2)

T_MSG = const(1)
T_TEST = const(2)
T_END = const(3)

# ---------------------------------------------------------------- GF(256)
_EXP = bytearray(512)
_LOG = bytearray(256)


def _init_tables():
    x = 1
    for i in range(255):
        _EXP[i] = x
        _LOG[x] = i
        x <<= 1
        if x & 0x100:
            x ^= 0x11D
    for i in range(255, 512):
        _EXP[i] = _EXP[i - 255]


_init_tables()


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _EXP[_LOG[a] + _LOG[b]]


def gf_div(a, b):
    if a == 0:
        return 0
    return _EXP[(_LOG[a] + 255 - _LOG[b]) % 255]


def gf_pow(x, p):
    return _EXP[(_LOG[x] * p) % 255]


def gf_inv(x):
    return _EXP[255 - _LOG[x]]


def p_scale(p, x):
    return [gf_mul(c, x) for c in p]


def p_add(p, q):
    n = max(len(p), len(q))
    r = [0] * n
    for i in range(len(p)):
        r[i + n - len(p)] = p[i]
    for i in range(len(q)):
        r[i + n - len(q)] ^= q[i]
    return r


def p_mul(p, q):
    r = [0] * (len(p) + len(q) - 1)
    for j in range(len(q)):
        qj = q[j]
        if qj:
            for i in range(len(p)):
                r[i + j] ^= gf_mul(p[i], qj)
    return r


def p_eval(p, x):
    y = p[0]
    for i in range(1, len(p)):
        y = gf_mul(y, x) ^ p[i]
    return y


def p_div(dividend, divisor):
    out = list(dividend)
    for i in range(len(dividend) - (len(divisor) - 1)):
        coef = out[i]
        if coef:
            for j in range(1, len(divisor)):
                if divisor[j]:
                    out[i + j] ^= gf_mul(divisor[j], coef)
    sep = -(len(divisor) - 1)
    return out[:sep], out[sep:]


# ---------------------------------------------------------------- RS
def _gen_poly(nsym):
    g = [1]
    for i in range(nsym):
        g = p_mul(g, [1, gf_pow(2, i)])
    return g


_GEN = _gen_poly(NSYM)


def rs_encode(data):
    out = bytearray(len(data) + NSYM)
    out[:len(data)] = data
    gen = _GEN
    for i in range(len(data)):
        coef = out[i]
        if coef:
            for j in range(1, len(gen)):
                out[i + j] ^= gf_mul(gen[j], coef)
    out[:len(data)] = data
    return out


def _syndromes(msg):
    s = [0] * (NSYM + 1)
    nz = False
    for i in range(NSYM):
        v = p_eval(msg, _EXP[i])
        s[i + 1] = v
        if v:
            nz = True
    return s, nz


def _error_locator(synd):
    err_loc = [1]
    old_loc = [1]
    for i in range(NSYM):
        k = i + 1
        delta = synd[k]
        for j in range(1, len(err_loc)):
            delta ^= gf_mul(err_loc[-(j + 1)], synd[k - j])
        old_loc = old_loc + [0]
        if delta:
            if len(old_loc) > len(err_loc):
                new_loc = p_scale(old_loc, delta)
                old_loc = p_scale(err_loc, gf_inv(delta))
                err_loc = new_loc
            err_loc = p_add(err_loc, p_scale(old_loc, delta))
    while len(err_loc) and err_loc[0] == 0:
        del err_loc[0]
    if (len(err_loc) - 1) * 2 > NSYM:
        return None
    return err_loc


def _find_errors(err_loc_rev, nmess):
    errs = len(err_loc_rev) - 1
    pos = []
    for i in range(nmess):
        if p_eval(err_loc_rev, gf_pow(2, i)) == 0:
            pos.append(nmess - 1 - i)
    if len(pos) != errs:
        return None
    return pos


def _correct(msg, synd, err_pos):
    n = len(msg)
    coef_pos = [n - 1 - p for p in err_pos]
    e_loc = [1]
    for i in coef_pos:
        e_loc = p_mul(e_loc, p_add([1], [gf_pow(2, i), 0]))
    rs = synd[::-1]
    _, rem = p_div(p_mul(rs, e_loc), [1] + [0] * len(e_loc))
    err_eval = rem[::-1]
    X = [gf_pow(2, -(255 - c)) for c in coef_pos]
    E = [0] * n
    for i in range(len(X)):
        Xi = X[i]
        Xi_inv = gf_inv(Xi)
        prime = 1
        for j in range(len(X)):
            if j != i:
                prime = gf_mul(prime, 1 ^ gf_mul(Xi_inv, X[j]))
        if prime == 0:
            return None
        y = gf_mul(Xi, p_eval(err_eval[::-1], Xi_inv))
        E[err_pos[i]] = gf_div(y, prime)
    for i in range(n):
        msg[i] ^= E[i]
    return msg


def rs_decode(codeword):
    """Return (data_bytes, n_corrected) or (None, -1) if uncorrectable."""
    msg = list(codeword)
    synd, nz = _syndromes(msg)
    if not nz:
        return bytes(msg[:-NSYM]), 0
    err_loc = _error_locator(synd)
    if err_loc is None:
        return None, -1
    err_pos = _find_errors(err_loc[::-1], len(msg))
    if err_pos is None:
        return None, -1
    msg = _correct(msg, synd, err_pos)
    if msg is None:
        return None, -1
    _, nz = _syndromes(msg)
    if nz:
        return None, -1
    return bytes(msg[:-NSYM]), len(err_pos)


# ---------------------------------------------------------------- CRC
def crc16(data, n):
    crc = 0xFFFF
    for i in range(n):
        crc ^= data[i] << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------- frames
def build_frame(ftype, seq, mid, frag, payload):
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("payload too long")
    d = bytearray(K)
    d[0] = ftype
    d[1] = seq & 0xFF
    d[2] = mid & 0xFF
    d[3] = frag & 0xFF
    d[4] = len(payload)
    d[5:5 + len(payload)] = payload
    c = crc16(d, 22)
    d[22] = c >> 8
    d[23] = c & 0xFF
    return bytes([SYNC0, SYNC1]) + bytes(rs_encode(d))


def split_message(text_bytes):
    """Yield (frag_byte, chunk) for a message (max 127 fragments)."""
    chunks = []
    for i in range(0, max(len(text_bytes), 1), PAYLOAD_MAX):
        chunks.append(text_bytes[i:i + PAYLOAD_MAX])
    if len(chunks) > 127:
        raise ValueError("message too long")
    out = []
    for i in range(len(chunks)):
        f = i | (0x80 if i == len(chunks) - 1 else 0)
        out.append((f, chunks[i]))
    return out


def test_payload(run, idx):
    x = ((run & 0xFF) << 16 | (idx & 0xFFFF)) ^ 0x9E3779B9
    x &= 0xFFFFFFFF
    if x == 0:
        x = 1
    out = bytearray(PAYLOAD_MAX)
    for i in range(PAYLOAD_MAX):
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        out[i] = x & 0xFF
    return bytes(out)


_POP = bytes([bin(i).count("1") for i in range(256)])


class Frame:
    def __init__(self, data, corrected):
        self.type = data[0]
        self.seq = data[1]
        self.mid = data[2]
        self.frag = data[3]
        ln = data[4]
        if ln > PAYLOAD_MAX:
            ln = PAYLOAD_MAX
        self.payload = bytes(data[5:5 + ln])
        self.corrected = corrected


class FrameDecoder:
    """Feed raw UART bytes, get decoded Frames back."""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0
        self.corrected = 0
        self.rs_fail = 0
        self.crc_fail = 0
        self.junk = 0

    def _sync_dist(self, b, i):
        # 0 = exact sync. A candidate is accepted for an RS decode attempt if
        # it is within a couple of bits of the sync word OR either sync byte is
        # exact, so a frame whose sync byte got hit by noise is still found.
        d = _POP[b[i] ^ SYNC0] + _POP[b[i + 1] ^ SYNC1]
        if d > SYNC_TOL_BITS and (b[i] == SYNC0 or b[i + 1] == SYNC1):
            d = SYNC_TOL_BITS
        return d

    def feed(self, data):
        self.buf.extend(data)
        if len(self.buf) > 1024:
            drop = len(self.buf) - 1024
            self.junk += drop
            self.buf = self.buf[drop:]
        frames = []
        i = 0
        b = self.buf
        while i + 1 < len(b):
            d = self._sync_dist(b, i)
            if d > SYNC_TOL_BITS:
                i += 1
                continue
            if i + FRAME_LEN > len(b):
                break  # wait for the rest of this candidate
            data, nfix = rs_decode(b[i + 2:i + FRAME_LEN])
            if data is not None:
                c = crc16(data, 22)
                if (c >> 8) == data[22] and (c & 0xFF) == data[23]:
                    self.ok += 1
                    self.corrected += nfix
                    self.junk += i
                    frames.append(Frame(data, nfix))
                    i += FRAME_LEN
                    b = b[i:]
                    i = 0
                    continue
                if d == 0:
                    self.crc_fail += 1
            elif d == 0:
                self.rs_fail += 1
            i += 1
        if i > 0:
            self.junk += i
            b = b[i:]
        self.buf = b
        return frames


class MessageAssembler:
    """Collect MSG fragments into complete messages."""

    def __init__(self, timeout_ms=2500):
        self.parts = {}     # mid -> {frag_index: bytes}
        self.last = {}      # mid -> index of last fragment, if seen
        self.touched = {}   # mid -> time of last fragment
        self.done = []      # recently completed mids (dedup repeats)
        self.timeout = timeout_ms

    def add(self, fr, now):
        mid = fr.mid
        if mid in self.done:
            return None
        idx = fr.frag & 0x7F
        p = self.parts.setdefault(mid, {})
        p[idx] = fr.payload
        self.touched[mid] = now
        if fr.frag & 0x80:
            self.last[mid] = idx
        if mid in self.last:
            n = self.last[mid] + 1
            if len(p) >= n and all(i in p for i in range(n)):
                return self._finish(mid, False)
        return None

    def _finish(self, mid, partial):
        p = self.parts.pop(mid)
        n = self.last.pop(mid, max(p) if p else 0) + 1
        self.touched.pop(mid, None)
        self.done.append(mid)
        if len(self.done) > 32:
            self.done.pop(0)
        raw = b""
        for i in range(n):
            raw += p.get(i, b"?" * 3 if partial else b"")
        try:
            text = raw.decode("utf-8")
        except Exception:
            text = repr(raw)
        return (mid, text, partial)

    def service(self, now):
        out = []
        for mid in list(self.touched):
            if ticks_diff(now, self.touched[mid]) > self.timeout:
                out.append(self._finish(mid, True))
        return out
