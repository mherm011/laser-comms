# main.py - laser link firmware for Raspberry Pi Pico (MicroPython).
# Same file on both boards; config.py picks the role (TX / RX / BOTH).
#
# Talk to it over USB serial (Thonny shell, or host/pico_term.py).
# One command per line, type 'help' for the list.

import sys
import time
import select
from array import array
from machine import Pin, UART, ADC, PWM, Timer

import config as cfg
import linkproto as lp

ROLE = cfg.ROLE.upper()
HAS_TX = ROLE in ("TX", "BOTH")
HAS_RX = ROLE in ("RX", "BOTH")
FRAME_MS = lp.FRAME_LEN * 10 * 1000 // cfg.BAUD + cfg.GAP_MS


def out(*a):
    print(*a)


# ===================================================================== TX
class Laser:
    """Owns the laser pin: UART data mode, steady on/off, or square wave."""

    def __init__(self):
        self.uart = None
        self.pwm = None
        self.timer = None
        self.pin = None
        self.deadline = None
        self.mode = ""
        self.data_mode()

    def _release(self):
        if self.uart:
            try:
                self.uart.flush()
            except Exception:
                time.sleep_ms(FRAME_MS)
        if self.pwm:
            self.pwm.deinit()
            self.pwm = None
        if self.timer:
            self.timer.deinit()
            self.timer = None
        self.deadline = None

    def data_mode(self):
        if self.mode == "data":
            return
        self._release()
        # Inverted: idle line = laser OFF, start bit = laser ON.
        self.uart = UART(cfg.TX_UART, baudrate=cfg.BAUD, bits=8, parity=None,
                         stop=1, tx=Pin(cfg.LASER_PIN), txbuf=1024,
                         invert=UART.INV_TX)
        self.mode = "data"

    def steady(self, on):
        self._release()
        self.uart = None
        self.pin = Pin(cfg.LASER_PIN, Pin.OUT, value=1 if on else 0)
        self.mode = "on" if on else "off"
        if on:
            self.deadline = time.ticks_add(time.ticks_ms(), cfg.ON_TIMEOUT_S * 1000)

    def square(self, hz):
        self._release()
        self.uart = None
        if hz >= 10:
            self.pwm = PWM(Pin(cfg.LASER_PIN))
            self.pwm.freq(int(hz))
            self.pwm.duty_u16(32768)
        else:
            self.pin = Pin(cfg.LASER_PIN, Pin.OUT, value=0)
            p = self.pin
            self.timer = Timer(period=max(1, int(500 / hz)), mode=Timer.PERIODIC,
                               callback=lambda t: p.toggle())
        self.mode = "sq %g Hz" % hz
        self.deadline = time.ticks_add(time.ticks_ms(), cfg.ON_TIMEOUT_S * 1000)

    def service(self):
        if self.deadline is not None and time.ticks_diff(time.ticks_ms(), self.deadline) >= 0:
            self.steady(False)
            self.data_mode()
            out("INFO laser timeout -> off")


class Sender:
    def __init__(self, laser):
        self.laser = laser
        self.queue = []
        self.seq = 0
        self.mid = 0
        self.run = 0
        self.test_left = 0
        self.test_idx = 0
        self.test_total = 0
        self.test_t0 = 0
        self.end_left = 0
        self.repeat = cfg.REPEAT
        self.next_ok = time.ticks_ms()
        self.frames_sent = 0
        self.abc = None      # None, "raw" or "fec": repeating A..Z test
        self.abc_ms = 100
        self.abc_i = 0
        self.abc_next = 0
        self.abc_end = 0

    def _frame(self, ftype, mid, frag, payload, seq=None):
        if seq is None:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFF
        return lp.build_frame(ftype, seq, mid, frag, payload)

    def message(self, text):
        data = text.encode("utf-8")
        if len(data) > 2000:
            return "ERR message too long (max 2000 bytes)"
        parts = lp.split_message(data)
        mid = self.mid
        self.mid = (self.mid + 1) & 0xFF
        for frag, chunk in parts:
            f = self._frame(lp.T_MSG, mid, frag, chunk)
            for _ in range(self.repeat):
                self.queue.append(f)
        self.laser.data_mode()
        secs = len(self.queue) * FRAME_MS / 1000
        return "OK msg %d queued: %d bytes, %d frame(s), ~%.1f s" % (mid, len(data), len(parts) * self.repeat, secs)

    def start_test(self, n):
        self.run = (self.run + 1) & 0xFF
        self.test_left = n
        self.test_total = n
        self.test_idx = 0
        self.test_t0 = time.ticks_ms()
        self.laser.data_mode()
        return "OK test run %d: %d frames, ~%.0f s" % (self.run, n, n * FRAME_MS / 1000)

    def start_abc(self, mode, ms):
        self.abc = mode
        self.abc_ms = ms
        self.abc_i = 0
        self.abc_next = time.ticks_ms()
        self.abc_end = time.ticks_add(self.abc_next, cfg.ON_TIMEOUT_S * 1000)
        self.laser.data_mode()
        how = ("raw bytes, no error correction" if mode == "raw"
               else "one error-corrected frame per letter")
        return "OK abc %s: A..Z every %d ms (%s). 'stop' ends it; auto-stop in %d s" % (
            mode, ms, how, cfg.ON_TIMEOUT_S)

    def stop_abc(self):
        was = self.abc
        self.abc = None
        return was

    def _abc_service(self, now):
        if time.ticks_diff(now, self.abc_end) >= 0:
            self.abc = None
            out("INFO abc timeout -> stopped")
            return
        if time.ticks_diff(now, self.abc_next) < 0 or self.queue:
            return
        ch = 65 + self.abc_i % 26
        self.abc_i += 1
        self.abc_next = time.ticks_add(now, self.abc_ms)
        if self.abc == "raw":
            self.laser.uart.write(bytes([ch]))
        else:
            # Only queued when the queue is empty, so the rate tops out at
            # one letter per frame time.
            self.queue.append(self._frame(lp.T_MSG, self.mid, 0x80, bytes([ch])))
            self.mid = (self.mid + 1) & 0xFF

    def busy(self):
        return bool(self.queue or self.test_left or self.end_left or self.abc)

    def service(self):
        if self.laser.mode != "data":
            return
        now = time.ticks_ms()
        if self.abc:
            self._abc_service(now)
        if time.ticks_diff(now, self.next_ok) < 0:
            return
        f = None
        if self.queue:
            f = self.queue.pop(0)
        elif self.test_left:
            i = self.test_idx
            # test frames: seq = low byte of index, frag = high bits
            f = self._frame(lp.T_TEST, self.run, (i >> 8) & 0x7F,
                            lp.test_payload(self.run, i), seq=i & 0xFF)
            self.test_idx += 1
            self.test_left -= 1
            if not self.test_left:
                self.end_left = 3
        elif self.end_left:
            n = self.test_total
            f = self._frame(lp.T_END, self.run, 0, bytes([n >> 8, n & 0xFF]))
            self.end_left -= 1
            if not self.end_left:
                dt = time.ticks_diff(now, self.test_t0) / 1000
                out("OK test run %d sent: %d frames in %.1f s at %d bps (~%.0f bps of payload)"
                    % (self.run, self.test_total, dt, cfg.BAUD,
                       self.test_total * lp.PAYLOAD_MAX * 8 / max(dt, 0.001)))
        if f is not None:
            self.laser.uart.write(f)
            self.frames_sent += 1
            self.next_ok = time.ticks_add(now, FRAME_MS)


# ===================================================================== RX
class Receiver:
    def __init__(self):
        self.open_uart()
        self.adc = ADC(cfg.ADC_PIN)
        self.dec = lp.FrameDecoder()
        self.asm = lp.MessageAssembler()
        self.test = None
        self.mon = False
        self.mon_next = 0
        self.mon_end = 0
        self.raw = False     # print every received byte instead of decoding
        self.raw_col = 0
        self.raw_last = 0
        self.raw_quiet = True
        self.beacon_pin = Pin(cfg.BEACON_PIN, Pin.OUT, value=0)
        self.beacon_timer = None
        self.beacon(cfg.BEACON_HZ)

    def open_uart(self):
        self.uart = UART(cfg.RX_UART, baudrate=cfg.BAUD, bits=8, parity=None,
                         stop=1, rx=Pin(cfg.RX_PIN), rxbuf=2048,
                         invert=UART.INV_RX)

    # ---------- beacon LED
    def beacon(self, hz):
        if self.beacon_timer:
            self.beacon_timer.deinit()
            self.beacon_timer = None
        if hz is True:
            self.beacon_pin.value(1)
            return "OK beacon steady on"
        if not hz:
            self.beacon_pin.value(0)
            return "OK beacon off"
        p = self.beacon_pin
        self.beacon_timer = Timer(period=max(1, int(500 / hz)), mode=Timer.PERIODIC,
                                  callback=lambda t: p.toggle())
        return "OK beacon blinking at %g Hz" % hz

    # ---------- analog helpers
    def volts(self, raw):
        return raw * cfg.ADC_VREF / 65535

    def cal(self):
        """2 s of slow samples. Run with the TX on 'sq 2' to see both levels."""
        n = 2000
        buf = array("H", bytes(2 * n))
        r = self.adc.read_u16
        for i in range(n):
            buf[i] = r()
            time.sleep_us(1000)
        mn = min(buf)
        mx = max(buf)
        lines = ["CAL pin volts: min %.3f  max %.3f  (OPT101 out ~%.2f..%.2f V)"
                 % (self.volts(mn), self.volts(mx),
                    self.volts(mn) * cfg.DIVIDER, self.volts(mx) * cfg.DIVIDER)]
        if mx - mn < 6000:
            lv = self.volts(sum(buf) / n)
            lines.append("CAL one level only: %.3f V. Run 'sq 2' (or on/off) on the laser and repeat." % lv)
            lines.append("CAL dark must be <= 0.5 V, lit must be >= 2.3 V on the pin.")
            return "\n".join(lines)
        thr = (mn + mx) // 2
        lo = [v for v in buf if v < thr]
        hi = [v for v in buf if v >= thr]
        lo_mean = self.volts(sum(lo) / len(lo))
        hi_mean = self.volts(sum(hi) / len(hi))
        lines.append("CAL dark level %.3f V (%s, need <= 0.5)" % (lo_mean, "OK" if lo_mean <= 0.5 else "TOO HIGH - shade sensor / move beacon"))
        lines.append("CAL lit  level %.3f V (%s, need >= 2.3)" % (hi_mean, "OK" if hi_mean >= 2.3 else "TOO LOW - re-aim laser / check divider"))
        return "\n".join(lines)

    def scope(self, n=3000):
        """Fast capture; run with the TX on 'sq 500' (= 1000 bps 1010...)."""
        buf = array("H", bytes(2 * n))
        r = self.adc.read_u16
        t0 = time.ticks_us()
        for i in range(n):
            buf[i] = r()
        us = time.ticks_diff(time.ticks_us(), t0) / n
        mn = min(buf)
        mx = max(buf)
        res = ["SCOPE %d samples, %.1f us/sample, min %.3f V max %.3f V"
               % (n, us, self.volts(mn), self.volts(mx))]
        if mx - mn < 6000:
            res.append("SCOPE no swing - is the laser running 'sq 500'?")
            return "\n".join(res)
        thr = (mn + mx) // 2
        runs = {0: [], 1: []}
        state = 1 if buf[0] >= thr else 0
        length = 0
        first = True
        for v in buf:
            s = 1 if v >= thr else 0
            if s == state:
                length += 1
            else:
                if not first:
                    runs[state].append(length)
                first = False
                state = s
                length = 1
        for s, name in ((1, "lit "), (0, "dark")):
            rl = runs[s]
            if rl:
                res.append("SCOPE %s pulses: n=%d mean %.0f us, min %.0f, max %.0f"
                           % (name, len(rl), sum(rl) / len(rl) * us, min(rl) * us, max(rl) * us))
        stride = max(1, int(4000 / us / 80))
        res.append("SCOPE 4 ms: " + "".join("#" if buf[i] >= thr else "_" for i in range(0, min(n, stride * 80), stride)))
        bit_us = 1e6 / cfg.BAUD
        res.append("SCOPE at 'sq %g' (= %d bps) both pulse means should be %.0f us +/- %.0f"
                   % (cfg.BAUD / 2, cfg.BAUD, bit_us, bit_us / 4))
        return "\n".join(res)

    # ---------- data path
    def handle_frame(self, fr, now):
        if fr.type == lp.T_MSG:
            r = self.asm.add(fr, now)
            if r:
                self.show(r)
        elif fr.type == lp.T_TEST:
            t = self.test
            if t is None or t["run"] != fr.mid:
                t = self.test = {"run": fr.mid, "got": set(), "fix": 0,
                                 "bad": 0, "rsf": self.dec.rs_fail, "crcf": self.dec.crc_fail,
                                 "last": now, "max": 0, "done": False, "quiet": False}
            idx = ((fr.frag & 0x7F) << 8) | fr.seq
            if idx not in t["got"]:
                t["got"].add(idx)
                t["fix"] += fr.corrected
                if fr.payload != lp.test_payload(fr.mid, idx):
                    t["bad"] += 1
            t["max"] = max(t["max"], idx)
            t["last"] = now
            t["quiet"] = False
        elif fr.type == lp.T_END:
            t = self.test
            if t and t["run"] == fr.mid and not t["done"] and len(fr.payload) == 2:
                self.report((fr.payload[0] << 8) | fr.payload[1])

    def report(self, sent=None):
        # sent=None: interim result after a gap; the test keeps counting and
        # the final report still prints when the end marker arrives.
        t = self.test
        final = sent is not None
        t["done"] = final
        got = len(t["got"])
        if sent is None:
            sent = t["max"] + 1
        lost = sent - got
        nbytes = max(1, got * lp.N)
        out("TEST run %d: sent %d, received %d, lost %d (%.2f%%)"
            % (t["run"], sent, got, lost, 100 * lost / max(1, sent)))
        out("TEST bytes repaired by FEC: %d of %d (raw byte error rate %.2e)"
            % (t["fix"], nbytes, t["fix"] / nbytes))
        out("TEST payload errors after FEC: %d   undecodable frames: %d"
            % (t["bad"], self.dec.rs_fail - t["rsf"] + self.dec.crc_fail - t["crcf"]))
        verdict = "PASS" if lost == 0 and t["bad"] == 0 else "FAIL"
        out("TEST verdict: %s" % verdict if final else
            "TEST so far: %s (still listening for the rest of the test)" % verdict)

    def show(self, r):
        mid, text, partial = r
        if partial:
            out("MSG %d (INCOMPLETE): %s" % (mid, text))
        else:
            out("MSG %d: %s" % (mid, text))

    def set_raw(self, on):
        self.raw = on
        self.raw_col = 0
        self.raw_quiet = True
        if on:
            self.uart.read()  # drop anything already buffered
            return "OK raw display ON: received bytes print as they arrive ('raw off' to stop)"
        return "OK raw display OFF"

    def show_raw(self, data, now):
        # Letters print as themselves, anything else as {hex}. New line after Z.
        self.raw_last = now
        self.raw_quiet = False
        s = []
        for b in data:
            if 32 <= b < 127:
                s.append(chr(b))
                self.raw_col += 1
            else:
                s.append("{%02X}" % b)
                self.raw_col += 4
            if b == 90 or self.raw_col >= 64:
                s.append("\n")
                self.raw_col = 0
        sys.stdout.write("".join(s))

    def service(self):
        now = time.ticks_ms()
        n = self.uart.any()
        if n:
            data = self.uart.read(n)
            if self.raw:
                self.show_raw(data, now)
            else:
                for fr in self.dec.feed(data):
                    self.handle_frame(fr, now)
        if self.raw and not self.raw_quiet and time.ticks_diff(now, self.raw_last) > 1000:
            self.raw_quiet = True
            if self.raw_col:
                sys.stdout.write("\n")
                self.raw_col = 0
            out("RAW (no signal)")
        for r in self.asm.service(now):
            self.show(r)
        t = self.test
        if t and not t["done"] and not t["quiet"] and time.ticks_diff(now, t["last"]) > 3000:
            t["quiet"] = True
            out("TEST no frames for 3 s (beam off the sensor?). Interim count:")
            self.report()
        if self.mon and time.ticks_diff(now, self.mon_end) >= 0:
            self.mon = False
            out("OK mon stopped (20 s limit; type mon again to continue)")
        if self.mon and time.ticks_diff(now, self.mon_next) >= 0:
            self.mon_next = time.ticks_add(now, 100)
            v = self.volts(self.adc.read_u16())
            out("MON %.3f V |%s" % (v, "#" * int(v / cfg.ADC_VREF * 50)))

    def stat(self):
        d = self.dec
        return ("STAT rx frames ok %d, bytes repaired %d, undecodable %d (rs) %d (crc), junk bytes %d"
                % (d.ok, d.corrected, d.rs_fail, d.crc_fail, d.junk))


# ===================================================================== commands
HELP_TX = """TX commands:
  msg <text>   send a text message
  on / off     steady laser (alignment); 'on' times out after %d s
  sq <hz>      square wave (sq 2 for 'cal', sq 500 = 1000 bps for 'scope')
  data         back to data mode (laser idle off)
  test <n>     send n test frames (default 200) and get a PASS/FAIL on RX
  rep <1|2>    send each message frame 1 or 2 times
  abc [ms]     repeat A..Z as raw bytes (default every 100 ms); RX needs 'raw'
  abc fec [ms] repeat A..Z, one error-corrected frame per letter
  stop         stop abc
  mot <args>   pass to motors.py""" % cfg.ON_TIMEOUT_S
HELP_RX = """RX commands:
  cal          2 s level check (run with laser 'sq 2')
  scope        fast capture + pulse widths (run with laser 'sq 500')
  mon          print sensor volts 10x/s for 20 s; any key stops it
  raw [on|off] print every received byte as it arrives (for 'abc')
  beacon <hz|on|off>
  stat / reset decoder counters"""
HELP_ALL = """All: help, ping, stat, role
  baud <bps>   change the line rate until reboot (set it on BOTH ends)"""


def main():
    out("READY role=%s baud=%d frame=%d ms (type 'help')" % (ROLE, cfg.BAUD, FRAME_MS))
    laser = Laser() if HAS_TX else None
    tx = Sender(laser) if HAS_TX else None
    rx = Receiver() if HAS_RX else None
    motors = None
    if HAS_TX:
        try:
            import motors
        except ImportError:
            motors = None

    def set_baud(b):
        global FRAME_MS
        cfg.BAUD = b
        FRAME_MS = lp.FRAME_LEN * 10 * 1000 // b + cfg.GAP_MS
        if tx:
            tx.stop_abc()
            if laser.mode == "data":
                laser.mode = ""      # force data_mode() to reopen the UART
                laser.data_mode()
        if rx:
            rx.open_uart()
            rx.dec = lp.FrameDecoder()
            rx.test = None

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    linebuf = []

    def handle(line):
        cmd, _, arg = line.strip().partition(" ")
        c = cmd.lower()
        arg = arg.strip()
        if rx and rx.mon:
            rx.mon = False
            out("OK mon stopped")
            if not c:
                return
        if c == "":
            return
        if tx and tx.abc and c in ("on", "off", "sq", "data", "msg", "test"):
            tx.stop_abc()
            out("OK abc stopped")
            if rx and rx.raw:
                out(rx.set_raw(False))
        if c == "help":
            out(HELP_ALL)
            if tx:
                out(HELP_TX)
            if rx:
                out(HELP_RX)
        elif c == "ping":
            out("PONG")
        elif c == "role":
            out("OK role=%s" % ROLE)
        elif c == "baud":
            if not arg:
                out("OK baud=%d frame=%d ms" % (cfg.BAUD, FRAME_MS))
                return
            b = int(arg)
            if not 300 <= b <= 20000:
                out("ERR baud range 300..20000")
                return
            set_baud(b)
            out("OK baud=%d frame=%d ms (until reboot; set the other board too)"
                % (cfg.BAUD, FRAME_MS))
        elif c == "stat":
            if tx:
                out("STAT tx frames sent %d, queued %d, laser mode %s, repeat %d"
                    % (tx.frames_sent, len(tx.queue), laser.mode, tx.repeat))
            if rx:
                out(rx.stat())
        elif tx and c == "msg":
            out(tx.message(arg))
        elif tx and c == "on":
            laser.steady(True)
            out("OK laser ON (auto-off in %d s)" % cfg.ON_TIMEOUT_S)
        elif tx and c == "off":
            laser.steady(False)
            out("OK laser OFF")
        elif tx and c == "data":
            laser.data_mode()
            out("OK data mode")
        elif tx and c == "sq":
            hz = float(arg or "500")
            if not 0.5 <= hz <= 5000:
                out("ERR sq range 0.5..5000 Hz")
            else:
                laser.square(hz)
                out("OK square wave %g Hz (= %g bps alternating)" % (hz, 2 * hz))
        elif tx and c == "test":
            out(tx.start_test(int(arg or "200")))
        elif tx and c == "rep":
            tx.repeat = max(1, min(3, int(arg or "1")))
            out("OK repeat %d" % tx.repeat)
        elif tx and c == "abc":
            parts = arg.lower().split()
            mode = "fec" if "fec" in parts else "raw"
            nums = [p for p in parts if p != "fec"]
            ms = int(nums[0]) if nums else (FRAME_MS if mode == "fec" else 100)
            if not 15 <= ms <= 10000:
                out("ERR abc interval 15..10000 ms")
            else:
                out(tx.start_abc(mode, ms))
                if rx:  # BOTH board: show what comes back the matching way
                    out(rx.set_raw(mode == "raw"))
        elif tx and c == "stop":
            out("OK abc stopped" if tx.stop_abc() else "OK abc was not running")
            if rx and rx.raw:
                out(rx.set_raw(False))
        elif rx and c == "raw":
            a = arg.lower()
            out(rx.set_raw(not rx.raw if a == "" else a in ("on", "1")))
        elif tx and c == "mot":
            out(motors.handle(arg) if motors else "ERR no motors.py")
        elif rx and c == "cal":
            out("OK measuring 2 s ...")
            out(rx.cal())
        elif rx and c == "scope":
            out(rx.scope())
        elif rx and c == "mon":
            rx.mon = True
            rx.mon_end = time.ticks_add(time.ticks_ms(), 20000)
            out("OK monitoring for 20 s; press any key to stop early")
        elif rx and c == "beacon":
            a = arg.lower()
            out(rx.beacon(True if a == "on" else (0 if a in ("", "off", "0") else float(a))))
        elif rx and c == "reset":
            rx.dec = lp.FrameDecoder()
            rx.test = None
            out("OK counters reset")
        else:
            out("ERR unknown command '%s' (try help)" % cmd)

    while True:
        while poll.poll(0):
            ch = sys.stdin.read(1)
            if rx and rx.mon:
                # any key stops the monitor (and is otherwise ignored)
                rx.mon = False
                out("OK mon stopped")
                del linebuf[:]
                continue
            if ch in ("\n", "\r"):
                if linebuf:
                    line = "".join(linebuf)
                    del linebuf[:]
                    try:
                        handle(line)
                    except Exception as e:
                        out("ERR %r" % e)
            elif len(linebuf) < 2100:
                linebuf.append(ch)
        try:
            if laser:
                laser.service()
                tx.service()
            if motors:
                motors.service()
            if rx:
                rx.service()
        except Exception as e:
            out("ERR loop %r" % e)
        time.sleep_ms(1)


main()