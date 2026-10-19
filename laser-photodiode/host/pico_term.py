#!/usr/bin/env python3
"""pico_term.py - talk to a laser-link Pico over USB.

On the Pi 5 (transmitter):   python3 pico_term.py
    Anything you type is sent as a laser message.
    Lines starting with '/' are raw Pico commands: /test 500, /stat, /on, /help

On the receiving laptop:     python3 pico_term.py --rx
    Received messages print as they arrive. Type Pico commands
    (cal, scope, stat, beacon 4 ...) with or without the '/'.

Other programs on the Pi 5 (e.g. the gimbal/vision code) can share the same
Pico while this terminal runs: it listens on 127.0.0.1:7777 and forwards each
line it receives there to the Pico, e.g.

    from pico_term import hub_send
    hub_send("MOT 12.5 -3.0")

Or, if the vision program should own the Pico itself:

    from pico_term import PicoLink
    link = PicoLink()
    link.send_message("hello")
    link.command("MOT 12.5 -3.0")

Needs pyserial:  Pi OS: sudo apt install python3-serial
                 laptop: python -m pip install --user pyserial
"""
import argparse
import socket
import socketserver
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing: 'sudo apt install python3-serial' (Pi) or "
             "'python -m pip install --user pyserial' (laptop)")

PICO_VID = 0x2E8A
HUB_PORT = 7777


def find_pico_ports():
    return [p.device for p in list_ports.comports() if p.vid == PICO_VID]


class PicoLink:
    def __init__(self, port=None, on_line=None):
        if port is None:
            ports = find_pico_ports()
            if not ports:
                raise RuntimeError("No Pico found on USB (is it plugged in and running MicroPython?)")
            if len(ports) > 1:
                print("Several Picos found %s, using %s (use --port to choose)" % (ports, ports[0]))
            port = ports[0]
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.on_line = on_line or (lambda s: print(s, flush=True))
        self._wlock = threading.Lock()
        self._pong = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()
        self._ensure_running()

    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.on_line("!! serial error: %s" % e)
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                if line.strip() == "PONG":
                    self._pong.set()
                if line.strip() and not line.startswith(">>>"):
                    self.on_line(line)

    def _ensure_running(self):
        """If the Pico is sitting at the REPL (e.g. Thonny stopped it), restart main.py."""
        for attempt in range(2):
            self._pong.clear()
            self.command("ping")
            if self._pong.wait(1.5):
                return
            if attempt == 0:
                print("(Pico not answering - soft-rebooting it to start main.py)")
                with self._wlock:
                    self.ser.write(b"\r\x03\x03\x04")
                time.sleep(2.5)
        print("!! Pico still not answering. Is main.py + linkproto.py + config.py on it?")

    def command(self, line):
        line = line.replace("\r", " ").replace("\n", " ").strip()
        with self._wlock:
            self.ser.write((line + "\n").encode("utf-8"))

    def send_message(self, text):
        self.command("msg " + text)


def hub_send(line, port=HUB_PORT):
    """Send one command line to a running pico_term (from another program)."""
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        s.sendall((line.strip() + "\n").encode("utf-8"))


def start_hub(link, port):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            for raw in self.rfile:
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    link.command(line)

    class Server(socketserver.ThreadingMixIn, socketserver.TCPServer):
        allow_reuse_address = True
        daemon_threads = True

    try:
        srv = Server(("127.0.0.1", port), Handler)
    except OSError as e:
        print("(hub port %d unavailable: %s)" % (port, e))
        return
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print("(other programs can send Pico commands via 127.0.0.1:%d)" % port)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rx", action="store_true", help="receiver mode: typed lines are Pico commands")
    ap.add_argument("--port", help="serial port (default: auto-detect Pico)")
    ap.add_argument("--hub-port", type=int, default=HUB_PORT, help="local TCP port for other programs (0 = off)")
    args = ap.parse_args()

    def show(line):
        if line.startswith("MSG "):
            print("\n\033[1m" + line + "\033[0m", flush=True)
        else:
            print(line, flush=True)

    link = PicoLink(args.port, on_line=show)
    print("Connected to %s. %s Ctrl-C to quit."
          % (link.port, "Type Pico commands (help)." if args.rx else
             "Type a message and press Enter. '/help' for commands."))
    if not args.rx and args.hub_port:
        start_hub(link, args.hub_port)
    try:
        while True:
            line = input()
            if not line.strip():
                continue
            if args.rx or line.startswith("/"):
                link.command(line.lstrip("/"))
            else:
                link.send_message(line)
    except (KeyboardInterrupt, EOFError):
        print()


if __name__ == "__main__":
    main()
