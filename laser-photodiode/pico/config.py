# config.py - edit this one file per board.

# "BOTH" = one Pico drives the laser AND reads the OPT101 (bench loopback).
# "TX"   = laser Pico (plugged into the Pi 5).
# "RX"   = photodiode Pico (plugged into the receiving laptop).
ROLE = "TX"

BAUD = 1000          # line rate in bits/s (8N1 -> 100 bytes/s raw)
GAP_MS = 20          # idle laser-off gap between frames (lets UART resync)
REPEAT = 1           # send every message frame this many times (1 or 2)

# Transmitter
TX_UART = 0
LASER_PIN = 16       # GP16 -> 1k -> laser yellow wire (10k pull-down to GND)
ON_TIMEOUT_S = 300   # 'on' / 'sq' modes switch the laser off after this

# Receiver
RX_UART = 1
RX_PIN = 5           # GP5 <- divider output (same node as GP26)
ADC_PIN = 26         # GP26 <- divider output, for cal/scope/mon
ADC_VREF = 3.3
DIVIDER = 1.5        # OPT101 volts = pin volts * 1.5 (10k top, 20k bottom)

# Optional beacon LED for the gimbal camera (driven by the RX Pico)
BEACON_PIN = 15
BEACON_HZ = 0        # 0 = off at boot; set e.g. 4 for a 4 Hz blink
