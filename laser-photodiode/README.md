# Laser link: bring-up guide

Goal: type on a laptop plugged into the Pi 5, the text goes over the laser at 1000 bps with error correction, and a second laptop prints it.

```
Laptop A ──ssh──► Pi 5 ──USB──► TX Pico ──GP16──► laser ~~~~~~► OPT101 ──► RX Pico ──USB──► Laptop B
                  (vision)      (laser + motors)                (+ beacon LED)
```

## Folder contents

| File | Goes on | What it does |
|---|---|---|
| `pico/main.py` | both Picos | Firmware. Role is set in `config.py`. |
| `pico/linkproto.py` | both Picos | Framing, Reed-Solomon error correction, message reassembly |
| `pico/config.py` | both Picos | `ROLE = "BOTH"`, `"TX"` or `"RX"`, plus pins and baud |
| `pico/motors.py` | TX Pico | Empty hook for the gimbal team (`MOT ...` commands) |
| `host/pico_term.py` | Pi 5 and laptop B | Terminal: typed lines become laser messages, received messages print |
| `tests/sim_test.py` | your PC | Desktop test of the error correction with a simulated noisy link |

## Design decisions

**The laser is driven by the Pico's hardware UART, not by Python timing.** Each byte goes out as a start bit, 8 data bits and a stop bit at exactly 1000 bps. The bit timing is done in silicon, so Python delays, garbage collection or motor code can't corrupt bits; they can only lengthen the gap between frames. The output is inverted, so an idle link means the laser is **off**, and every byte begins with a laser-on start bit.

**The receiver also uses the hardware UART.** The OPT101 output passes through a resistor divider into GP5 (UART RX) and GP26 (ADC, used for the calibration tools). The UART resynchronises on every byte, so clock drift between the two Picos doesn't matter.

**Error correction: Reed-Solomon (32,24).** Each frame is 2 sync bytes followed by a 32-byte codeword: 5 header bytes, up to 17 text bytes, a CRC-16 and 8 parity bytes. Any 4 corrupted bytes in a frame are repaired, which covers a 40 ms dropout such as a hand passing through the beam. The CRC stops a badly damaged frame from being printed as wrong text. The link is one-way, so lost frames can't be resent on request. `rep 2` sends every frame twice if you want extra safety.

**Throughput.** The line rate is 1000 bps. After framing, FEC and the 20 ms gap between frames, it carries about **380 bps of text (≈47 characters/s)**. A frame takes 360 ms. If you need 1000 bps of *payload*, set `BAUD = 2400` on both boards. The OPT101 is fast enough for that, and `scope` will show whether it is.

**Manchester encoding: not needed.** Manchester solves two problems: recovering the clock over long runs of identical bits, and keeping the signal DC-balanced for AC-coupled receivers. UART framing already resyncs the clock every 10 bits, and the OPT101 is DC-coupled with a fixed threshold. Manchester would only matter if you later add an adaptive or AC-coupled threshold, for example to fight sunlight at long range.

**Motors: on the TX Pico (recommended).** The Pi 5 runs Linux, which can't produce jitter-free step or servo pulses. The Pico can, with PWM and PIO in hardware. Because the laser is on the hardware UART, motor code can't disturb the data. The Pi 5 sends targets (`MOT <pan> <tilt>`) at camera frame rate over the same USB link, and the gimbal person writes `motors.py`. The one exception: if the motor boards are "smart" (they take serial or I²C commands and run their own loops), the Pico adds nothing, and the Pi 5 can talk to them directly.

## Hardware change from the last session: OPT101 now runs from 5 V

At 3.3 V the OPT101 tops out around 2.15 V. That is barely above the Pico's 2.0 V logic-high threshold, so the UART input would be unreliable. At 5 V (VBUS) it tops out around 3.85 V, and a 5 k / 10 k divider scales that to about 2.6 V. That is safely under 3.3 V and well above 2.0 V.

**Pico 2 W note:** the RP2350 chip (in early "A2" silicon) has a GPIO input bug, erratum E9. A digital input pin can get stuck near 2.1 V unless whatever drives it has low impedance (under about 8 kΩ). The divider uses 5 k / 10 k rather than 10 k / 20 k for that reason: same ratio, half the impedance (about 3.3 kΩ). If `mon` shows the dark level stuck around 2 V on the pin after the laser has been on, this bug is the cause.

**Never connect the OPT101 output directly to a Pico pin while it runs from 5 V.** Always go through the divider.

### TX wiring (laser)

| Pico pin | Connects to |
|---|---|
| 40 VBUS | laser red |
| 38 GND | laser black |
| 21 GP16 | 1 kΩ → laser yellow; 10 kΩ from yellow to GND |

Don't wire GP1. The UART reserves it.

### RX wiring (OPT101, DIP-8). Check pin numbers against the datasheet's basic-connection figure.

| OPT101 pin | Connects to |
|---|---|
| 1 (V+) | Pico pin 40 VBUS, plus 0.1 µF to GND |
| 3 (−V) and 8 (Common) | GND |
| 4 (1 MΩ feedback) | jumper to pin 5 |
| 5 (Output) | 5 kΩ (two 10 kΩ **in parallel**) → **node X** |
| node X | 10 kΩ → GND |
| node X | Pico pin 31 GP26 **and** pin 7 GP5 |

Optional beacon LED: Pico pin 20 GP15. A blue LED on 3.3 V is dim, so drive it from VBUS through a transistor if the camera needs it brighter. Don't wire GP4.

For the one-Pico bench test (ROLE `BOTH`), wire both tables to the same Pico.

## Bring-up, stage by stage

Do each stage in order, and don't move on until it passes. Wire only with USB unplugged.

### Stage 0: flash the Pico (10 min)
1. Hold BOOTSEL, plug the Pico in, and open Thonny. Go to Tools → Options → Interpreter, choose "MicroPython (Raspberry Pi Pico)", then click "Install or update MicroPython" and pick the **Pico 2 W** variant (or drag the RPI_PICO2_W `.uf2` from micropython.org/download onto the RP2350 drive). Unplug and replug the Pico.
2. Open each file in `pico/` and use File → Save as… → Raspberry Pi Pico, keeping the same name. Save `main.py` last. Leave `ROLE = "BOTH"`.
3. Press Stop/Restart, then Run (F5). The Shell should print `READY role=BOTH baud=1000 frame=360 ms`. You type commands in the Shell, and `help` lists them.

### Stage 1: laser on 3.3 V drive
Wire the TX table only.
- `on` lights the laser and `off` turns it off. Measure the yellow wire while it's on. **You want ≥ 2.4 V and full brightness.** If it's dim, flickers or doesn't light, change the 1 kΩ to 330 Ω.
- `sq 2` blinks the laser twice per second. `data` returns it to data mode with the laser idle off.

### Stage 2: OPT101 alone
Wire the RX table. `mon` prints the pin voltage 10 times per second. Press Enter to stop it.
- Covered: about 0.00–0.02 V.
- Room light: somewhere in between.
- Flashlight right on the sensor: about 2.4–2.6 V, and **never above 2.8 V** (if it is, recheck the divider).

### Stage 3: optical loopback on one Pico (the key test)
Clamp the laser and the OPT101 facing each other a short distance apart, with the black tube on the sensor.
1. **Levels:** run `sq 2`, then `cal`. You want **dark ≤ 0.5 V and lit ≥ 2.3 V**, and both lines should say OK.
2. **Edges:** run `sq 500` (that's a 1000 bps 1010… pattern), then `scope`. **The lit and dark pulse means should each be 1000 µs ± 250.** If the lit pulses are much longer than the dark ones, the OPT101 is recovering slowly from heavy saturation. Aim the beam slightly off-centre, add a layer of paper or tape over the tube, or move the parts further apart, then run `scope` again.
3. **Data:** run `data`, then `test 200` (about 72 s). The RX side prints `TEST verdict: PASS` with 0 lost frames and 0 payload errors. "Bytes repaired" should be 0 or close to it on a good link.
4. `msg hello world` should print `MSG 0: hello world`.
5. **Prove it's consistent:** run `test 1000` (about 6 min) a few times: with room lights on and off, while waving a flashlight near the tube, and with the beacon blinking (`beacon 4`). Each run should PASS.
6. *Optional margin check:* set `BAUD = 2000` in `config.py`, then rerun `scope` (with `sq 1000`) and `test 200`. If 2000 bps passes, 1000 bps has real margin. Set it back to 1000 afterwards.

### Stage 4: split onto two Picos (you'll need a second Pico)
- Copy the same four files to both boards. Set `ROLE = "TX"` on the laser board and `ROLE = "RX"` on the OPT101 board.
- Repeat Stage 3 across the two boards. Type `sq 2`, `sq 500` and `test 200` on the TX board, and `cal`, `scope` and read the results on the RX board. Use two Thonny windows, or one Thonny window plus `pico_term.py`.

### Stage 5: Pi 5 and the two laptops
1. **Pi 5:** run `sudo apt install python3-serial`, copy `host/pico_term.py` over, and plug in the TX Pico.
2. **Laptop A → Pi 5:** connect them with an Ethernet cable and run `ssh <user>@<pi-hostname>.local`. The ssh client is built into Windows, so no admin rights are needed. If `.local` doesn't resolve over a direct cable, put both on the same Wi-Fi network, or give the Pi a static IP.
3. On the Pi, run `tmux`, then `python3 pico_term.py`. Type a line and press Enter, and the laser sends it. `/test 500` and `/stat` also work from here.
4. **Laptop B:** plug in the RX Pico. Either open Thonny and Run `main.py` (messages appear in the Shell), or run `python -m pip install --user pyserial` and then `python pico_term.py --rx`.
5. The vision program can send gimbal commands while the terminal is open: `from pico_term import hub_send; hub_send("MOT 10 -2")`.

### Stage 6: down the hall
At 20–30 m the laser spot grows to a few centimetres, but enough light still lands on the sensor to saturate it, so signal strength won't be the problem. Background light is the thing to watch. Rerun `cal` at the real distance with the hallway lights on. If the dark level climbs toward 0.5 V, lengthen the tube or narrow its opening.

Then `test 1000` should PASS end to end before you show the demo.

## Things to watch
- **Eye safety:** the datasheet allows up to 10 mW. Keep the beam below eye height, put a stop behind the receiver, and warn people in the hall. In data mode the laser is off when idle, and `on` and `sq` turn themselves off after 5 minutes.
- **The beacon LED sits right on top of the sensor.** If its light reaches the photodiode, it raises the dark level. Run `cal` with the beacon blinking to check.
- **Gimbal pointing jitter** shows up as burst errors. Up to 40 ms of beam loss per frame is repaired automatically. Use `rep 2` if the tracking is wobbly.
- **Thonny stops `main.py` when it connects.** `pico_term.py` detects this and soft-reboots the Pico.

## What's tested and what isn't
- Error correction: 3000 random codewords with 0–4 byte errors all repaired, and no wrong frames got through in 3000 overloaded trials. It was run under both CPython and real MicroPython (the WebAssembly build). Run `python3 tests/sim_test.py` to repeat it.
- Firmware logic: run against mocked Pico hardware with a 1 % byte-error loopback, a 300-frame test passed and a 5-frame message reassembled correctly. All Pico files compile with `mpy-cross`.
- `pico_term.py` was tested against a simulated Pico.
- **Not yet run on real hardware.** The Stage 1–3 numbers above are the check for that.
