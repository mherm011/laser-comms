#!/usr/bin/env python3
"""
beacon_test.py - find a blinking blue light with the camera and measure how
steady the detected position is. Works on Windows and on Linux (Pi 5).

Detection is a per-pixel lock-in at the blink frequency: each pixel's
"blueness" (blue minus the larger of red and green) is high-passed, multiplied
by a sine and cosine at the blink rate, and low-passed. Only things that are
blue AND blink at that rate light up, so steady blue objects, screens and
room-light flicker drop out. It doesn't need to be synced to the light.

Windows (unless --headless):
  camera           scaled live view, yellow circle = detected beacon
  blue only        everything black except blue pixels
  blink detector   lock-in strength map (bright = blinking blue)
Keys: q or Esc quit, s save a snapshot.

Windows examples:
  py beacon_test.py                  # camera 0, defaults
  py beacon_test.py --settings       # also open the camera's settings dialog
  py beacon_test.py --exposure -7    # manual exposure, about 1/128 s
  py beacon_test.py --device 1       # if the Kiyo isn't camera 0

Pi / Linux examples:
  python3 beacon_test.py --exposure 20 --gain 0
  python3 beacon_test.py --headless --range 25
"""
import argparse
import collections
import math
import platform
import subprocess
import time

import cv2
import numpy as np

IS_WIN = platform.system() == "Windows"


def v4l2_set(dev, ctrl):
    try:
        r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", ctrl],
                           capture_output=True, text=True)
    except FileNotFoundError:
        print("[warn] v4l2-ctl not found; install v4l-utils")
        return
    if r.returncode != 0:
        print(f"[warn] could not set {ctrl}: {r.stderr.strip() or 'failed'}")


def lock_camera_linux(dev, exposure, gain):
    # Control names differ between kernels. Failures are reported, not fatal.
    # Run `v4l2-ctl -d /dev/video0 --list-ctrls` to see what yours are called.
    for ctrl in ("auto_exposure=1",
                 f"exposure_time_absolute={int(exposure)}",
                 f"gain={gain}",
                 "focus_automatic_continuous=0",
                 "white_balance_automatic=0"):
        v4l2_set(dev, ctrl)


def lock_camera_windows(cap, exposure, gain):
    # DirectShow: 0.25 = manual exposure, 0.75 = auto. Exposure is log2 of
    # seconds: -5 = 1/32 s, -7 = 1/128 s, -9 = 1/512 s. Not every driver
    # honours every property; --settings is the reliable fallback.
    for prop, val, name in ((cv2.CAP_PROP_AUTO_EXPOSURE, 0.25, "manual exposure"),
                            (cv2.CAP_PROP_EXPOSURE, exposure, "exposure"),
                            (cv2.CAP_PROP_GAIN, gain, "gain"),
                            (cv2.CAP_PROP_AUTOFOCUS, 0, "autofocus off"),
                            (cv2.CAP_PROP_AUTO_WB, 0, "auto white balance off")):
        if not cap.set(prop, val):
            print(f"[warn] camera ignored {name}; use --settings instead")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="0" if IS_WIN else "/dev/video0",
                    help="camera index (Windows) or device path (Linux)")
    ap.add_argument("--freq", type=float, default=10.0,
                    help="blink frequency in Hz (default 10)")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--mjpeg", action="store_true",
                    help="request MJPEG instead of uncompressed")
    ap.add_argument("--hfov", type=float, default=80.0,
                    help="horizontal field of view in degrees (80, 90 or 103)")
    ap.add_argument("--range", type=float, default=25.0,
                    help="distance to the light in metres, for cm figures")
    ap.add_argument("--exposure", type=float, default=None,
                    help="manual exposure (Windows: log2 seconds, e.g. -7; "
                         "Linux: v4l2 units, e.g. 20). Omit to leave as is")
    ap.add_argument("--gain", type=int, default=0)
    ap.add_argument("--settings", action="store_true",
                    help="Windows: open the camera's own settings dialog")
    ap.add_argument("--blue-thresh", type=int, default=40,
                    help="threshold for the 'blue only' view")
    ap.add_argument("--scale", type=float, default=0.5,
                    help="processing scale; try 0.25 if fps is low")
    ap.add_argument("--tau", type=float, default=0.5,
                    help="lock-in time constant in seconds")
    ap.add_argument("--min-signal", type=float, default=3.0,
                    help="minimum lock-in strength to count as a detection")
    ap.add_argument("--min-contrast", type=float, default=4.0,
                    help="minimum peak / background ratio to count as a detection")
    ap.add_argument("--headless", action="store_true",
                    help="no windows, print status only")
    a = ap.parse_args()

    dev = int(a.device) if a.device.isdigit() else a.device
    if IS_WIN:
        cap = cv2.VideoCapture(dev, cv2.CAP_DSHOW)
        raw_fmt = "YUY2"
    else:
        if a.exposure is not None:
            lock_camera_linux(a.device, a.exposure, a.gain)
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        raw_fmt = "YUYV"
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {a.device}. "
                         "Close other apps using it (Synapse preview, Teams, "
                         "OBS) or try --device 1")

    cap.set(cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*("MJPG" if a.mjpeg else raw_fmt)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    if IS_WIN:
        if a.exposure is not None:
            lock_camera_windows(cap, a.exposure, a.gain)
        if a.settings:
            cap.set(cv2.CAP_PROP_SETTINGS, 1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    code = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
    print(f"camera: {w}x{h} {fourcc!r}, requested {a.fps} fps")
    if (w, h) != (a.width, a.height):
        print("[warn] camera gave a different size; on Windows try --mjpeg "
              "or a USB 3 port")

    sw, sh = max(1, int(w * a.scale)), max(1, int(h * a.scale))
    mean = np.zeros((sh, sw), np.float32)
    I = np.zeros_like(mean)
    Q = np.zeros_like(mean)
    omega = 2.0 * math.pi * a.freq
    mrad_per_px = math.radians(a.hfov) * 1000.0 / w
    budget_px = 250.0 / a.range / mrad_per_px
    print(f"{mrad_per_px:.3f} mrad/px; 25 cm at {a.range:g} m = {budget_px:.1f} px")

    positions = collections.deque(maxlen=120)
    stamps = collections.deque(maxlen=120)
    t0 = prev = warmup_until = None
    # V4L2 gives real capture timestamps; DirectShow usually doesn't, so on
    # Windows use perf_counter (time.monotonic is ~16 ms coarse there).
    use_cam_ts = None if not IS_WIN else False
    last_print = 0.0
    status = "starting"
    show = not a.headless

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("frame grab failed")
                break
            now = time.perf_counter()
            if use_cam_ts is None:
                use_cam_ts = cap.get(cv2.CAP_PROP_POS_MSEC) > 0
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0 if use_cam_ts else now
            stamps.append(now)

            small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_AREA)
            b, g, r = cv2.split(small.astype(np.int16))
            blue = np.clip(b - np.maximum(g, r), 0, 255).astype(np.float32)

            if t0 is None:
                t0 = prev = t
                warmup_until = t + 3.0 * a.tau
                mean[:] = blue
            dt = max(t - prev, 0.0)
            prev = t

            # High-pass (remove each pixel's slow average), then lock-in.
            km = 1.0 - math.exp(-dt / (2.0 * a.tau))
            k = 1.0 - math.exp(-dt / a.tau)
            mean += km * (blue - mean)
            ac = blue - mean
            ph = omega * (t - t0)
            I += k * (ac * math.cos(ph) - I)
            Q += k * (ac * math.sin(ph) - Q)
            mag = cv2.magnitude(I, Q)

            _, peak, _, (px, py) = cv2.minMaxLoc(mag)
            floor = float(np.percentile(mag[::4, ::4], 99.5)) + 1e-3
            contrast = peak / floor
            found = (t >= warmup_until and peak >= a.min_signal
                     and contrast >= a.min_contrast)

            cx = cy = sx = sy = None
            if found:
                rad = 7
                x0, x1 = max(px - rad, 0), min(px + rad + 1, sw)
                y0, y1 = max(py - rad, 0), min(py + rad + 1, sh)
                win = mag[y0:y1, x0:x1]
                wts = np.where(win >= 0.5 * peak, win, 0.0)
                ys, xs = np.mgrid[y0:y1, x0:x1]
                s = float(wts.sum())
                sx = float((wts * xs).sum()) / s
                sy = float((wts * ys).sum()) / s
                cx = (sx + 0.5) / a.scale - 0.5  # full-resolution pixels
                cy = (sy + 0.5) / a.scale - 0.5
                positions.append((cx, cy))
            else:
                positions.clear()

            if now - last_print >= 1.0:
                last_print = now
                span = stamps[-1] - stamps[0]
                fps = (len(stamps) - 1) / span if span > 0 else 0.0
                status = f"fps {fps:5.1f} | "
                if found:
                    status += (f"beacon ({cx:7.1f}, {cy:7.1f}) px "
                               f"contrast {contrast:5.1f}")
                    if len(positions) >= 30:
                        jx, jy = np.array(positions).std(axis=0)
                        j = math.hypot(jx, jy)
                        status += (f" jitter {j:.2f} px = "
                                   f"{j * mrad_per_px:.2f} mrad = "
                                   f"{j * mrad_per_px * a.range / 10:.1f} cm "
                                   f"@ {a.range:g} m")
                elif t < warmup_until:
                    status += "warming up"
                else:
                    status += (f"no beacon (peak {peak:.1f}, "
                               f"contrast {contrast:.1f})")
                if len(stamps) > 60 and fps < 0.85 * a.fps:
                    status += " [slow: try --scale 0.25]"
                print(status, flush=True)

            if show:
                disp = small.copy()
                cv2.drawMarker(disp, (sw // 2, sh // 2), (128, 128, 128),
                               cv2.MARKER_CROSS, 20, 1)
                if found:
                    c = (int(round(sx)), int(round(sy)))
                    cv2.circle(disp, c, 12, (0, 255, 255), 2)
                    cv2.drawMarker(disp, c, (0, 255, 255),
                                   cv2.MARKER_CROSS, 8, 1)
                cv2.putText(disp, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
                blue_only = np.where((blue >= a.blue_thresh)[..., None],
                                     small, 0).astype(np.uint8)
                heat = cv2.applyColorMap(
                    cv2.convertScaleAbs(mag, alpha=255.0 / max(peak, 1e-3)),
                    cv2.COLORMAP_INFERNO)
                cv2.imshow("camera", disp)
                cv2.imshow("blue only", blue_only)
                cv2.imshow("blink detector", heat)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key == ord("s"):
                    tag = time.strftime("%Y%m%d-%H%M%S")
                    cv2.imwrite(f"snap-{tag}-frame.png", frame)
                    cv2.imwrite(f"snap-{tag}-blue.png", blue_only)
                    cv2.imwrite(f"snap-{tag}-blink.png", heat)
                    print(f"saved snap-{tag}-.png")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()