# MINI UMWAMBI — Vision-to-FCC Error Correction

A companion-computer prototype that turns **what a gimbal camera sees** into **steering-error angles a flight controller can understand**.

An operator watches the live camera feed on a dashboard, clicks a detected target to lock it, and from that moment the system:

1. keeps the gimbal camera pointed at the target automatically, and
2. streams the target's angular offset **from the aircraft's nose** (yaw error + pitch error, in degrees) to the flight controller over MAVLink, 10 times per second.

The output is **advisory telemetry only** — nothing in this codebase commands or flies the aircraft. The flight controller (or a script running on it) decides what to do with the numbers. The intended use case is **dive correction**: during a dive toward a target, the FCC can read "target is 2.1° right and 0.4° below your nose" and trim its trajectory. The math is valid at any dive angle from 0° to 90°.

## The core problem this solves

A camera on a gimbal does **not** point where the aircraft points. The gimbal aims itself independently, so "the target is 30 pixels right of the image center" says nothing useful to a flight controller by itself. Three coordinate frames have to be reconciled:

| Frame | Reported by | Yaw measured from |
|---|---|---|
| Camera (pixels) | The tracker | image center |
| Gimbal attitude | SIYI ZR10 (own IMU, follow mode) | the aircraft's nose |
| Aircraft attitude | Flight controller (MAVLink `ATTITUDE`) | compass North |

The pipeline converts pixel offset → camera-frame angles (pinhole model, `atan(px / focal_px)`), computes the camera-vs-nose rotation by composing the gimbal and FCC attitudes (the shared compass heading cancels out), then rotates the target ray into the aircraft body frame. The final pitch error is measured against the body's horizontal plane rather than the forward axis, which is what keeps the math stable all the way to a 90° off-boresight target — and hence valid for near-vertical dives. The full derivation, with worked numeric examples and its honest limitations (mounting bias, IMU disagreement), is in [system_math.md](system_math.md).

## What gets sent

A MAVLink `DEBUG_VECT` message named **`VIS_ERR`** at 10 Hz, only while a target is locked:

- **x** = yaw error in degrees, positive = target is right of the nose (steer right)
- **y** = pitch error in degrees, positive = target is above the nose (steer up)
- **z** = unused (0)

`DEBUG_VECT` is deliberately passive: autopilots log and display it but never act on it, so the aircraft cannot be affected by this system alone.

## Hardware

| Component | Role | Link |
|---|---|---|
| SIYI ZR10 gimbal camera | Video source + steerable mount | RTSP video (`192.168.144.25:8554`) + SIYI UDP SDK (port 37260) |
| Flight controller (CubeOrange+, ArduPilot) | Attitude source + telemetry sink | MAVLink over USB (`/dev/ttyACM0` @ 115200) |
| Companion computer (tested on NVIDIA Jetson) | Runs everything in this repo | — |

The prototype tracks **human heads** (YuNet face-detection CNN, bundled as `face_detection_yunet.onnx`) as a stand-in target class; swapping in a different detector only means replacing `HeadDetector` in [head_tracker.py](head_tracker.py).

## Repository layout

| File | What it is |
|---|---|
| [gui_dashboard.py](gui_dashboard.py) | **Main entry point.** OpenCV dashboard: live feed, click-to-lock, gimbal tracking loop, and the MAVLink `VIS_ERR` publisher. Five worker threads (video, gimbal poll, detection, gimbal servo, FCC I/O). |
| [head_tracker.py](head_tracker.py) | YuNet head detector, nearest-box target re-association, and the PID used to steer the gimbal. |
| [fcc_landing_feedback.py](fcc_landing_feedback.py) | All the frame-conversion math: pixel→angle, gimbal/FCC attitude composition, body-frame target angles. Also contains an alternative `LANDING_TARGET` publisher for classic precision-landing setups (not used by the dashboard). |
| [siyi_zr10.py](siyi_zr10.py) | Minimal SIYI SDK client (UDP): attitude polling, rate-controlled rotation, center, zoom, photo. Runnable standalone to print live gimbal attitude. |
| [fcc_attitude.py](fcc_attitude.py) | Standalone utility: print live FCC roll/pitch/yaw over MAVLink. Useful as a link check. |
| [system_math.md](system_math.md) | Plain-language walkthrough of the math, with the real numbers from a live run. |
| [requirements.txt](requirements.txt) | Pinned Python dependencies (numpy, opencv-python, pymavlink, pyserial). |
| `face_detection_yunet.onnx` | YuNet detection model weights. |
| `*.pdf`, `report_src.html` | Project reports / documentation. |

## Running it

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
python3 gui_dashboard.py
```

Expects the ZR10 reachable at its default IP (`192.168.144.25`) and the FCC on `/dev/ttyACM0`; both are constants at the top of [gui_dashboard.py](gui_dashboard.py). The dashboard degrades gracefully — it runs with any subset of camera/gimbal/FCC connected and shows per-link status lights.

**Controls:** left-click a green detection box to lock it · right-click to release · `q` to quit.

Once locked, the box turns red, the gimbal begins centering the target, and — if the FCC link is up — `VIS_ERR` starts streaming. The right-hand "Dive Correction to FCC" card shows the live chain: *nose→camera* + *camera→target* = *nose→target*, the last row being exactly what is sent.

## Status

Working prototype, bench- and ground-tested. Known limitations are documented at the end of [system_math.md](system_math.md): a gimbal mounting twist biases the correction 1:1 (checkable via the boresight check — center the gimbal and confirm CAMERA VS NOSE reads ≈ 0/0/0), and disagreement between the gimbal's and FCC's IMUs leaks directly into the output. Compass error does *not* matter — heading cancels in the math.
