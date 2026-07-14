#!/usr/bin/env python3
"""MINI UMWAMBI — vision correction dashboard.

Detects targets in the ZR10 feed, keeps the gimbal on the one the operator
locks, and streams the bare nose-relative angular error to the FCC as passive
MAVLink DEBUG_VECT telemetry (x = yaw error deg, y = pitch error deg). Built
for dive correction at any dive angle (0-90 deg): the error is measured off
the nose, not off the body-down axis, so it stays valid from shallow to
near-vertical dives. Nothing here commands the aircraft.
"""
import math
import threading
import time

import cv2
import numpy as np
from pymavlink import mavutil

from siyi_zr10 import ZR10
from head_tracker import HeadDetector, PID, box_center, find_nearest_box, MATCH_MAX_DIST_FRAC
from fcc_landing_feedback import (
    VisionOffset,
    offset_to_angles,
    body_frame_angles,
    mount_relative_euler,
)

CAMERA_RTSP = "rtsp://192.168.144.25:8554/main.264"
FCC_PORT = "/dev/ttyACM0"
FCC_BAUD = 115200

CAMERA_HFOV_DEG = 65.0
CAMERA_VFOV_DEG = 39.0

SERVO_HZ = 10
SERVO_KP_YAW = 0.05
SERVO_KP_PITCH = 0.05
SERVO_OUT_LIMIT = 50

TX_RATE_HZ = 10.0
TX_VECT_NAME = b"VIS_ERR"  # DEBUG_VECT name: x = yaw error deg (+right), y = pitch error deg (+up)
LINK_STALE_S = 2.0
CONSOLE_LOG_PERIOD_S = 1.0

# ---------------------------------------------------------------- layout ----
WINDOW_W, WINDOW_H = 1280, 696
MARGIN = 12
HEADER_H = 44
TITLE_H = 26

VIDEO_X, VIDEO_Y, VIDEO_W = MARGIN, HEADER_H + MARGIN, 800
VIDEO_IMG_H = 450  # 16:9 inside the card, below its title bar
VIDEO_H = TITLE_H + VIDEO_IMG_H

ATT_X, ATT_Y, ATT_W, ATT_H = MARGIN, VIDEO_Y + VIDEO_H + MARGIN, VIDEO_W, 124

RIGHT_X = VIDEO_X + VIDEO_W + MARGIN
RIGHT_W = WINDOW_W - RIGHT_X - MARGIN
TARGET_Y, TARGET_H = VIDEO_Y, 220
TX_Y, TX_H = TARGET_Y + TARGET_H + MARGIN, VIDEO_H - TARGET_H - MARGIN
LINK_Y, LINK_H = ATT_Y, ATT_H

# BGR palette
C_BG = (14, 14, 14)
C_CARD = (26, 26, 26)
C_CARD_HEAD = (38, 38, 38)
C_BORDER = (58, 58, 58)
C_TEXT = (235, 235, 235)
C_MUTED = (140, 140, 140)
C_GREEN = (96, 205, 110)
C_AMBER = (0, 190, 255)
C_RED = (80, 80, 235)
C_ACCENT = (215, 170, 80)

FONT = cv2.FONT_HERSHEY_SIMPLEX

state_lock = threading.Lock()
state = {
    "frame": None,
    "gimbal": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
    "fcc": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
    "detections": [],
    "locked_box": None,
    "locked": False,
    "tx": {"connected": False, "count": 0, "last_sent": 0.0},
    "ts": {"frame": 0.0, "gimbal": 0.0, "fcc": 0.0},
    "running": True,
}


# --------------------------------------------------------------- workers ----
def video_worker():
    cap = cv2.VideoCapture(CAMERA_RTSP, cv2.CAP_FFMPEG)
    while state["running"]:
        ok, frame = cap.read()
        if ok:
            with state_lock:
                state["frame"] = frame
                state["ts"]["frame"] = time.time()
        else:
            time.sleep(0.1)
    cap.release()


def gimbal_worker():
    cam = ZR10()
    while state["running"]:
        att = cam.attitude()
        if att:
            with state_lock:
                state["gimbal"] = {"pitch": att["pitch"], "yaw": att["yaw"], "roll": att["roll"]}
                state["ts"]["gimbal"] = time.time()
        time.sleep(0.1)


def detection_worker():
    detector = HeadDetector()
    while state["running"]:
        with state_lock:
            frame = state["frame"]
        if frame is None:
            time.sleep(0.05)
            continue
        detections = detector.detect(frame)
        h, w = frame.shape[:2]
        max_dist = MATCH_MAX_DIST_FRAC * math.hypot(w, h)
        with state_lock:
            state["detections"] = detections
            if state["locked"] and state["locked_box"] is not None:
                matched = find_nearest_box(detections, box_center(state["locked_box"]), max_dist)
                if matched is not None:
                    state["locked_box"] = matched
                else:
                    state["locked"] = False
                    state["locked_box"] = None


def servo_worker():
    cam = ZR10(timeout=0.3)
    pid_yaw = PID(SERVO_KP_YAW, 0.0, 0.01, SERVO_OUT_LIMIT)
    pid_pitch = PID(SERVO_KP_PITCH, 0.0, 0.01, SERVO_OUT_LIMIT)
    period = 1.0 / SERVO_HZ
    was_locked = False
    while state["running"]:
        with state_lock:
            locked = state["locked"]
            locked_box = state["locked_box"]
            frame = state["frame"]
        if locked and locked_box is not None and frame is not None:
            h, w = frame.shape[:2]
            cx, cy = box_center(locked_box)
            err_x = cx - w / 2.0
            err_y = cy - h / 2.0
            yaw_rate = pid_yaw.update(err_x, period)
            pitch_rate = pid_pitch.update(-err_y, period)
            cam.rotate(int(yaw_rate), int(pitch_rate))
            was_locked = True
        elif was_locked:
            cam.rotate(0, 0)
            pid_yaw.reset()
            pid_pitch.reset()
            was_locked = False
        time.sleep(period)


def fcc_worker():
    """Reads FCC attitude and streams the nose-relative error while locked.
    Reconnects on link loss so the dashboard keeps running without the FCC."""
    while state["running"]:
        try:
            master = mavutil.mavlink_connection(FCC_PORT, baud=FCC_BAUD)
            if master.wait_heartbeat(timeout=5) is None:
                master.close()
                time.sleep(1.0)
                continue
        except Exception:
            time.sleep(1.0)
            continue
        with state_lock:
            state["tx"]["connected"] = True
            state["ts"]["fcc"] = time.time()
        next_send = 0.0
        while state["running"]:
            try:
                msg = master.recv_match(type=["ATTITUDE", "HEARTBEAT"], blocking=True, timeout=0.2)
            except Exception:
                break
            if msg:
                with state_lock:
                    state["ts"]["fcc"] = time.time()
                    if msg.get_type() == "ATTITUDE":
                        state["fcc"] = {
                            "pitch": math.degrees(msg.pitch),
                            "yaw": math.degrees(msg.yaw),
                            "roll": math.degrees(msg.roll),
                        }
            if time.monotonic() >= next_send:
                if publish_correction(master):
                    next_send = time.monotonic() + 1.0 / TX_RATE_HZ
        with state_lock:
            state["tx"]["connected"] = False
        try:
            master.close()
        except Exception:
            pass


def publish_correction(master):
    """Sends the nose-relative error as DEBUG_VECT: x = yaw deg, y = pitch deg.
    Returns True when a message went out (used for rate limiting)."""
    with state_lock:
        locked = state["locked"]
        locked_box = state["locked_box"]
        frame = state["frame"]
        gimbal = dict(state["gimbal"])
        fcc = dict(state["fcc"])
    if not locked or locked_box is None or frame is None:
        return False
    h, w = frame.shape[:2]
    cx, cy = box_center(locked_box)
    offset = VisionOffset(cx - w / 2.0, cy - h / 2.0, w, h)
    mount_rel = mount_relative_euler(gimbal, fcc)
    body_x, body_y = body_frame_angles(offset, mount_rel, CAMERA_HFOV_DEG, CAMERA_VFOV_DEG)
    master.mav.debug_vect_send(
        TX_VECT_NAME,
        int(time.time() * 1_000_000),
        math.degrees(body_x),
        -math.degrees(body_y),
        0.0,
    )
    with state_lock:
        state["tx"]["count"] += 1
        state["tx"]["last_sent"] = time.time()
    return True


# ------------------------------------------------------------ correction ----
def compute_correction(frame, locked_box, gimbal, fcc):
    """All the numbers shown for a locked target, derived once per render."""
    if frame is None or locked_box is None:
        return None
    h, w = frame.shape[:2]
    cx, cy = box_center(locked_box)
    offset = VisionOffset(cx - w / 2.0, cy - h / 2.0, w, h)
    cam_x, cam_y = offset_to_angles(offset, CAMERA_HFOV_DEG, CAMERA_VFOV_DEG)
    mount_rel = mount_relative_euler(gimbal, fcc)
    body_x, body_y = body_frame_angles(offset, mount_rel, CAMERA_HFOV_DEG, CAMERA_VFOV_DEG)
    return {
        "pixel_x": offset.pixel_x,
        "pixel_y": offset.pixel_y,
        "cam_yaw": math.degrees(cam_x),
        "cam_pitch": -math.degrees(cam_y),
        "body_yaw": math.degrees(body_x),
        "body_pitch": -math.degrees(body_y),
    }


# --------------------------------------------------------------- drawing ----
def put(canvas, text, x, y, scale=0.55, color=C_TEXT, thick=1):
    cv2.putText(canvas, text, (x, y), FONT, scale, color, thick, cv2.LINE_AA)


def card(canvas, x, y, w, h, title):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), C_CARD, -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + TITLE_H), C_CARD_HEAD, -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), C_BORDER, 1)
    put(canvas, title.upper(), x + 10, y + 18, 0.46, C_TEXT, 1)


def draw_header(canvas, ts, tx, now):
    cv2.rectangle(canvas, (0, 0), (WINDOW_W, HEADER_H), C_CARD_HEAD, -1)
    cv2.line(canvas, (0, HEADER_H), (WINDOW_W, HEADER_H), C_BORDER, 1)
    title = "MINI UMWAMBI"
    put(canvas, title, MARGIN, 29, 0.75, C_TEXT, 2)
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.75, 2)
    put(canvas, "VISION CORRECTION TO FCC", MARGIN + tw + 24, 29, 0.5, C_MUTED, 1)
    links = [
        ("CAMERA", now - ts["frame"] < LINK_STALE_S),
        ("GIMBAL", now - ts["gimbal"] < LINK_STALE_S),
        ("FCC", tx["connected"] and now - ts["fcc"] < LINK_STALE_S),
    ]
    x = WINDOW_W - MARGIN
    for name, ok in reversed(links):
        (tw, _), _ = cv2.getTextSize(name, FONT, 0.48, 1)
        x -= tw
        put(canvas, name, x, 28, 0.48, C_TEXT if ok else C_MUTED, 1)
        x -= 14
        cv2.circle(canvas, (x, 23), 5, C_GREEN if ok else C_RED, -1)
        x -= 24


def draw_video(canvas, frame, detections, locked_box):
    card(canvas, VIDEO_X, VIDEO_Y, VIDEO_W, VIDEO_H, "Live Feed - SIYI ZR10")
    ix, iy = VIDEO_X, VIDEO_Y + TITLE_H
    if frame is None:
        put(canvas, "NO VIDEO SIGNAL", ix + 20, iy + VIDEO_IMG_H // 2, 0.8, C_RED, 2)
        return
    annotated = frame.copy()
    h, w = annotated.shape[:2]
    cv2.drawMarker(annotated, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 26, 1)
    for box in detections:
        x, y, bw, bh = box
        is_locked = locked_box is not None and box == locked_box
        color = (80, 80, 235) if is_locked else (96, 205, 110)
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), color, 3 if is_locked else 2)
        if is_locked:
            cv2.putText(annotated, "LOCKED", (x, max(0, y - 10)), FONT, 0.7, color, 2, cv2.LINE_AA)
            cx, cy = box_center(box)
            cv2.line(annotated, (w // 2, h // 2), (int(cx), int(cy)), color, 1, cv2.LINE_AA)
    canvas[iy:iy + VIDEO_IMG_H, ix:ix + VIDEO_W] = cv2.resize(annotated, (VIDEO_W, VIDEO_IMG_H))


def draw_value_row(canvas, x, y, label, value, unit="", color=C_TEXT, label_w=185):
    put(canvas, label, x, y, 0.48, C_MUTED, 1)
    put(canvas, value, x + label_w, y, 0.56, color, 2)
    if unit:
        (tw, _), _ = cv2.getTextSize(value, FONT, 0.56, 2)
        put(canvas, unit, x + label_w + tw + 8, y, 0.42, C_MUTED, 1)


def draw_target_card(canvas, locked, n_detections, correction):
    card(canvas, RIGHT_X, TARGET_Y, RIGHT_W, TARGET_H, "Target Tracking")
    x, y = RIGHT_X + 12, TARGET_Y + TITLE_H + 30
    if not locked or correction is None:
        put(canvas, "NO TARGET LOCKED", x, y, 0.58, C_AMBER, 2)
        put(canvas, "LEFT-CLICK A DETECTION TO LOCK", x, y + 28, 0.48, C_MUTED, 1)
        put(canvas, f"DETECTIONS IN VIEW: {n_detections}", x, y + 56, 0.48, C_MUTED, 1)
        return
    put(canvas, "TARGET LOCKED - GIMBAL TRACKING", x, y, 0.54, C_GREEN, 2)
    y += 36
    draw_value_row(canvas, x, y, "PIXEL ERROR X / Y",
                   f"{correction['pixel_x']:+.0f} / {correction['pixel_y']:+.0f}", "px")
    y += 30
    draw_value_row(canvas, x, y, "OFF BORESIGHT YAW",
                   f"{correction['cam_yaw']:+.2f}", "deg")
    y += 30
    draw_value_row(canvas, x, y, "OFF BORESIGHT PITCH",
                   f"{correction['cam_pitch']:+.2f}", "deg")
    y += 30
    put(canvas, "ANGLES FROM THE CAMERA LENS CENTERLINE", x, y, 0.4, C_MUTED, 1)


def draw_tx_card(canvas, locked, correction, mount_rel, tx, now):
    card(canvas, RIGHT_X, TX_Y, RIGHT_W, TX_H, "Dive Correction to FCC - MAVLink")
    x, y = RIGHT_X + 12, TX_Y + TITLE_H + 28
    if correction is None:
        put(canvas, "AWAITING TARGET LOCK", x, y, 0.54, C_MUTED, 2)
    else:
        col_pitch, col_yaw = x + 240, x + 340
        rows = [
            ("(DEGREES)", "PITCH", "YAW", C_MUTED, 0.42, 1),
            ("UAV NOSE > CAMERA", f"{mount_rel['pitch']:+.1f}", f"{mount_rel['yaw']:+.1f}", C_TEXT, 0.48, 1),
            ("+ CAMERA > TARGET", f"{correction['cam_pitch']:+.1f}", f"{correction['cam_yaw']:+.1f}", C_TEXT, 0.48, 1),
            ("= UAV NOSE > TARGET", f"{correction['body_pitch']:+.1f}", f"{correction['body_yaw']:+.1f}", C_ACCENT, 0.54, 2),
        ]
        for label, pitch_txt, yaw_txt, color, scale, thick in rows:
            put(canvas, label, x, y, scale, color, thick)
            put(canvas, pitch_txt, col_pitch, y, scale, color, thick)
            put(canvas, yaw_txt, col_yaw, y, scale, color, thick)
            y += 26
        cv2.line(canvas, (x, y - 46), (RIGHT_X + RIGHT_W - 12, y - 46), C_BORDER, 1)
        #put(canvas, "COMBINED AS 3D ROTATIONS - NOT AN EXACT SUM", x, y - 6, 0.38, C_MUTED, 1)
        y += 26
        put(canvas, "STEER TOWARD TARGET: + YAW RIGHT, + PITCH UP", x, y, 0.44, C_TEXT, 1)
        y += 24
        put(canvas, f"SENT AS DEBUG_VECT \"{TX_VECT_NAME.decode()}\"  X=YAW  Y=PITCH", x, y, 0.4, C_MUTED, 1)

    if not tx["connected"]:
        status, color = "FCC OFFLINE - NOTHING SENT", C_RED
    elif not locked or correction is None:
        status, color = "LINKED - WAITING FOR LOCK", C_MUTED
    elif now - tx["last_sent"] < 1.0:
        status, color = f"STREAMING {TX_RATE_HZ:.0f} HZ - SENT {tx['count']}", C_GREEN
    else:
        status, color = "LINKED - IDLE", C_MUTED
    put(canvas, status, x, TX_Y + TX_H - 14, 0.52, color, 2)


def draw_attitude_card(canvas, fcc, gimbal, mount_rel):
    card(canvas, ATT_X, ATT_Y, ATT_W, ATT_H, "Attitude")
    cols = [
        ("FCC (MAVLINK)", fcc, C_TEXT),
        ("GIMBAL (ZR10)", gimbal, C_TEXT),
        ("CAMERA VS NOSE", mount_rel, C_ACCENT),
    ]
    col_w = ATT_W // 3
    for i, (title, att, color) in enumerate(cols):
        x = ATT_X + 12 + i * col_w
        y = ATT_Y + TITLE_H + 22
        put(canvas, title, x, y, 0.46, C_MUTED, 1)
        for name in ("pitch", "yaw", "roll"):
            y += 22
            put(canvas, f"{name.upper():<6}{att[name]:+8.2f} DEG", x, y, 0.5, color, 1)
        if i:
            cv2.line(canvas, (ATT_X + i * col_w, ATT_Y + TITLE_H + 8),
                     (ATT_X + i * col_w, ATT_Y + ATT_H - 8), C_BORDER, 1)


def draw_link_card(canvas, ts, tx, now):
    card(canvas, RIGHT_X, LINK_Y, RIGHT_W, LINK_H, "System")
    rows = [
        ("CAMERA FEED", now - ts["frame"] < LINK_STALE_S, "RTSP " + CAMERA_RTSP.split("//")[1].split(":")[0]),
        ("GIMBAL SDK", now - ts["gimbal"] < LINK_STALE_S, "UDP ATTITUDE POLL"),
        ("FCC MAVLINK", tx["connected"] and now - ts["fcc"] < LINK_STALE_S, FCC_PORT),
    ]
    y = LINK_Y + TITLE_H + 26
    for name, ok, detail in rows:
        cv2.circle(canvas, (RIGHT_X + 18, y - 5), 5, C_GREEN if ok else C_RED, -1)
        put(canvas, name, RIGHT_X + 32, y, 0.48, C_TEXT, 1)
        put(canvas, detail.upper(), RIGHT_X + 180, y, 0.42, C_MUTED, 1)
        y += 26


def draw_footer(canvas):
    put(canvas, "LEFT-CLICK  LOCK TARGET      RIGHT-CLICK  RELEASE      Q  QUIT",
        MARGIN, WINDOW_H - 10, 0.45, C_MUTED, 1)


def compose(snap, now):
    canvas = np.full((WINDOW_H, WINDOW_W, 3), C_BG, dtype=np.uint8)
    mount_rel = mount_relative_euler(snap["gimbal"], snap["fcc"])
    correction = compute_correction(snap["frame"], snap["locked_box"], snap["gimbal"], snap["fcc"])
    draw_header(canvas, snap["ts"], snap["tx"], now)
    draw_video(canvas, snap["frame"], snap["detections"], snap["locked_box"])
    draw_target_card(canvas, snap["locked"], len(snap["detections"]), correction)
    draw_tx_card(canvas, snap["locked"], correction, mount_rel, snap["tx"], now)
    draw_attitude_card(canvas, snap["fcc"], snap["gimbal"], mount_rel)
    draw_link_card(canvas, snap["ts"], snap["tx"], now)
    draw_footer(canvas)
    return canvas, correction


# ------------------------------------------------------------------ main ----
def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        with state_lock:
            frame = state["frame"]
            detections = list(state["detections"])
        ix, iy = VIDEO_X, VIDEO_Y + TITLE_H
        if frame is None or not (ix <= x < ix + VIDEO_W and iy <= y < iy + VIDEO_IMG_H):
            return
        h0, w0 = frame.shape[:2]
        raw_x = (x - ix) * (w0 / VIDEO_W)
        raw_y = (y - iy) * (h0 / VIDEO_IMG_H)
        for box in detections:
            bx, by, bw, bh = box
            if bx <= raw_x <= bx + bw and by <= raw_y <= by + bh:
                with state_lock:
                    state["locked_box"] = box
                    state["locked"] = True
                break
    elif event == cv2.EVENT_RBUTTONDOWN:
        with state_lock:
            state["locked"] = False
            state["locked_box"] = None


def main():
    print("Centering gimbal...")
    ZR10().center()
    time.sleep(2)

    for worker in (video_worker, gimbal_worker, fcc_worker, detection_worker, servo_worker):
        threading.Thread(target=worker, daemon=True).start()

    window_name = "MINI UMWAMBI - Vision Correction Dashboard"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)
    last_log = 0.0

    try:
        while True:
            now = time.time()
            with state_lock:
                snap = {
                    "frame": state["frame"],
                    "gimbal": dict(state["gimbal"]),
                    "fcc": dict(state["fcc"]),
                    "detections": list(state["detections"]),
                    "locked_box": state["locked_box"],
                    "locked": state["locked"],
                    "tx": dict(state["tx"]),
                    "ts": dict(state["ts"]),
                }
            canvas, correction = compose(snap, now)

            if correction is not None and now - last_log >= CONSOLE_LOG_PERIOD_S:
                print(
                    f"[correction] nose->target yaw {correction['body_yaw']:+6.2f} deg  "
                    f"pitch {correction['body_pitch']:+6.2f} deg | sent {snap['tx']['count']}"
                )
                last_log = now

            try:
                cv2.imshow(window_name, canvas)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    except KeyboardInterrupt:
        pass
    finally:
        state["running"] = False
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
