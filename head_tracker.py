#!/usr/bin/env python3
"""Head detection (YuNet CNN) + click-to-lock nearest-match tracking + pixel-error PID."""
import math
import os

import cv2

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_detection_yunet.onnx")
DETECT_W, DETECT_H = 640, 360
MATCH_MAX_DIST_FRAC = 0.25  # fraction of frame diagonal; beyond this, a detection is not "the same" target


class HeadDetector:
    def __init__(self, model_path=MODEL_PATH, score_threshold=0.6):
        self.detector = cv2.FaceDetectorYN_create(model_path, "", (DETECT_W, DETECT_H), score_threshold=score_threshold)

    def detect(self, frame):
        """Returns list of (x, y, w, h) boxes in the input frame's own coordinate space."""
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (DETECT_W, DETECT_H))
        _, faces = self.detector.detect(small)
        if faces is None:
            return []
        sx, sy = w / DETECT_W, h / DETECT_H
        boxes = []
        for f in faces:
            x, y, bw, bh = f[0] * sx, f[1] * sy, f[2] * sx, f[3] * sy
            boxes.append((int(x), int(y), int(bw), int(bh)))
        return boxes


def box_center(box):
    x, y, w, h = box
    return (x + w / 2.0, y + h / 2.0)


def find_nearest_box(boxes, target_point, max_dist):
    best, best_dist = None, None
    for box in boxes:
        cx, cy = box_center(box)
        dist = math.hypot(cx - target_point[0], cy - target_point[1])
        if dist <= max_dist and (best_dist is None or dist < best_dist):
            best, best_dist = box, dist
    return best


class PID:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.out_limit, min(self.out_limit, out))
