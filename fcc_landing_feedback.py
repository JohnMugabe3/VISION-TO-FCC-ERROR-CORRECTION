#!/usr/bin/env python3
"""Vision-to-FCC landing correction feedback via MAVLink LANDING_TARGET."""
import math
import time
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VisionOffset:
    """Target offset from the camera optical center."""

    pixel_x: float
    pixel_y: float
    frame_w: int
    frame_h: int


def offset_to_angles(offset: VisionOffset, hfov_deg=65.0, vfov_deg=39.0):
    """Bearing of the target from the camera's own optical boresight (camera frame).

    This says nothing about where the target is relative to the aircraft body unless
    the camera boresight happens to coincide with the body's forward axis (gimbal
    centered). Use `body_frame_angles` for the angle that's actually valid to report
    to the FCC as MAV_FRAME_BODY_FRD.
    """
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.radians(vfov_deg)
    fx = offset.frame_w / (2.0 * math.tan(hfov_rad / 2.0))
    fy = offset.frame_h / (2.0 * math.tan(vfov_rad / 2.0))
    angle_x = math.atan2(offset.pixel_x, fx)
    angle_y = math.atan2(offset.pixel_y, fy)
    return angle_x, angle_y


def euler_to_matrix(roll_deg, pitch_deg, yaw_deg):
    """Mount-to-body rotation matrix from Euler angles (degrees), ZYX (yaw-pitch-roll) order."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_euler(r):
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    pitch = math.degrees(math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2])))
    roll = math.degrees(math.atan2(r[2, 1], r[2, 2]))
    return {"pitch": pitch, "yaw": yaw, "roll": roll}


def camera_ray(offset: VisionOffset, hfov_deg=65.0, vfov_deg=39.0):
    """Pinhole ray to the target in camera FRD axes (x out the lens, y right, z down).
    Consumers use atan2, which is scale-invariant, so the ray is not normalized."""
    angle_x, angle_y = offset_to_angles(offset, hfov_deg, vfov_deg)
    return np.array([1.0, math.tan(angle_x), math.tan(angle_y)])


def body_frame_angles(offset: VisionOffset, mount_attitude, hfov_deg=65.0, vfov_deg=39.0):
    """Bearing of the target off the vehicle's nose: (azimuth, elevation-style).

    `mount_attitude` must already be mount-relative (camera relative to the body,
    e.g. from `mount_relative_euler`). angle_x is the azimuth off the nose
    (positive right); angle_y is measured against the body's horizontal plane
    (positive down) rather than against the forward axis, so it stays sane when
    the target is far off to the side — atan2(down, forward) would blow up as
    the forward component approaches zero near 90 deg of azimuth.
    """
    r_mount = euler_to_matrix(mount_attitude["roll"], mount_attitude["pitch"], mount_attitude["yaw"])
    v_body = r_mount @ camera_ray(offset, hfov_deg, vfov_deg)
    angle_x = math.atan2(v_body[1], v_body[0])
    angle_y = math.atan2(v_body[2], math.hypot(v_body[0], v_body[1]))
    return angle_x, angle_y


# Rigidly mounted straight-down camera: the classic companion-computer
# precision-landing setup, and the assumption baked into the FCC side.
DOWN_CAMERA_ATTITUDE = {"roll": 0.0, "pitch": -90.0, "yaw": 0.0}


def siyi_body_from_camera_matrix(gimbal_attitude, aircraft_attitude=None):
    """Rotation taking camera-frame vectors into the vehicle body frame.

    SIYI gimbals stabilize against vehicle motion with their own IMU, so their
    reported pitch/roll are earth-referenced, while yaw in follow mode is relative
    to the vehicle nose. Composing with the aircraft attitude strips the vehicle's
    own tilt out of the report, leaving a true mount-relative rotation. Without an
    aircraft attitude the vehicle is assumed level and the report is used directly
    as mount-relative.
    """
    if aircraft_attitude is None:
        return euler_to_matrix(gimbal_attitude["roll"], gimbal_attitude["pitch"], gimbal_attitude["yaw"])
    r_world_from_body = euler_to_matrix(
        aircraft_attitude["roll"], aircraft_attitude["pitch"], aircraft_attitude["yaw"]
    )
    r_world_from_cam = euler_to_matrix(
        gimbal_attitude["roll"], gimbal_attitude["pitch"], aircraft_attitude["yaw"] + gimbal_attitude["yaw"]
    )
    return r_world_from_body.T @ r_world_from_cam


def mount_relative_euler(gimbal_attitude, aircraft_attitude=None):
    """Camera attitude relative to the vehicle body (degrees), from SIYI conventions."""
    return matrix_to_euler(siyi_body_from_camera_matrix(gimbal_attitude, aircraft_attitude))


def landing_target_angles(offset: VisionOffset, gimbal_attitude=None, aircraft_attitude=None, hfov_deg=65.0, vfov_deg=39.0):
    """LANDING_TARGET angles in the down-looking convention ArduPilot/PX4 expect.

    Both autopilots interpret angle_x/angle_y as the target's angular offset from
    the vehicle's body DOWN axis — the frame of a rigidly mounted downward camera
    (IRLock-style) with image-up aligned to the nose:
      angle_x positive = target to the vehicle's right
      angle_y positive = target behind, toward the tail
    The FCC rotates these by its own attitude on receipt, so they must be body-frame
    and NOT pre-leveled for vehicle tilt. With the gimbal pointing straight down and
    the vehicle level this reduces exactly to the raw pixel angles.
    """
    gimbal = gimbal_attitude if gimbal_attitude is not None else DOWN_CAMERA_ATTITUDE
    r = siyi_body_from_camera_matrix(gimbal, aircraft_attitude)
    v_body = r @ camera_ray(offset, hfov_deg, vfov_deg)
    angle_x = math.atan2(v_body[1], v_body[2])
    angle_y = math.atan2(-v_body[0], v_body[2])
    return angle_x, angle_y


class LandingTargetPublisher:
    """Publishes vision offsets as MAVLink LANDING_TARGET observations.

    This intentionally reports the target to the flight controller instead of
    sending raw position/velocity setpoints. The FCC remains responsible for
    deciding whether precision landing is active, how hard to correct, and when
    to reject stale or invalid observations.
    """

    def __init__(
        self,
        mav,
        mavlink,
        hfov_deg=65.0,
        vfov_deg=39.0,
        rate_hz=10.0,
        max_angle_deg=20.0,
        target_num=0,
    ):
        self.mav = mav
        self.mavlink = mavlink
        self.hfov_rad = math.radians(hfov_deg)
        self.vfov_rad = math.radians(vfov_deg)
        self.period = 1.0 / rate_hz
        self.max_angle_rad = math.radians(max_angle_deg)
        self.target_num = target_num
        self.next_send = 0.0

    def due(self, now=None):
        return (time.monotonic() if now is None else now) >= self.next_send

    def send_offset(self, offset: VisionOffset, gimbal_attitude=None, aircraft_attitude=None, distance_m=0.0, now=None):
        """Report a target observation to the FCC.

        `gimbal_attitude` is the SIYI-reported gimbal attitude (degrees): pitch/roll
        earth-referenced, yaw follow-mode (relative to the vehicle nose). If omitted,
        a rigidly mounted straight-down camera is assumed, in which case the sent
        angles reduce exactly to the raw pixel angles — the classic companion-computer
        precision-landing setup. `aircraft_attitude` is the FCC ATTITUDE (degrees),
        used to strip the vehicle's own tilt out of the gimbal's earth-referenced
        report; the autopilot re-applies its attitude on receipt, so the sent angles
        must stay body-frame. The angle gate rejects observations more than
        `max_angle_deg` off the body-down axis.
        """
        now = time.monotonic() if now is None else now
        if now < self.next_send:
            return False
        self.next_send = now + self.period

        angle_x, angle_y = self.landing_target_angles(offset, gimbal_attitude, aircraft_attitude)
        if abs(angle_x) > self.max_angle_rad or abs(angle_y) > self.max_angle_rad:
            return False

        self._send_landing_target(angle_x, angle_y, distance_m)
        return True

    def offset_to_angles(self, offset: VisionOffset):
        return offset_to_angles(offset, math.degrees(self.hfov_rad), math.degrees(self.vfov_rad))

    def landing_target_angles(self, offset: VisionOffset, gimbal_attitude=None, aircraft_attitude=None):
        return landing_target_angles(
            offset, gimbal_attitude, aircraft_attitude, math.degrees(self.hfov_rad), math.degrees(self.vfov_rad)
        )

    def _send_landing_target(self, angle_x, angle_y, distance_m):
        time_usec = int(time.time() * 1_000_000)
        frame = getattr(self.mavlink, "MAV_FRAME_BODY_FRD", 12)

        self.mav.landing_target_send(
            time_usec,
            self.target_num,
            frame,
            angle_x,
            angle_y,
            distance_m,
            0.0,
            0.0,
        )
