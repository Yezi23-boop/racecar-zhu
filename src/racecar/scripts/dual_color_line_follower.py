#!/usr/bin/env python3

"""Follow the center between two colored lines with an Ackermann racecar."""

import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# ======================== User configuration ========================

# The two boundary lines may use the same color or different colors.
# Supported values: "red", "blue", "white", "yellow".
LEFT_LINE_COLOR = "yellow"
RIGHT_LINE_COLOR = "yellow"

# True: show the camera image. False: show the black/white line mask.
SHOW_CAMERA_IMAGE = False

# Record one lightweight annotated camera stream. A small mask inset shows raw
# HSV detections in white and the selected lane boundaries in green.
RECORD_DIAGNOSTIC_VIDEO = True
DIAGNOSTIC_VIDEO_DIRECTORY = "/home/bianbu/racecar/src/racecar/scripts/video"
DIAGNOSTIC_VIDEO_PREFIX = "dual_line"
DIAGNOSTIC_VIDEO_CODEC = "MJPG"
DIAGNOSTIC_VIDEO_EXTENSION = ".avi"
DIAGNOSTIC_VIDEO_FPS = 6.0
DIAGNOSTIC_VIDEO_SCALE = 0.75
DIAGNOSTIC_MASK_INSET_SCALE = 0.32

# OpenCV HSV ranges: H=0..179, S=0..255, V=0..255.
COLOR_HSV_RANGES = {
    "red": [
        ((0, 100, 70), (10, 255, 255)),
        ((170, 100, 70), (179, 255, 255)),
    ],
    "blue": [
        ((95, 90, 60), (135, 255, 255)),
    ],
    "white": [
        ((0, 0, 180), (179, 75, 255)),
    ],
    "yellow": [
        ((14, 30, 100), (45, 255, 255)),
    ],
}

CAMERA_INDEX = 20
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30.0
DISPLAY_SCALE = 0.5
DISPLAY_FPS = 15.0

# Only this lower region is analyzed.
ROI_TOP_RATIO = 0.45
ROI_BOTTOM_RATIO = 0.95
MIN_LINE_AREA = 250.0
MIN_LINE_SPAN = 45
MAX_LINE_AREA_RATIO = 0.12
BLUR_KERNEL = 5
MORPH_KERNEL = 5
MAX_LINE_CANDIDATES = 8

# Each contour is sampled in fixed horizontal layers. Real lane lines normally
# cross several layers, while isolated reflections tend to occur in only one.
SCAN_BAND_RATIOS = (0.12, 0.30, 0.48, 0.66, 0.84)
SCAN_BAND_HALF_HEIGHT = 8
LOOKAHEAD_WEIGHT = 0.75

# Temporal association keeps left/right identities stable between frames.
MIN_NEW_TRACK_QUALITY = 0.45
MIN_EXISTING_TRACK_QUALITY = 0.18
MIN_TRACK_MATCH_SCORE = 0.12
TRACK_HORIZONTAL_GATE_RATIO = 0.36
TRACK_VERTICAL_GATE_RATIO = 0.48
TRACK_MEMORY_FRAMES = 8
TRACK_VELOCITY_SMOOTHING = 0.45
PAIR_SCORE_BONUS = 0.40

# Valid distance between the two line centers.
MIN_LANE_WIDTH_RATIO = 0.20
MAX_LANE_WIDTH_RATIO = 0.90
DEFAULT_LANE_WIDTH_RATIO = 0.55
LANE_WIDTH_SMOOTHING = 0.25

# Add a small offset here if the camera is not mounted at the vehicle center.
# Positive moves the desired path to the right; negative moves it to the left.
CENTER_OFFSET_RATIO = 0.0

CONTROL_TOPIC = "/car_cmd_vel"
STOP_PWM = 1500.0
STRAIGHT_DRIVE_PWM = 1546.0
TURN_DRIVE_PWM = 1545.0
DRIVER_SPEED_SCALE = 100.0
CONTROL_RATE = 15.0
TERMINAL_LOG_RATE = 5.0

# This car turns right for negative angular.z and left for positive angular.z.
STEERING_DIRECTION = -1.0
STEERING_LIMIT = 0.60
STEERING_GAIN = 0.65
STEERING_SMOOTHING = 0.55
CENTER_DEADBAND = 0.025
TURN_SPEED_THRESHOLD = 0.06

# Ackermann recovery: keep rolling briefly in the last steering direction if
# both boundaries leave the camera while the vehicle is already turning.
LOST_LANE_RECOVERY_SECONDS = 1.2
RECOVERY_TRIGGER_STEERING = 0.06
RECOVERY_MIN_STEERING = 0.32

# ===================================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def validate_configuration():
    errors = []
    for setting_name, color in (
        ("LEFT_LINE_COLOR", LEFT_LINE_COLOR),
        ("RIGHT_LINE_COLOR", RIGHT_LINE_COLOR),
    ):
        if color not in COLOR_HSV_RANGES:
            errors.append(
                f"{setting_name} must be one of: {', '.join(COLOR_HSV_RANGES)}"
            )
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0 or CAMERA_FPS <= 0:
        errors.append("camera width, height and fps must be greater than zero")
    if DISPLAY_SCALE <= 0 or DISPLAY_FPS <= 0:
        errors.append("display scale and fps must be greater than zero")
    if len(DIAGNOSTIC_VIDEO_CODEC) != 4:
        errors.append("DIAGNOSTIC_VIDEO_CODEC must contain four characters")
    if not DIAGNOSTIC_VIDEO_EXTENSION.startswith("."):
        errors.append("DIAGNOSTIC_VIDEO_EXTENSION must start with a dot")
    if DIAGNOSTIC_VIDEO_FPS <= 0 or DIAGNOSTIC_VIDEO_SCALE <= 0:
        errors.append("diagnostic video fps and scale must be greater than zero")
    if not 0.1 <= DIAGNOSTIC_MASK_INSET_SCALE <= 0.6:
        errors.append("DIAGNOSTIC_MASK_INSET_SCALE must be in [0.1, 0.6]")
    if not 0.0 <= ROI_TOP_RATIO < ROI_BOTTOM_RATIO <= 1.0:
        errors.append("ROI ratios must satisfy 0 <= top < bottom <= 1")
    if MIN_LINE_AREA <= 0 or MIN_LINE_SPAN <= 0:
        errors.append("minimum line area and span must be greater than zero")
    if not 0.0 < MAX_LINE_AREA_RATIO < 1.0:
        errors.append("MAX_LINE_AREA_RATIO must be between 0 and 1")
    if MAX_LINE_CANDIDATES < 2:
        errors.append("line candidate settings are invalid")
    if not SCAN_BAND_RATIOS or any(
        not 0.0 < ratio < 1.0 for ratio in SCAN_BAND_RATIOS
    ):
        errors.append("SCAN_BAND_RATIOS must contain values between 0 and 1")
    if tuple(sorted(SCAN_BAND_RATIOS)) != SCAN_BAND_RATIOS:
        errors.append("SCAN_BAND_RATIOS must be in ascending order")
    if SCAN_BAND_HALF_HEIGHT < 1:
        errors.append("SCAN_BAND_HALF_HEIGHT must be at least 1")
    if not 0.0 <= LOOKAHEAD_WEIGHT <= 1.0:
        errors.append("LOOKAHEAD_WEIGHT must be between 0 and 1")
    if not 0.0 <= MIN_EXISTING_TRACK_QUALITY <= MIN_NEW_TRACK_QUALITY <= 1.0:
        errors.append("track quality thresholds are invalid")
    if MIN_TRACK_MATCH_SCORE < 0.0:
        errors.append("MIN_TRACK_MATCH_SCORE must be >= 0")
    if not 0.0 < TRACK_HORIZONTAL_GATE_RATIO <= 1.0:
        errors.append("TRACK_HORIZONTAL_GATE_RATIO must be in (0, 1]")
    if not 0.0 < TRACK_VERTICAL_GATE_RATIO <= 1.0:
        errors.append("TRACK_VERTICAL_GATE_RATIO must be in (0, 1]")
    if TRACK_MEMORY_FRAMES < 1:
        errors.append("TRACK_MEMORY_FRAMES must be at least 1")
    if not 0.0 < TRACK_VELOCITY_SMOOTHING <= 1.0:
        errors.append("TRACK_VELOCITY_SMOOTHING must be in (0, 1]")
    for name, kernel in (
        ("BLUR_KERNEL", BLUR_KERNEL),
        ("MORPH_KERNEL", MORPH_KERNEL),
    ):
        if kernel < 1 or kernel % 2 == 0:
            errors.append(f"{name} must be a positive odd integer")
    if not (
        0.0
        < MIN_LANE_WIDTH_RATIO
        < DEFAULT_LANE_WIDTH_RATIO
        < MAX_LANE_WIDTH_RATIO
        <= 1.0
    ):
        errors.append("lane width ratios are invalid")
    if not 0.0 < LANE_WIDTH_SMOOTHING <= 1.0:
        errors.append("LANE_WIDTH_SMOOTHING must be in (0, 1]")
    if not -0.5 < CENTER_OFFSET_RATIO < 0.5:
        errors.append("CENTER_OFFSET_RATIO must be between -0.5 and 0.5")
    if TURN_DRIVE_PWM < STOP_PWM or STRAIGHT_DRIVE_PWM < TURN_DRIVE_PWM:
        errors.append("PWM values must satisfy STOP <= TURN <= STRAIGHT")
    if DRIVER_SPEED_SCALE <= 0 or CONTROL_RATE <= 0 or TERMINAL_LOG_RATE <= 0:
        errors.append("control scales and rates must be greater than zero")
    if STEERING_DIRECTION not in (-1.0, 1.0):
        errors.append("STEERING_DIRECTION must be -1.0 or 1.0")
    if STEERING_LIMIT <= 0 or STEERING_GAIN < 0:
        errors.append("steering limit/gain values are invalid")
    if not 0.0 <= STEERING_SMOOTHING <= 1.0:
        errors.append("STEERING_SMOOTHING must be between 0 and 1")
    if not 0.0 <= CENTER_DEADBAND < 1.0:
        errors.append("CENTER_DEADBAND must be in [0, 1)")
    if not 0.0 <= TURN_SPEED_THRESHOLD <= STEERING_LIMIT:
        errors.append("TURN_SPEED_THRESHOLD is outside the steering limit")
    if LOST_LANE_RECOVERY_SECONDS < 0:
        errors.append("LOST_LANE_RECOVERY_SECONDS must be >= 0")
    if not 0.0 <= RECOVERY_TRIGGER_STEERING <= STEERING_LIMIT:
        errors.append("RECOVERY_TRIGGER_STEERING is outside the steering limit")
    if not RECOVERY_TRIGGER_STEERING <= RECOVERY_MIN_STEERING <= STEERING_LIMIT:
        errors.append("RECOVERY_MIN_STEERING is outside the recovery range")
    return errors


@dataclass
class LineObservation:
    center_x: int
    center_y: int
    anchor_x: int
    anchor_y: int
    lookahead_x: int
    lookahead_y: int
    area: float
    box: tuple
    contour: object
    scan_count: int
    quality: float


@dataclass
class LaneDetection:
    left: object
    right: object
    center_x: object
    mask: np.ndarray
    raw_mask: np.ndarray
    candidate_count: int
    roi_top: int
    roi_bottom: int


class LatestFrameReader:
    """Continuously capture frames and expose only the newest one."""

    def __init__(self, capture):
        self.capture = capture
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = None
        self.sequence = 0
        self.last_frame_time = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame
                self.sequence += 1
                self.last_frame_time = time.monotonic()

    def latest(self):
        with self.lock:
            return self.sequence, self.frame, self.last_frame_time

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.capture.release()


class AsyncVideoRecorder:
    """Encode video in the background without blocking vehicle control."""

    def __init__(self, writer):
        self.writer = writer
        self.frames = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.written_frames = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, frame):
        try:
            self.frames.put_nowait(frame)
            return
        except queue.Full:
            pass
        try:
            self.frames.get_nowait()
        except queue.Empty:
            pass
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            pass

    def _run(self):
        try:
            while not self.stop_event.is_set() or not self.frames.empty():
                try:
                    frame = self.frames.get(timeout=0.05)
                except queue.Empty:
                    continue
                self.writer.write(frame)
                self.written_frames += 1
        finally:
            self.writer.release()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            print("warning: diagnostic video encoder did not stop in time")


class DualColorLineFollower(Node):
    def __init__(self):
        super().__init__("dual_color_line_follower")
        self.publisher = self.create_publisher(Twist, CONTROL_TOPIC, 5)
        self.filtered_steering = 0.0
        self.estimated_lane_width = None
        self.estimated_anchor_width = None
        self.left_track = None
        self.right_track = None
        self.left_track_misses = TRACK_MEMORY_FRAMES + 1
        self.right_track_misses = TRACK_MEMORY_FRAMES + 1
        self.left_anchor_velocity = 0.0
        self.right_anchor_velocity = 0.0
        self.last_lane_seen_time = 0.0
        self.last_publish_time = 0.0
        self.last_log_time = 0.0
        self.last_speed_pwm = STOP_PWM
        self.last_steering = 0.0
        self.control_state = "LOST"
        self.path_state = "LOST"

    @staticmethod
    def make_color_mask(hsv, color, roi_mask):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_HSV_RANGES[color]:
            interval = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, interval)
        mask = cv2.bitwise_and(mask, roi_mask)
        kernel = np.ones((MORPH_KERNEL, MORPH_KERNEL), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    @staticmethod
    def find_candidates(mask, roi_top, roi_bottom):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        _, frame_width = mask.shape
        roi_height = max(1, roi_bottom - roi_top)
        roi_area = float(frame_width * roi_height)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = float(cv2.contourArea(contour))
            if area < MIN_LINE_AREA:
                break
            if area > roi_area * MAX_LINE_AREA_RATIO:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if max(width, height) < MIN_LINE_SPAN:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            local_mask = np.zeros((height, width), dtype=np.uint8)
            shifted_contour = contour - np.array([[[x, y]]], dtype=contour.dtype)
            cv2.drawContours(local_mask, [shifted_contour], -1, 255, -1)
            scan_points = []
            for ratio in SCAN_BAND_RATIOS:
                scan_y = int(round(roi_top + ratio * roi_height))
                local_y = scan_y - y
                band_top = max(0, local_y - SCAN_BAND_HALF_HEIGHT)
                band_bottom = min(
                    height, local_y + SCAN_BAND_HALF_HEIGHT + 1
                )
                if band_top >= band_bottom:
                    continue
                _, local_xs = np.nonzero(local_mask[band_top:band_bottom, :])
                if local_xs.size == 0:
                    continue
                scan_x = int(round(float(np.median(local_xs)))) + x
                scan_points.append((scan_x, scan_y))

            points = contour.reshape(-1, 2)
            if scan_points:
                lookahead_x, lookahead_y = scan_points[0]
                anchor_x, anchor_y = scan_points[-1]
            else:
                anchor_y = int(np.max(points[:, 1]))
                anchor_x = int(
                    round(float(np.median(points[points[:, 1] == anchor_y, 0])))
                )
                lookahead_y = int(np.min(points[:, 1]))
                lookahead_x = int(
                    round(
                        float(np.median(points[points[:, 1] == lookahead_y, 0]))
                    )
                )

            span_score = clamp(
                max(width / float(frame_width), height / float(roi_height))
                / 0.45,
                0.0,
                1.0,
            )
            depth_score = clamp(
                (anchor_y - roi_top) / float(roi_height), 0.0, 1.0
            )
            band_score = len(scan_points) / float(len(SCAN_BAND_RATIOS))
            area_score = clamp(area / (roi_area * 0.025), 0.0, 1.0)
            average_thickness = area / float(max(width, height))
            thickness_limit = roi_height * 0.24
            thickness_factor = clamp(
                thickness_limit / max(average_thickness, 1.0), 0.0, 1.0
            )
            quality = thickness_factor * (
                0.24 * span_score
                + 0.30 * depth_score
                + 0.28 * band_score
                + 0.18 * area_score
            )
            candidates.append(
                LineObservation(
                    center_x=int(moments["m10"] / moments["m00"]),
                    center_y=int(moments["m01"] / moments["m00"]),
                    anchor_x=anchor_x,
                    anchor_y=anchor_y,
                    lookahead_x=lookahead_x,
                    lookahead_y=lookahead_y,
                    area=area,
                    box=(x, y, width, height),
                    contour=contour,
                    scan_count=len(scan_points),
                    quality=quality,
                )
            )
            if len(candidates) >= MAX_LINE_CANDIDATES:
                break
        return candidates

    def track_is_active(self, side):
        track = getattr(self, f"{side}_track")
        misses = getattr(self, f"{side}_track_misses")
        return track is not None and misses <= TRACK_MEMORY_FRAMES

    def expected_anchor_x(self, side, frame_width):
        track = getattr(self, f"{side}_track")
        misses = getattr(self, f"{side}_track_misses")
        velocity = getattr(self, f"{side}_anchor_velocity")
        if track is not None and misses <= TRACK_MEMORY_FRAMES:
            prediction_steps = min(misses + 1, 3)
            return track.anchor_x + velocity * prediction_steps

        lane_width = self.estimated_anchor_width
        if lane_width is None:
            lane_width = frame_width * DEFAULT_LANE_WIDTH_RATIO
        other_side = "right" if side == "left" else "left"
        other_track = getattr(self, f"{other_side}_track")
        other_misses = getattr(self, f"{other_side}_track_misses")
        if other_track is not None and other_misses <= TRACK_MEMORY_FRAMES:
            offset = -lane_width if side == "left" else lane_width
            return other_track.anchor_x + offset
        offset = -lane_width / 2.0 if side == "left" else lane_width / 2.0
        return frame_width / 2.0 + offset

    def candidate_side_score(self, candidate, side, frame_width, roi_height):
        active = self.track_is_active(side)
        minimum_quality = (
            MIN_EXISTING_TRACK_QUALITY if active else MIN_NEW_TRACK_QUALITY
        )
        if candidate.quality < minimum_quality:
            return None

        expected_x = self.expected_anchor_x(side, frame_width)
        horizontal_error = abs(candidate.anchor_x - expected_x) / frame_width
        if active and horizontal_error > TRACK_HORIZONTAL_GATE_RATIO:
            return None

        vertical_error = 0.0
        track = getattr(self, f"{side}_track")
        if active and track is not None:
            vertical_error = abs(candidate.anchor_y - track.anchor_y) / roi_height
            if vertical_error > TRACK_VERTICAL_GATE_RATIO:
                return None

        score = (
            2.0 * candidate.quality
            - 1.7 * horizontal_error
            - 0.9 * vertical_error
        )
        if score < MIN_TRACK_MATCH_SCORE:
            return None
        return score

    def select_temporal_candidates(
        self, left_candidates, right_candidates, frame_width, roi_height
    ):
        best_left = None
        best_right = None
        best_score = 0.0
        left_options = [None] + left_candidates
        right_options = [None] + right_candidates
        expected_width = self.estimated_anchor_width
        if expected_width is None:
            expected_width = frame_width * DEFAULT_LANE_WIDTH_RATIO

        for left in left_options:
            for right in right_options:
                if left is not None and left is right:
                    continue
                left_score = 0.0
                right_score = 0.0
                if left is not None:
                    left_score = self.candidate_side_score(
                        left, "left", frame_width, roi_height
                    )
                    if left_score is None:
                        continue
                if right is not None:
                    right_score = self.candidate_side_score(
                        right, "right", frame_width, roi_height
                    )
                    if right_score is None:
                        continue

                pair_score = 0.0
                if left is not None and right is not None:
                    measured_width = right.anchor_x - left.anchor_x
                    if not (
                        frame_width * MIN_LANE_WIDTH_RATIO
                        <= measured_width
                        <= frame_width * MAX_LANE_WIDTH_RATIO
                    ):
                        continue
                    width_error = abs(measured_width - expected_width) / frame_width
                    pair_score = PAIR_SCORE_BONUS - 0.35 * width_error

                total_score = left_score + right_score + pair_score
                if total_score > best_score:
                    best_left = left
                    best_right = right
                    best_score = total_score
        return best_left, best_right

    def update_boundary_track(self, side, observation):
        track_name = f"{side}_track"
        misses_name = f"{side}_track_misses"
        velocity_name = f"{side}_anchor_velocity"
        previous = getattr(self, track_name)
        if observation is None:
            misses = getattr(self, misses_name) + 1
            setattr(self, misses_name, misses)
            if misses > TRACK_MEMORY_FRAMES:
                setattr(self, track_name, None)
                setattr(self, velocity_name, 0.0)
            return

        velocity = getattr(self, velocity_name)
        if previous is not None:
            measured_velocity = observation.anchor_x - previous.anchor_x
            velocity = (
                TRACK_VELOCITY_SMOOTHING * measured_velocity
                + (1.0 - TRACK_VELOCITY_SMOOTHING) * velocity
            )
        else:
            velocity = 0.0
        setattr(self, velocity_name, velocity)
        setattr(self, track_name, observation)
        setattr(self, misses_name, 0)

    @staticmethod
    def tracking_x(observation):
        return (
            (1.0 - LOOKAHEAD_WEIGHT) * observation.center_x
            + LOOKAHEAD_WEIGHT * observation.lookahead_x
        )

    def detect_lane(self, frame):
        blurred = cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        height, width = frame.shape[:2]
        roi_top = int(height * ROI_TOP_RATIO)
        roi_bottom = int(height * ROI_BOTTOM_RATIO)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        roi_mask[roi_top:roi_bottom, :] = 255

        left_mask = self.make_color_mask(hsv, LEFT_LINE_COLOR, roi_mask)
        if LEFT_LINE_COLOR == RIGHT_LINE_COLOR:
            right_mask = left_mask
            candidates = self.find_candidates(
                left_mask, roi_top, roi_bottom
            )
            left, right = self.select_temporal_candidates(
                candidates, candidates, width, roi_bottom - roi_top
            )
            candidate_count = len(candidates)
        else:
            right_mask = self.make_color_mask(hsv, RIGHT_LINE_COLOR, roi_mask)
            left_candidates = self.find_candidates(
                left_mask, roi_top, roi_bottom
            )
            right_candidates = self.find_candidates(
                right_mask, roi_top, roi_bottom
            )
            left, right = self.select_temporal_candidates(
                left_candidates,
                right_candidates,
                width,
                roi_bottom - roi_top,
            )
            candidate_count = len(left_candidates) + len(right_candidates)

        raw_mask = cv2.bitwise_or(left_mask, right_mask)
        self.update_boundary_track("left", left)
        self.update_boundary_track("right", right)

        lane_width = self.estimated_lane_width
        if lane_width is None:
            lane_width = width * DEFAULT_LANE_WIDTH_RATIO

        if left is not None and right is not None:
            measured_width = abs(
                self.tracking_x(right) - self.tracking_x(left)
            )
            measured_anchor_width = float(right.anchor_x - left.anchor_x)
            if self.estimated_lane_width is None:
                self.estimated_lane_width = measured_width
                self.estimated_anchor_width = measured_anchor_width
            else:
                self.estimated_lane_width = (
                    LANE_WIDTH_SMOOTHING * measured_width
                    + (1.0 - LANE_WIDTH_SMOOTHING) * self.estimated_lane_width
                )
                self.estimated_anchor_width = (
                    LANE_WIDTH_SMOOTHING * measured_anchor_width
                    + (1.0 - LANE_WIDTH_SMOOTHING)
                    * self.estimated_anchor_width
                )
            lane_width = self.estimated_lane_width
            center_x = (
                self.tracking_x(left) + self.tracking_x(right)
            ) / 2.0
            self.control_state = "BOTH"
        elif left is not None:
            center_x = self.tracking_x(left) + lane_width / 2.0
            self.control_state = "LEFT_ONLY"
        elif right is not None:
            center_x = self.tracking_x(right) - lane_width / 2.0
            self.control_state = "RIGHT_ONLY"
        else:
            center_x = None

        # The preview and all downstream logic use only the selected boundary
        # contours. This removes HSV noise without narrowing the yellow range.
        combined_mask = np.zeros((height, width), dtype=np.uint8)
        selected_contours = []
        if left is not None:
            selected_contours.append(left.contour)
        if right is not None and right is not left:
            selected_contours.append(right.contour)
        if selected_contours:
            cv2.drawContours(combined_mask, selected_contours, -1, 255, -1)

        return LaneDetection(
            left=left,
            right=right,
            center_x=center_x,
            mask=combined_mask,
            raw_mask=raw_mask,
            candidate_count=candidate_count,
            roi_top=roi_top,
            roi_bottom=roi_bottom,
        )

    def calculate_command(self, detection, frame_width):
        now = time.monotonic()
        if detection.center_x is None:
            recovery_age = now - self.last_lane_seen_time
            if (
                self.last_lane_seen_time > 0.0
                and recovery_age <= LOST_LANE_RECOVERY_SECONDS
                and abs(self.filtered_steering) >= RECOVERY_TRIGGER_STEERING
            ):
                direction = 1.0 if self.filtered_steering > 0.0 else -1.0
                recovery_steering = direction * max(
                    abs(self.filtered_steering), RECOVERY_MIN_STEERING
                )
                self.filtered_steering = clamp(
                    recovery_steering, -STEERING_LIMIT, STEERING_LIMIT
                )
                self.control_state = "RECOVERY"
                self.path_state = "RECOVERY"
                return TURN_DRIVE_PWM, self.filtered_steering
            self.filtered_steering = 0.0
            self.control_state = "LOST"
            self.path_state = "LOST"
            return STOP_PWM, 0.0

        self.last_lane_seen_time = now
        target_x = frame_width * (0.5 + CENTER_OFFSET_RATIO)
        error = (detection.center_x - target_x) / (frame_width / 2.0)
        if abs(error) < CENTER_DEADBAND:
            error = 0.0
        desired_steering = clamp(
            STEERING_DIRECTION * STEERING_GAIN * error,
            -STEERING_LIMIT,
            STEERING_LIMIT,
        )
        self.filtered_steering = (
            STEERING_SMOOTHING * desired_steering
            + (1.0 - STEERING_SMOOTHING) * self.filtered_steering
        )
        turn_demand = max(abs(desired_steering), abs(self.filtered_steering))
        self.path_state = (
            "CURVE" if turn_demand >= TURN_SPEED_THRESHOLD else "STRAIGHT"
        )
        speed_pwm = (
            TURN_DRIVE_PWM
            if turn_demand >= TURN_SPEED_THRESHOLD
            else STRAIGHT_DRIVE_PWM
        )
        return speed_pwm, self.filtered_steering

    def publish_command(self, speed_pwm, steering, force=False):
        now = time.monotonic()
        if not force and now - self.last_publish_time < 1.0 / CONTROL_RATE:
            return
        linear_command = (speed_pwm - STOP_PWM) / DRIVER_SPEED_SCALE
        message = Twist()
        message.linear.x = float(linear_command)
        message.angular.z = float(steering)
        self.publisher.publish(message)
        self.last_publish_time = now
        self.last_speed_pwm = speed_pwm
        self.last_steering = steering

        if now - self.last_log_time >= 1.0 / TERMINAL_LOG_RATE:
            motion = "MOVE" if speed_pwm > STOP_PWM else "STOP"
            steering_pwm = 800.0 + steering * 400.0
            print(
                f"{motion} mode={self.control_state} "
                f"path={self.path_state} "
                f"speed_pwm={speed_pwm:.0f} linear.x={linear_command:.3f} "
                f"angular.z={steering:+.3f} steering_pwm={steering_pwm:.0f}",
                flush=True,
            )
            self.last_log_time = now

    def stop_vehicle(self):
        self.filtered_steering = 0.0
        self.control_state = "STOPPED"
        self.path_state = "STOPPED"
        for _ in range(5):
            self.publish_command(STOP_PWM, 0.0, force=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def open_camera():
    capture = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(CAMERA_INDEX)
    if not capture.isOpened():
        return None
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def make_diagnostic_video_path():
    directory = Path(DIAGNOSTIC_VIDEO_DIRECTORY).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return directory / (
        f"{DIAGNOSTIC_VIDEO_PREFIX}_{timestamp}{DIAGNOSTIC_VIDEO_EXTENSION}"
    )


def draw_line_observation(preview, observation, color):
    if observation is None:
        return
    x, y, width, height = observation.box
    cv2.rectangle(preview, (x, y), (x + width, y + height), color, 2)
    cv2.circle(
        preview,
        (observation.center_x, observation.center_y),
        6,
        color,
        -1,
    )
    cv2.circle(
        preview,
        (observation.anchor_x, observation.anchor_y),
        5,
        (255, 255, 0),
        -1,
    )
    cv2.circle(
        preview,
        (observation.lookahead_x, observation.lookahead_y),
        5,
        (255, 0, 255),
        -1,
    )
    cv2.line(
        preview,
        (observation.anchor_x, observation.anchor_y),
        (observation.lookahead_x, observation.lookahead_y),
        (255, 255, 0),
        1,
    )


def draw_overlay(
    preview, follower, detection, measured_fps, display_mode=None
):
    height, width = preview.shape[:2]
    cv2.rectangle(
        preview,
        (0, detection.roi_top),
        (width - 1, detection.roi_bottom),
        (0, 255, 255),
        1,
    )
    target_x = int(width * (0.5 + CENTER_OFFSET_RATIO))
    cv2.line(
        preview,
        (target_x, detection.roi_top),
        (target_x, detection.roi_bottom),
        (0, 255, 0),
        1,
    )
    draw_line_observation(preview, detection.left, (255, 0, 0))
    draw_line_observation(preview, detection.right, (0, 0, 255))
    if detection.center_x is not None:
        center_x = int(round(detection.center_x))
        center_y = int((detection.roi_top + detection.roi_bottom) / 2)
        cv2.line(
            preview,
            (center_x, detection.roi_top),
            (center_x, detection.roi_bottom),
            (0, 255, 255),
            2,
        )
        cv2.circle(preview, (center_x, center_y), 7, (0, 255, 255), -1)

    if display_mode is None:
        display_mode = "CAMERA" if SHOW_CAMERA_IMAGE else "MASK"
    left_quality = detection.left.quality if detection.left is not None else 0.0
    right_quality = detection.right.quality if detection.right is not None else 0.0
    lines = (
        f"ACTIVE left={LEFT_LINE_COLOR.upper()} right={RIGHT_LINE_COLOR.upper()} "
        f"mode={display_mode} lane={follower.control_state} "
        f"path={follower.path_state}",
        f"PWM={follower.last_speed_pwm:.0f} "
        f"turn={follower.last_steering:+.3f} FPS={measured_fps:.1f}",
        f"candidates={detection.candidate_count} "
        f"quality L={left_quality:.2f} R={right_quality:.2f}",
        "Dual-line following - Q/Esc: stop and quit",
    )
    for index, text in enumerate(lines):
        position = (10, 26 + index * 25)
        cv2.putText(
            preview,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        color = (0, 255, 0) if index < 2 else (255, 255, 255)
        cv2.putText(
            preview,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )


def make_diagnostic_frame(frame, follower, detection, measured_fps):
    diagnostic = frame.copy()
    draw_overlay(
        diagnostic, follower, detection, measured_fps, display_mode="CAMERA"
    )

    mask_inset = cv2.cvtColor(detection.raw_mask, cv2.COLOR_GRAY2BGR)
    mask_inset[detection.mask > 0] = (0, 255, 0)
    inset_width = int(round(frame.shape[1] * DIAGNOSTIC_MASK_INSET_SCALE))
    inset_height = int(round(frame.shape[0] * DIAGNOSTIC_MASK_INSET_SCALE))
    mask_inset = cv2.resize(
        mask_inset,
        (inset_width, inset_height),
        interpolation=cv2.INTER_NEAREST,
    )
    margin = 8
    inset_x = diagnostic.shape[1] - inset_width - margin
    inset_y = diagnostic.shape[0] - inset_height - margin
    cv2.rectangle(
        diagnostic,
        (inset_x - 2, inset_y - 2),
        (inset_x + inset_width + 1, inset_y + inset_height + 1),
        (255, 255, 255),
        2,
    )
    diagnostic[
        inset_y : inset_y + inset_height,
        inset_x : inset_x + inset_width,
    ] = mask_inset
    cv2.putText(
        diagnostic,
        "RAW MASK / GREEN=TRACK",
        (inset_x + 4, inset_y + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    if DIAGNOSTIC_VIDEO_SCALE != 1.0:
        output_width = int(round(diagnostic.shape[1] * DIAGNOSTIC_VIDEO_SCALE))
        output_height = int(round(diagnostic.shape[0] * DIAGNOSTIC_VIDEO_SCALE))
        diagnostic = cv2.resize(
            diagnostic,
            (output_width, output_height),
            interpolation=cv2.INTER_AREA,
        )
    return diagnostic


def main():
    errors = validate_configuration()
    if errors:
        for error in errors:
            print(f"configuration error: {error}", file=sys.stderr)
        return 2
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        print("warning: DISPLAY is not set; enable SSH X11 forwarding")

    rclpy.init()
    follower = DualColorLineFollower()
    capture = open_camera()
    if capture is None:
        print(f"camera open failed: /dev/video{CAMERA_INDEX}", file=sys.stderr)
        follower.stop_vehicle()
        follower.destroy_node()
        rclpy.shutdown()
        return 1

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    video_recorder = None
    diagnostic_video_path = None
    if RECORD_DIAGNOSTIC_VIDEO:
        diagnostic_video_path = make_diagnostic_video_path()
        video_width = int(round(actual_width * DIAGNOSTIC_VIDEO_SCALE))
        video_height = int(round(actual_height * DIAGNOSTIC_VIDEO_SCALE))
        video_writer = cv2.VideoWriter(
            str(diagnostic_video_path),
            cv2.VideoWriter_fourcc(*DIAGNOSTIC_VIDEO_CODEC),
            DIAGNOSTIC_VIDEO_FPS,
            (video_width, video_height),
        )
        if not video_writer.isOpened():
            video_writer.release()
            capture.release()
            print(
                f"diagnostic video open failed: {diagnostic_video_path}",
                file=sys.stderr,
            )
            follower.stop_vehicle()
            follower.destroy_node()
            rclpy.shutdown()
            return 1
        video_recorder = AsyncVideoRecorder(video_writer)
        video_recorder.start()
    mode = "camera" if SHOW_CAMERA_IMAGE else "mask"
    print(f"camera opened: /dev/video{CAMERA_INDEX} ({actual_width}x{actual_height})")
    print(
        f"following left={LEFT_LINE_COLOR}, right={RIGHT_LINE_COLOR}; "
        f"preview={mode}; topic={CONTROL_TOPIC}"
    )
    print("autonomous dual-line following is ACTIVE; Q or Esc stops and exits")
    if diagnostic_video_path is not None:
        print(f"recording diagnostic video: {diagnostic_video_path}")

    reader = LatestFrameReader(capture)
    reader.start()
    window_name = "Racecar Dual Color Line Follower"
    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    next_record_time = 0.0
    measured_fps = 0.0
    window_has_frame = False

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        first_frame_deadline = time.monotonic() + 5.0
        while rclpy.ok():
            rclpy.spin_once(follower, timeout_sec=0.0)
            sequence, frame, last_frame_time = reader.latest()
            now = time.monotonic()

            if frame is None:
                if now >= first_frame_deadline:
                    print("read frame failed", file=sys.stderr)
                    return 1
            elif now - last_frame_time > 2.0:
                print("camera stream stopped", file=sys.stderr)
                return 1
            elif sequence != last_sequence:
                detection = follower.detect_lane(frame)
                speed_pwm, steering = follower.calculate_command(
                    detection, frame.shape[1]
                )
                follower.publish_command(speed_pwm, steering)
                last_sequence = sequence

                if video_recorder is not None and now >= next_record_time:
                    diagnostic_frame = make_diagnostic_frame(
                        frame, follower, detection, measured_fps
                    )
                    video_recorder.submit(diagnostic_frame)
                    next_record_time = now + 1.0 / DIAGNOSTIC_VIDEO_FPS

                if now >= next_display_time:
                    if last_display_time > 0.0:
                        instant_fps = 1.0 / max(now - last_display_time, 1e-6)
                        measured_fps = (
                            instant_fps
                            if measured_fps == 0.0
                            else measured_fps * 0.85 + instant_fps * 0.15
                        )
                    last_display_time = now
                    if SHOW_CAMERA_IMAGE:
                        preview = frame.copy()
                    else:
                        preview = cv2.cvtColor(
                            detection.mask, cv2.COLOR_GRAY2BGR
                        )
                    draw_overlay(preview, follower, detection, measured_fps)
                    if DISPLAY_SCALE != 1.0:
                        preview = cv2.resize(
                            preview,
                            None,
                            fx=DISPLAY_SCALE,
                            fy=DISPLAY_SCALE,
                            interpolation=cv2.INTER_AREA,
                        )
                    cv2.imshow(window_name, preview)
                    next_display_time = now + 1.0 / DISPLAY_FPS
                    window_has_frame = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if (
                window_has_frame
                and cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
            ):
                break
            time.sleep(0.001)
    except cv2.error as exc:
        print(f"OpenCV X11 window error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        follower.stop_vehicle()
        reader.stop()
        if video_recorder is not None:
            video_recorder.stop()
        cv2.destroyAllWindows()
        follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if diagnostic_video_path is not None:
        recorded_frames = (
            video_recorder.written_frames if video_recorder is not None else 0
        )
        print(
            f"diagnostic video saved: {diagnostic_video_path} "
            f"({recorded_frames} frames)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
