#!/usr/bin/env python3

"""识别红绿灯和白色矩形人行道，并发布统一的道路感知状态。"""

import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# ======================== User configuration ========================

CAMERA_INDEX = 20
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30.0
DISPLAY_SCALE = 0.5
DISPLAY_FPS = 15.0
SHOW_CAMERA_IMAGE = True

# 红绿灯和人行道通过同一个 JSON 话题发布：
# {"traffic_light":"RED|GREEN|UNKNOWN", "crosswalk":true|false}
PERCEPTION_TOPIC = "/road_perception"
PUBLISH_INTERVAL = 0.20

# Only the middle band containing the traffic-light box is analyzed.  The
# whiteboard decorations above and floor reflections below are ignored.
ROI_TOP_RATIO = 0.30
ROI_BOTTOM_RATIO = 0.78
ROI_LEFT_RATIO = 0.05
ROI_RIGHT_RATIO = 0.95

# Relaxed masks keep the colored halo around an overexposed white lamp center.
COLOR_HSV_RANGES = {
    "red": [
        ((0, 45, 70), (15, 255, 255)),
        ((163, 45, 70), (179, 255, 255)),
    ],
    "green": [
        ((32, 25, 70), (105, 255, 255)),
    ],
}

BLUR_KERNEL = 5
MORPH_KERNEL = 3
MIN_COMPONENT_AREA = 40

# Only brightness above this value contributes to the score.  The score is
# the equivalent number of fully bright colored pixels in the strongest blob.
SCORE_VALUE_FLOOR = 150
MIN_ACTIVE_SCORE = 45.0
CONFIRM_FRAMES = 2

# Visualization-only localization. Red is on the left and green is on the
# right. The winning color component is used directly for the rectangle.
LOCALIZE_TOP_RATIO = 0.34
LOCALIZE_BOTTOM_RATIO = 0.72
MARKER_PADDING = 4
LOCALIZE_WHITE_VALUE = 220
LOCALIZE_WHITE_MAX_SATURATION = 100
LOCALIZE_COLOR_DILATE_KERNEL = 61
MIN_WHITE_CORE_AREA = 80
MAX_WHITE_CORE_AREA = 10000

# 人行道只会出现一次，并且靠近车辆时会在画面下半部形成大面积白色区域。
# 不再限定纸条方向和矩形数量，避免透视下的竖向白纸条被过滤掉。
CROSSWALK_ROI_TOP_RATIO = 0.55
CROSSWALK_ROI_BOTTOM_RATIO = 1.00
CROSSWALK_ROI_LEFT_RATIO = 0.02
CROSSWALK_ROI_RIGHT_RATIO = 0.98
CROSSWALK_WHITE_MIN_VALUE = 190
CROSSWALK_WHITE_MAX_SATURATION = 70
CROSSWALK_MIN_WHITE_AREA_RATIO = 0.08
CROSSWALK_MIN_VALID_AREA_RATIO = 0.055
CROSSWALK_MIN_BLOB_AREA_RATIO = 0.012
CROSSWALK_MIN_BLOB_FILL_RATIO = 0.62
# 真实白纸受透视后大致向画面纵深延伸；横向黄线和细反光不满足该比例。
CROSSWALK_MIN_HEIGHT_WIDTH_RATIO = 0.70
CROSSWALK_MAX_HEIGHT_WIDTH_RATIO = 4.50
CROSSWALK_MIN_BLOBS = 2
CROSSWALK_MIN_CENTER_SPAN_RATIO = 0.16
CROSSWALK_MIN_POLYGON_VERTICES = 4
CROSSWALK_MAX_POLYGON_VERTICES = 8
# 实车预览只有约 0.6 FPS，大面积白色条件已经足够严格，因此单帧立即触发。
CROSSWALK_CONFIRM_FRAMES = 1

# ===================================================================


def validate_configuration():
    errors = []
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0 or CAMERA_FPS <= 0:
        errors.append("camera width, height and fps must be greater than zero")
    if DISPLAY_SCALE <= 0 or DISPLAY_FPS <= 0:
        errors.append("display scale and fps must be greater than zero")
    if not 0.0 <= ROI_TOP_RATIO < ROI_BOTTOM_RATIO <= 1.0:
        errors.append("vertical ROI ratios are invalid")
    if not 0.0 <= ROI_LEFT_RATIO < ROI_RIGHT_RATIO <= 1.0:
        errors.append("horizontal ROI ratios are invalid")
    for name, kernel in (
        ("BLUR_KERNEL", BLUR_KERNEL),
        ("MORPH_KERNEL", MORPH_KERNEL),
    ):
        if kernel < 1 or kernel % 2 == 0:
            errors.append(f"{name} must be a positive odd integer")
    if MIN_COMPONENT_AREA < 1:
        errors.append("MIN_COMPONENT_AREA must be at least 1")
    if not 0 <= SCORE_VALUE_FLOOR < 255:
        errors.append("SCORE_VALUE_FLOOR must be in [0, 255)")
    if MIN_ACTIVE_SCORE < 0:
        errors.append("MIN_ACTIVE_SCORE must be >= 0")
    if CONFIRM_FRAMES < 1:
        errors.append("CONFIRM_FRAMES must be at least 1")
    if not 0.0 <= LOCALIZE_TOP_RATIO < LOCALIZE_BOTTOM_RATIO <= 1.0:
        errors.append("localization ratios are invalid")
    if MARKER_PADDING < 0:
        errors.append("MARKER_PADDING must be >= 0")
    if not 0 <= LOCALIZE_WHITE_VALUE <= 255:
        errors.append("LOCALIZE_WHITE_VALUE must be between 0 and 255")
    if not 0 <= LOCALIZE_WHITE_MAX_SATURATION <= 255:
        errors.append("LOCALIZE_WHITE_MAX_SATURATION must be between 0 and 255")
    if (
        LOCALIZE_COLOR_DILATE_KERNEL < 1
        or LOCALIZE_COLOR_DILATE_KERNEL % 2 == 0
    ):
        errors.append("LOCALIZE_COLOR_DILATE_KERNEL must be a positive odd integer")
    if MIN_WHITE_CORE_AREA < 1 or MAX_WHITE_CORE_AREA < MIN_WHITE_CORE_AREA:
        errors.append("white-core area limits are invalid")
    if not 0.0 <= CROSSWALK_ROI_TOP_RATIO < CROSSWALK_ROI_BOTTOM_RATIO <= 1.0:
        errors.append("crosswalk vertical ROI ratios are invalid")
    if not 0.0 <= CROSSWALK_ROI_LEFT_RATIO < CROSSWALK_ROI_RIGHT_RATIO <= 1.0:
        errors.append("crosswalk horizontal ROI ratios are invalid")
    if not 0 <= CROSSWALK_WHITE_MAX_SATURATION <= 255:
        errors.append("CROSSWALK_WHITE_MAX_SATURATION must be in [0, 255]")
    if not 0 <= CROSSWALK_WHITE_MIN_VALUE <= 255:
        errors.append("CROSSWALK_WHITE_MIN_VALUE must be in [0, 255]")
    if not 0.0 < CROSSWALK_MIN_WHITE_AREA_RATIO <= 1.0:
        errors.append("CROSSWALK_MIN_WHITE_AREA_RATIO must be in (0, 1]")
    for name, ratio in (
        ("CROSSWALK_MIN_VALID_AREA_RATIO", CROSSWALK_MIN_VALID_AREA_RATIO),
        ("CROSSWALK_MIN_BLOB_AREA_RATIO", CROSSWALK_MIN_BLOB_AREA_RATIO),
        ("CROSSWALK_MIN_BLOB_FILL_RATIO", CROSSWALK_MIN_BLOB_FILL_RATIO),
        ("CROSSWALK_MIN_CENTER_SPAN_RATIO", CROSSWALK_MIN_CENTER_SPAN_RATIO),
    ):
        if not 0.0 < ratio <= 1.0:
            errors.append(f"{name} must be in (0, 1]")
    if not 0.0 < CROSSWALK_MIN_HEIGHT_WIDTH_RATIO < CROSSWALK_MAX_HEIGHT_WIDTH_RATIO:
        errors.append("crosswalk height/width ratio limits are invalid")
    if CROSSWALK_MIN_BLOBS < 2:
        errors.append("CROSSWALK_MIN_BLOBS must be at least 2")
    if not 3 <= CROSSWALK_MIN_POLYGON_VERTICES <= CROSSWALK_MAX_POLYGON_VERTICES:
        errors.append("crosswalk polygon vertex limits are invalid")
    if CROSSWALK_CONFIRM_FRAMES < 1:
        errors.append("CROSSWALK_CONFIRM_FRAMES must be positive")
    if PUBLISH_INTERVAL <= 0.0:
        errors.append("PUBLISH_INTERVAL must be greater than zero")
    return errors


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


class TrafficLightDetector:
    def __init__(self):
        self.stable_state = "UNKNOWN"
        self.pending_state = "UNKNOWN"
        self.pending_count = 0
        self.crosswalk_stable = False
        self.crosswalk_pending = False
        self.crosswalk_pending_count = 0
        self.crosswalk_detection_finished = False
        self.crosswalk_rectangles = []
        self.crosswalk_mask = None
        self.crosswalk_white_area_ratio = 0.0
        self.crosswalk_valid_area_ratio = 0.0
        self.red_score = 0.0
        self.green_score = 0.0
        self.active_marker = None
        self.active_mask = None

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
    def strongest_blob_score(mask, value_channel):
        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        best_score = 0.0
        value_span = float(255 - SCORE_VALUE_FLOOR)
        for label in range(1, component_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < MIN_COMPONENT_AREA:
                continue
            pixels = value_channel[labels == label].astype(np.float32)
            brightness = np.clip(
                (pixels - SCORE_VALUE_FLOOR) / value_span, 0.0, 1.0
            )
            score = float(np.sum(brightness))
            best_score = max(best_score, score)
        return best_score

    @staticmethod
    def locate_active_lamp(
        color_mask, saturation_channel, value_channel, state
    ):
        if state not in ("RED", "GREEN"):
            return None, np.zeros_like(color_mask)
        height, width = value_channel.shape
        top = int(height * LOCALIZE_TOP_RATIO)
        bottom = int(height * LOCALIZE_BOTTOM_RATIO)
        middle = width // 2
        left = 0 if state == "RED" else middle
        right = middle if state == "RED" else width

        search_mask = np.zeros_like(color_mask)
        search_mask[top:bottom, left:right] = 255
        white_mask = cv2.inRange(
            saturation_channel, 0, LOCALIZE_WHITE_MAX_SATURATION
        )
        white_mask = cv2.bitwise_and(
            white_mask,
            cv2.inRange(value_channel, LOCALIZE_WHITE_VALUE, 255),
        )
        color_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (LOCALIZE_COLOR_DILATE_KERNEL, LOCALIZE_COLOR_DILATE_KERNEL),
        )
        near_color_mask = cv2.dilate(color_mask, color_kernel)
        marker_mask = cv2.bitwise_and(white_mask, near_color_mask)
        marker_mask = cv2.bitwise_and(marker_mask, search_mask)
        cleanup_kernel = np.ones((5, 5), dtype=np.uint8)
        marker_mask = cv2.morphologyEx(
            marker_mask, cv2.MORPH_OPEN, cleanup_kernel
        )
        marker_mask = cv2.morphologyEx(
            marker_mask, cv2.MORPH_CLOSE, cleanup_kernel
        )

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            marker_mask, connectivity=8
        )
        best_marker = None
        best_component_mask = np.zeros_like(color_mask)
        best_area = 0
        for label in range(1, component_count):
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            box_width = int(stats[label, cv2.CC_STAT_WIDTH])
            box_height = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])
            if not MIN_WHITE_CORE_AREA <= area <= MAX_WHITE_CORE_AREA:
                continue
            aspect_ratio = box_width / float(max(box_height, 1))
            if not 0.50 <= aspect_ratio <= 2.0:
                continue
            if box_width > width * 0.25 or box_height > height * 0.30:
                continue
            if area <= best_area:
                continue
            x = max(0, x - MARKER_PADDING)
            y = max(0, y - MARKER_PADDING)
            box_width = min(width - x, box_width + MARKER_PADDING * 2)
            box_height = min(height - y, box_height + MARKER_PADDING * 2)
            best_marker = (
                x,
                y,
                box_width,
                box_height,
            )
            best_component_mask.fill(0)
            best_component_mask[labels == label] = 255
            best_area = area
        return best_marker, best_component_mask

    @staticmethod
    def detect_crosswalk(hsv):
        """通过画面下半部的大面积白色覆盖识别人行道。"""
        height, width = hsv.shape[:2]
        top = int(height * CROSSWALK_ROI_TOP_RATIO)
        bottom = int(height * CROSSWALK_ROI_BOTTOM_RATIO)
        left = int(width * CROSSWALK_ROI_LEFT_RATIO)
        right = int(width * CROSSWALK_ROI_RIGHT_RATIO)

        roi_mask = np.zeros((height, width), dtype=np.uint8)
        roi_mask[top:bottom, left:right] = 255
        white_mask = cv2.inRange(
            hsv,
            np.array((0, 0, CROSSWALK_WHITE_MIN_VALUE), dtype=np.uint8),
            np.array(
                (179, CROSSWALK_WHITE_MAX_SATURATION, 255), dtype=np.uint8
            ),
        )
        white_mask = cv2.bitwise_and(white_mask, roi_mask)
        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
        white_mask = cv2.morphologyEx(
            white_mask, cv2.MORPH_OPEN, open_kernel
        )
        white_mask = cv2.morphologyEx(
            white_mask, cv2.MORPH_CLOSE, close_kernel
        )

        contours, _ = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        roi_area = float(max((bottom - top) * (right - left), 1))
        white_area_ratio = cv2.countNonZero(white_mask) / roi_area
        valid_rectangles = []
        valid_blob_area = 0.0
        valid_center_x_values = []
        minimum_blob_area = roi_area * CROSSWALK_MIN_BLOB_AREA_RATIO
        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area < minimum_blob_area:
                continue

            rotated_rect = cv2.minAreaRect(contour)
            rect_width, rect_height = rotated_rect[1]
            rotated_rect_area = rect_width * rect_height
            if rotated_rect_area <= 0.0:
                continue
            fill_ratio = contour_area / rotated_rect_area
            if fill_ratio < CROSSWALK_MIN_BLOB_FILL_RATIO:
                continue

            perimeter = cv2.arcLength(contour, True)
            polygon = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
            if not (
                CROSSWALK_MIN_POLYGON_VERTICES
                <= len(polygon)
                <= CROSSWALK_MAX_POLYGON_VERTICES
            ):
                continue

            x, y, box_width, box_height = cv2.boundingRect(contour)
            height_width_ratio = box_height / float(max(box_width, 1))
            if not (
                CROSSWALK_MIN_HEIGHT_WIDTH_RATIO
                <= height_width_ratio
                <= CROSSWALK_MAX_HEIGHT_WIDTH_RATIO
            ):
                continue

            center_x = x + box_width * 0.5
            center_y = y + box_height * 0.5
            valid_rectangles.append(
                (
                    x,
                    y,
                    box_width,
                    box_height,
                    center_x,
                    center_y,
                )
            )
            valid_blob_area += contour_area
            valid_center_x_values.append(center_x)

        valid_area_ratio = valid_blob_area / roi_area
        center_span_ratio = 0.0
        if len(valid_center_x_values) >= 2:
            center_span_ratio = (
                max(valid_center_x_values) - min(valid_center_x_values)
            ) / float(max(right - left, 1))
        detected = (
            white_area_ratio >= CROSSWALK_MIN_WHITE_AREA_RATIO
            and valid_area_ratio >= CROSSWALK_MIN_VALID_AREA_RATIO
            and len(valid_rectangles) >= CROSSWALK_MIN_BLOBS
            and center_span_ratio >= CROSSWALK_MIN_CENTER_SPAN_RATIO
        )
        return (
            detected,
            white_mask,
            valid_rectangles,
            white_area_ratio,
            valid_area_ratio,
        )

    def analyze(self, frame):
        blurred = cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        saturation_channel = hsv[:, :, 1]
        value_channel = hsv[:, :, 2]
        height, width = frame.shape[:2]
        top = int(height * ROI_TOP_RATIO)
        bottom = int(height * ROI_BOTTOM_RATIO)
        left = int(width * ROI_LEFT_RATIO)
        right = int(width * ROI_RIGHT_RATIO)
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        roi_mask[top:bottom, left:right] = 255

        masks = {
            color: self.make_color_mask(hsv, color, roi_mask)
            for color in ("red", "green")
        }
        self.red_score = self.strongest_blob_score(masks["red"], value_channel)
        self.green_score = self.strongest_blob_score(
            masks["green"], value_channel
        )

        highest_score = max(self.red_score, self.green_score)
        if highest_score < MIN_ACTIVE_SCORE:
            raw_state = "UNKNOWN"
        elif self.green_score > self.red_score:
            raw_state = "GREEN"
        else:
            raw_state = "RED"
        winning_mask = masks[raw_state.lower()] if raw_state != "UNKNOWN" else None
        if winning_mask is None:
            self.active_marker = None
            self.active_mask = np.zeros_like(value_channel)
        else:
            self.active_marker, self.active_mask = self.locate_active_lamp(
                winning_mask,
                saturation_channel,
                value_channel,
                raw_state,
            )
        if self.crosswalk_detection_finished:
            # 本场比赛的人行道已经触发过，不再重复运行人行道图像检测。
            raw_crosswalk = True
        else:
            (
                raw_crosswalk,
                self.crosswalk_mask,
                self.crosswalk_rectangles,
                self.crosswalk_white_area_ratio,
                self.crosswalk_valid_area_ratio,
            ) = self.detect_crosswalk(hsv)
        return raw_state, raw_crosswalk

    def update_state(self, raw_state, raw_crosswalk):
        changed = False
        if raw_state == self.pending_state:
            self.pending_count += 1
        else:
            self.pending_state = raw_state
            self.pending_count = 1
        if self.pending_count >= CONFIRM_FRAMES and raw_state != self.stable_state:
            self.stable_state = raw_state
            changed = True
            print(
                f"traffic light: {self.stable_state} "
                f"red_score={self.red_score:.1f} "
                f"green_score={self.green_score:.1f}",
                flush=True,
            )

        if raw_crosswalk == self.crosswalk_pending:
            self.crosswalk_pending_count += 1
        else:
            self.crosswalk_pending = raw_crosswalk
            self.crosswalk_pending_count = 1
        if (
            raw_crosswalk
            and self.crosswalk_pending_count >= CROSSWALK_CONFIRM_FRAMES
            and not self.crosswalk_stable
        ):
            self.crosswalk_stable = True
            self.crosswalk_detection_finished = True
            changed = True
            print(
                "crosswalk: DETECTED (one-shot locked) "
                f"white_ratio={self.crosswalk_white_area_ratio:.3f} "
                f"valid_ratio={self.crosswalk_valid_area_ratio:.3f} "
                f"papers={len(self.crosswalk_rectangles)}",
                flush=True,
            )
        return changed


class RoadPerceptionPublisher(Node):
    """把红绿灯和人行道状态发布到同一个 ROS2 话题。"""

    def __init__(self):
        super().__init__("traffic_light_and_crosswalk_detector")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, PERCEPTION_TOPIC, qos)
        self.last_publish_time = 0.0
        self.last_payload = ""

    def publish_state(self, detector, force=False):
        now = time.monotonic()
        payload = json.dumps(
            {
                "traffic_light": detector.stable_state,
                "crosswalk": detector.crosswalk_stable,
            },
            separators=(",", ":"),
        )
        if (
            not force
            and payload == self.last_payload
            and now - self.last_publish_time < PUBLISH_INTERVAL
        ):
            return

        message = String()
        message.data = payload
        self.publisher.publish(message)
        self.last_payload = payload
        self.last_publish_time = now


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


def make_preview(frame, detector):
    if SHOW_CAMERA_IMAGE:
        return frame.copy()
    if detector.active_mask is None:
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    else:
        mask = detector.active_mask.copy()
    if detector.crosswalk_mask is not None:
        mask = cv2.bitwise_or(mask, detector.crosswalk_mask)
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def draw_overlay(preview, detector, raw_state, raw_crosswalk, measured_fps):
    if detector.active_marker is not None:
        x, y, width, height = detector.active_marker
        marker_color = (0, 0, 255) if raw_state == "RED" else (0, 255, 0)
        cv2.rectangle(
            preview,
            (x, y),
            (x + width, y + height),
            marker_color,
            2,
        )
    crosswalk_color = (0, 255, 255) if raw_crosswalk else (0, 140, 255)
    for x, y, width, height, _, _ in detector.crosswalk_rectangles:
        cv2.rectangle(
            preview,
            (x, y),
            (x + width, y + height),
            crosswalk_color,
            2,
        )
    lines = (
        f"STATE={detector.stable_state} raw={raw_state} FPS={measured_fps:.1f}",
        f"red_score={detector.red_score:.1f} green_score={detector.green_score:.1f}",
        "crosswalk="
        f"{detector.crosswalk_stable} raw={raw_crosswalk} "
        f"white={detector.crosswalk_white_area_ratio:.3f} "
        f"valid={detector.crosswalk_valid_area_ratio:.3f} "
        f"papers={len(detector.crosswalk_rectangles)}",
        "crosswalk detector="
        f"{'LOCKED' if detector.crosswalk_detection_finished else 'ACTIVE'}",
        "Q/Esc: quit",
    )
    for index, line in enumerate(lines):
        y = 28 + index * 26
        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def main():
    errors = validate_configuration()
    if errors:
        for error in errors:
            print(f"configuration error: {error}", file=sys.stderr)
        return 2
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        print("warning: DISPLAY is not set; enable SSH X11 forwarding")

    rclpy.init()
    perception_publisher = RoadPerceptionPublisher()
    detector = TrafficLightDetector()
    capture = open_camera()
    if capture is None:
        print(f"camera open failed: /dev/video{CAMERA_INDEX}", file=sys.stderr)
        perception_publisher.destroy_node()
        rclpy.shutdown()
        return 1

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera opened: /dev/video{CAMERA_INDEX} ({actual_width}x{actual_height})")
    print(f"publishing road perception: {PERCEPTION_TOPIC}")
    perception_publisher.publish_state(detector, force=True)

    reader = LatestFrameReader(capture)
    reader.start()
    window_name = "Racecar Traffic Light Detector"
    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    measured_fps = 0.0
    window_has_frame = False

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        first_frame_deadline = time.monotonic() + 5.0
        while rclpy.ok():
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
                raw_state, raw_crosswalk = detector.analyze(frame)
                state_changed = detector.update_state(
                    raw_state, raw_crosswalk
                )
                perception_publisher.publish_state(
                    detector, force=state_changed
                )
                last_sequence = sequence

                if now >= next_display_time:
                    if last_display_time > 0.0:
                        instant_fps = 1.0 / max(now - last_display_time, 1e-6)
                        measured_fps = (
                            instant_fps
                            if measured_fps == 0.0
                            else measured_fps * 0.85 + instant_fps * 0.15
                        )
                    last_display_time = now
                    preview = make_preview(frame, detector)
                    draw_overlay(
                        preview,
                        detector,
                        raw_state,
                        raw_crosswalk,
                        measured_fps,
                    )
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

            rclpy.spin_once(perception_publisher, timeout_sec=0.0)

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
        reader.stop()
        cv2.destroyAllWindows()
        perception_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
