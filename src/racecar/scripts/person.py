#!/usr/bin/env python3
"""White-only crosswalk detector for A4-paper stripes."""

import os
import sys
import threading
import time
import math

import cv2
import numpy as np


# ======================== User configuration ========================

CAMERA_INDEX = 20
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30.0

DISPLAY_SCALE = 0.8
DISPLAY_FPS = 15.0
SHOW_CAMERA_IMAGE = True

# Only inspect the road area.
ROI_TOP_RATIO = 0.42
ROI_BOTTOM_RATIO = 0.99
ROI_LEFT_RATIO = 0.03
ROI_RIGHT_RATIO = 0.97

# ---- White A4-paper threshold ----
# A pixel must be bright, low-saturation and have no strong RGB color bias.
WHITE_MIN_VALUE = 200
WHITE_MAX_SATURATION = 95
WHITE_MIN_RGB_CHANNEL = 155
WHITE_MAX_CHANNEL_SPREAD = 85

# Blue is explicitly rejected. It is NEVER used as a crosswalk feature.
BLUE_H_MIN = 90
BLUE_H_MAX = 125
BLUE_MIN_SATURATION = 80
BLUE_MIN_VALUE = 70

BLUR_KERNEL = 5
OPEN_KERNEL = 3
CLOSE_KERNEL_WIDTH = 7
CLOSE_KERNEL_HEIGHT = 5

# Geometry of one A4-paper stripe.
MIN_STRIPE_AREA = 300
MAX_STRIPE_AREA_RATIO = 0.22
MIN_STRIPE_ASPECT = 1.20
MAX_STRIPE_ASPECT = 6.0
MIN_RECT_FILL_RATIO = 0.52

# In the camera image, the long edge of the visible paper is approximately
# horizontal. Perspective can tilt the outer strips, so allow a generous angle.
MAX_STRIPE_ANGLE_DEG = 42.0

# At least this many white strips are needed.
MIN_STRIPES = 3
MAX_STRIPES_USED = 8

# Stripes should be side-by-side: similar Y position and enough X spread.
MAX_CENTER_Y_SPREAD_RATIO = 0.22
MIN_GROUP_X_SPAN_RATIO = 0.22

CONFIRM_FRAMES = 2
MARKER_PADDING = 6

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
        ("OPEN_KERNEL", OPEN_KERNEL),
    ):
        if kernel < 1 or kernel % 2 == 0:
            errors.append(f"{name} must be a positive odd integer")

    if CLOSE_KERNEL_WIDTH < 1 or CLOSE_KERNEL_HEIGHT < 1:
        errors.append("closing kernels must be positive")

    if MIN_STRIPE_AREA < 1:
        errors.append("MIN_STRIPE_AREA must be at least 1")

    if MIN_STRIPES < 2:
        errors.append("MIN_STRIPES must be at least 2")

    if CONFIRM_FRAMES < 1:
        errors.append("CONFIRM_FRAMES must be at least 1")

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


def long_edge_angle_deg(box):
    """Angle of the longest rectangle edge relative to image horizontal."""
    best_length = -1.0
    best_angle = 90.0

    for i in range(4):
        p1 = box[i].astype(np.float32)
        p2 = box[(i + 1) % 4].astype(np.float32)

        dx = float(p2[0] - p1[0])
        dy = float(p2[1] - p1[1])
        length = math.hypot(dx, dy)

        if length > best_length:
            angle = abs(math.degrees(math.atan2(dy, dx)))
            if angle > 90.0:
                angle = 180.0 - angle

            best_length = length
            best_angle = angle

    return best_angle


class CrosswalkDetector:
    def __init__(self):
        self.stable_state = "NONE"
        self.pending_state = "NONE"
        self.pending_count = 0

        self.white_mask = None
        self.stripes = []
        self.crosswalk_marker = None

        self.stripe_count = 0
        self.total_stripe_area = 0.0

    @staticmethod
    def make_white_mask(frame, hsv, roi_mask):
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        # 1. Basic HSV white test.
        hsv_white = cv2.inRange(
            hsv,
            np.array((0, 0, WHITE_MIN_VALUE), dtype=np.uint8),
            np.array((179, WHITE_MAX_SATURATION, 255), dtype=np.uint8),
        )

        # 2. RGB/BGR neutrality test.
        # Real white paper has R/G/B values relatively close to each other.
        blue, green, red = cv2.split(frame)

        maximum = cv2.max(cv2.max(blue, green), red)
        minimum = cv2.min(cv2.min(blue, green), red)
        channel_spread = cv2.subtract(maximum, minimum)

        min_channel_mask = cv2.inRange(
            minimum,
            WHITE_MIN_RGB_CHANNEL,
            255,
        )
        neutral_mask = cv2.inRange(
            channel_spread,
            0,
            WHITE_MAX_CHANNEL_SPREAD,
        )

        white_mask = cv2.bitwise_and(hsv_white, min_channel_mask)
        white_mask = cv2.bitwise_and(white_mask, neutral_mask)

        # 3. Explicitly reject blue.
        blue_mask = cv2.inRange(
            hsv,
            np.array(
                (BLUE_H_MIN, BLUE_MIN_SATURATION, BLUE_MIN_VALUE),
                dtype=np.uint8,
            ),
            np.array(
                (BLUE_H_MAX, 255, 255),
                dtype=np.uint8,
            ),
        )

        # Slight dilation prevents blue edges/highlights from leaking into white.
        blue_kernel = np.ones((3, 3), dtype=np.uint8)
        blue_mask = cv2.dilate(blue_mask, blue_kernel)

        not_blue = cv2.bitwise_not(blue_mask)
        white_mask = cv2.bitwise_and(white_mask, not_blue)

        # 4. Only keep the road ROI.
        white_mask = cv2.bitwise_and(white_mask, roi_mask)

        open_kernel = np.ones(
            (OPEN_KERNEL, OPEN_KERNEL),
            dtype=np.uint8,
        )
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_OPEN,
            open_kernel,
        )

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (CLOSE_KERNEL_WIDTH, CLOSE_KERNEL_HEIGHT),
        )
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
        )

        return white_mask

    @staticmethod
    def find_stripes(mask, roi_width, roi_height):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        roi_area = float(max(roi_width * roi_height, 1))
        candidates = []

        for contour in contours:
            area = float(cv2.contourArea(contour))

            if area < MIN_STRIPE_AREA:
                continue

            if area > roi_area * MAX_STRIPE_AREA_RATIO:
                continue

            rect = cv2.minAreaRect(contour)
            (_, _), (rect_width, rect_height), _ = rect

            if rect_width < 1.0 or rect_height < 1.0:
                continue

            long_side = max(rect_width, rect_height)
            short_side = min(rect_width, rect_height)
            aspect = long_side / max(short_side, 1.0)

            # A4 itself is only about 1.41:1, so do not require 2:1.
            if not MIN_STRIPE_ASPECT <= aspect <= MAX_STRIPE_ASPECT:
                continue

            rect_area = float(rect_width * rect_height)
            fill_ratio = area / max(rect_area, 1.0)

            if fill_ratio < MIN_RECT_FILL_RATIO:
                continue

            box = cv2.boxPoints(rect)
            angle = long_edge_angle_deg(box)

            if angle > MAX_STRIPE_ANGLE_DEG:
                continue

            candidates.append(
                {
                    "box": np.int32(np.round(box)),
                    "center_x": float(rect[0][0]),
                    "center_y": float(rect[0][1]),
                    "area": area,
                    "angle": angle,
                    "aspect": aspect,
                    "fill_ratio": fill_ratio,
                }
            )

        # Keep the strongest candidates.
        candidates.sort(
            key=lambda item: item["area"],
            reverse=True,
        )
        candidates = candidates[:MAX_STRIPES_USED]

        if len(candidates) < MIN_STRIPES:
            return candidates

        # Crosswalk strips in this scene are arranged LEFT -> RIGHT,
        # with roughly similar Y positions.
        median_y = float(
            np.median(
                [item["center_y"] for item in candidates]
            )
        )
        allowed_y_error = (
            roi_height * MAX_CENTER_Y_SPREAD_RATIO
        )

        row_candidates = [
            item
            for item in candidates
            if abs(item["center_y"] - median_y)
            <= allowed_y_error
        ]

        if len(row_candidates) >= MIN_STRIPES:
            candidates = row_candidates

        candidates.sort(
            key=lambda item: item["center_x"]
        )

        return candidates

    @staticmethod
    def is_crosswalk_group(stripes, roi_width):
        if len(stripes) < MIN_STRIPES:
            return False

        center_x = [
            item["center_x"]
            for item in stripes
        ]
        x_span = max(center_x) - min(center_x)

        # Several white rectangles close together at one point are not enough.
        # They must cover a useful width of the road.
        if x_span < roi_width * MIN_GROUP_X_SPAN_RATIO:
            return False

        return True

    @staticmethod
    def build_marker(stripes, frame_width, frame_height):
        if not stripes:
            return None

        all_points = np.concatenate(
            [item["box"] for item in stripes],
            axis=0,
        )

        x, y, width, height = cv2.boundingRect(all_points)

        x = max(0, x - MARKER_PADDING)
        y = max(0, y - MARKER_PADDING)

        width = min(
            frame_width - x,
            width + MARKER_PADDING * 2,
        )
        height = min(
            frame_height - y,
            height + MARKER_PADDING * 2,
        )

        return x, y, width, height

    def analyze(self, frame):
        blurred = cv2.GaussianBlur(
            frame,
            (BLUR_KERNEL, BLUR_KERNEL),
            0,
        )
        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV,
        )

        height, width = frame.shape[:2]

        top = int(height * ROI_TOP_RATIO)
        bottom = int(height * ROI_BOTTOM_RATIO)
        left = int(width * ROI_LEFT_RATIO)
        right = int(width * ROI_RIGHT_RATIO)

        roi_mask = np.zeros(
            (height, width),
            dtype=np.uint8,
        )
        roi_mask[top:bottom, left:right] = 255

        self.white_mask = self.make_white_mask(
            blurred,
            hsv,
            roi_mask,
        )

        roi_crop = self.white_mask[
            top:bottom,
            left:right,
        ]

        local_stripes = self.find_stripes(
            roi_crop,
            right - left,
            bottom - top,
        )

        stripes = []

        for stripe in local_stripes:
            stripe = dict(stripe)

            stripe["box"] = (
                stripe["box"]
                + np.array(
                    [left, top],
                    dtype=np.int32,
                )
            )
            stripe["center_x"] += left
            stripe["center_y"] += top

            stripes.append(stripe)

        self.stripes = stripes
        self.stripe_count = len(stripes)
        self.total_stripe_area = sum(
            item["area"]
            for item in stripes
        )

        if self.is_crosswalk_group(
            stripes,
            right - left,
        ):
            raw_state = "DETECTED"
            self.crosswalk_marker = self.build_marker(
                stripes,
                width,
                height,
            )
        else:
            raw_state = "NONE"
            self.crosswalk_marker = None

        return raw_state

    def update_state(self, raw_state):
        if raw_state == self.pending_state:
            self.pending_count += 1
        else:
            self.pending_state = raw_state
            self.pending_count = 1

        if (
            self.pending_count >= CONFIRM_FRAMES
            and raw_state != self.stable_state
        ):
            self.stable_state = raw_state

            print(
                f"crosswalk: {self.stable_state} "
                f"white_stripes={self.stripe_count} "
                f"area={self.total_stripe_area:.0f}",
                flush=True,
            )


def open_camera():
    capture = cv2.VideoCapture(
        CAMERA_INDEX,
        cv2.CAP_V4L2,
    )

    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(CAMERA_INDEX)

    if not capture.isOpened():
        return None

    capture.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG"),
    )
    capture.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        CAMERA_WIDTH,
    )
    capture.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        CAMERA_HEIGHT,
    )
    capture.set(
        cv2.CAP_PROP_FPS,
        CAMERA_FPS,
    )
    capture.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    return capture


def make_preview(frame, detector):
    if SHOW_CAMERA_IMAGE:
        return frame.copy()

    if detector.white_mask is None:
        mask = np.zeros(
            frame.shape[:2],
            dtype=np.uint8,
        )
    else:
        mask = detector.white_mask

    return cv2.cvtColor(
        mask,
        cv2.COLOR_GRAY2BGR,
    )


def draw_overlay(preview, detector, raw_state, measured_fps):
    # Green = accepted WHITE paper stripe.
    for stripe in detector.stripes:
        cv2.polylines(
            preview,
            [stripe["box"]],
            True,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    # Yellow = complete crosswalk.
    if detector.crosswalk_marker is not None:
        x, y, width, height = detector.crosswalk_marker

        cv2.rectangle(
            preview,
            (x, y),
            (x + width, y + height),
            (0, 255, 255),
            3,
        )

    height, width = preview.shape[:2]

    top = int(height * ROI_TOP_RATIO)
    bottom = int(height * ROI_BOTTOM_RATIO)
    left = int(width * ROI_LEFT_RATIO)
    right = int(width * ROI_RIGHT_RATIO)

    # Blue rectangle is ONLY the ROI display border.
    # It has nothing to do with blue-color detection.
    cv2.rectangle(
        preview,
        (left, top),
        (right, bottom),
        (255, 0, 0),
        1,
    )

    lines = (
        f"STATE={detector.stable_state} raw={raw_state} FPS={measured_fps:.1f}",
        f"WHITE stripes={detector.stripe_count} area={detector.total_stripe_area:.0f}",
        "green=WHITE A4  yellow=crosswalk  blue=ROI only",
        "Q/Esc: quit",
    )

    for index, line in enumerate(lines):
        y = 28 + index * 26

        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def main():
    errors = validate_configuration()

    if errors:
        for error in errors:
            print(
                f"configuration error: {error}",
                file=sys.stderr,
            )
        return 2

    if (
        sys.platform.startswith("linux")
        and not os.environ.get("DISPLAY")
    ):
        print(
            "warning: DISPLAY is not set; enable SSH X11 forwarding"
        )

    detector = CrosswalkDetector()

    capture = open_camera()

    if capture is None:
        print(
            f"camera open failed: /dev/video{CAMERA_INDEX}",
            file=sys.stderr,
        )
        return 1

    actual_width = int(
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )
    actual_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    print(
        f"camera opened: /dev/video{CAMERA_INDEX} "
        f"({actual_width}x{actual_height})"
    )

    reader = LatestFrameReader(capture)
    reader.start()

    window_name = "Racecar White Crosswalk Detector"

    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    measured_fps = 0.0
    window_has_frame = False

    try:
        cv2.namedWindow(
            window_name,
            cv2.WINDOW_NORMAL,
        )

        first_frame_deadline = (
            time.monotonic() + 5.0
        )

        while True:
            sequence, frame, last_frame_time = reader.latest()
            now = time.monotonic()

            if frame is None:
                if now >= first_frame_deadline:
                    print(
                        "read frame failed",
                        file=sys.stderr,
                    )
                    return 1

            elif now - last_frame_time > 2.0:
                print(
                    "camera stream stopped",
                    file=sys.stderr,
                )
                return 1

            elif sequence != last_sequence:
                raw_state = detector.analyze(frame)
                detector.update_state(raw_state)
                last_sequence = sequence

                if now >= next_display_time:
                    if last_display_time > 0.0:
                        instant_fps = (
                            1.0
                            / max(
                                now - last_display_time,
                                1e-6,
                            )
                        )

                        measured_fps = (
                            instant_fps
                            if measured_fps == 0.0
                            else measured_fps * 0.85
                            + instant_fps * 0.15
                        )

                    last_display_time = now

                    preview = make_preview(
                        frame,
                        detector,
                    )

                    draw_overlay(
                        preview,
                        detector,
                        raw_state,
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

                    cv2.imshow(
                        window_name,
                        preview,
                    )

                    next_display_time = (
                        now
                        + 1.0 / DISPLAY_FPS
                    )
                    window_has_frame = True

            key = cv2.waitKey(1) & 0xFF

            if key in (
                ord("q"),
                ord("Q"),
                27,
            ):
                break

            if (
                window_has_frame
                and cv2.getWindowProperty(
                    window_name,
                    cv2.WND_PROP_VISIBLE,
                ) < 1
            ):
                break

            time.sleep(0.001)

    except cv2.error as exc:
        print(
            f"OpenCV X11 window error: {exc}",
            file=sys.stderr,
        )
        return 1

    except KeyboardInterrupt:
        pass

    finally:
        reader.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
