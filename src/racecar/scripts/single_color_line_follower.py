#!/usr/bin/env python3

"""Follow one colored line with a low-latency X11 OpenCV preview."""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# ======================== User configuration ========================

# Supported values: "red", "blue", "white", "yellow".
TARGET_COLOR = "yellow"

# True: show the camera image. False: show the selected color mask.
SHOW_CAMERA_IMAGE = False

# OpenCV HSV ranges: H=0..179, S=0..255, V=0..255.
# Red wraps around the ends of the Hue range, so it uses two intervals.
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

# Only this vertical region is used for line following.
ROI_TOP_RATIO = 0.45
ROI_BOTTOM_RATIO = 0.95
MIN_LINE_AREA = 500.0
BLUR_KERNEL = 5
MORPH_KERNEL = 5

# A valid line must enter from the bottom of the ROI.  Do not constrain its
# aspect ratio because a sharp curve can produce a wide, L-shaped contour.
BOTTOM_CONTACT_MARGIN = 20
MIN_LINE_HEIGHT = 60

CONTROL_TOPIC = "/car_cmd_vel"
STOP_PWM = 1500.0
MIN_DRIVE_PWM = 1548.0
MAX_DRIVE_PWM = 1548.0
TURN_DRIVE_PWM = 1543.0
DRIVER_SPEED_SCALE = 100.0
CONTROL_RATE = 15.0
TERMINAL_LOG_RATE = 5.0

# This car turns right for negative angular.z and left for positive angular.z.
STEERING_DIRECTION = -1.0
STEERING_LIMIT = 0.60
STEERING_GAIN = 0.65
STEERING_SMOOTHING = 0.60
CENTER_DEADBAND = 0.03
TURN_SPEED_THRESHOLD = 0.08

# Ackermann recovery: while already turning, keep rolling and hold the last
# steering direction briefly if the line leaves the camera field of view.
LOST_LINE_RECOVERY_SECONDS = 1.0
RECOVERY_TRIGGER_STEERING = 0.08
RECOVERY_MIN_STEERING = 0.30

# ===================================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def validate_configuration():
    errors = []
    if TARGET_COLOR not in COLOR_HSV_RANGES:
        errors.append(
            f"TARGET_COLOR must be one of: {', '.join(COLOR_HSV_RANGES)}"
        )
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0 or CAMERA_FPS <= 0:
        errors.append("camera width, height and fps must be greater than zero")
    if DISPLAY_SCALE <= 0 or DISPLAY_FPS <= 0:
        errors.append("display scale and fps must be greater than zero")
    if not 0.0 <= ROI_TOP_RATIO < ROI_BOTTOM_RATIO <= 1.0:
        errors.append("ROI ratios must satisfy 0 <= top < bottom <= 1")
    if MIN_LINE_AREA <= 0:
        errors.append("MIN_LINE_AREA must be greater than zero")
    if BLUR_KERNEL < 1 or BLUR_KERNEL % 2 == 0:
        errors.append("BLUR_KERNEL must be a positive odd integer")
    if MORPH_KERNEL < 1 or MORPH_KERNEL % 2 == 0:
        errors.append("MORPH_KERNEL must be a positive odd integer")
    if BOTTOM_CONTACT_MARGIN < 0 or MIN_LINE_HEIGHT <= 0:
        errors.append("line geometry pixel limits are invalid")
    if MIN_DRIVE_PWM < STOP_PWM or MAX_DRIVE_PWM < MIN_DRIVE_PWM:
        errors.append("PWM values must satisfy STOP <= MIN_DRIVE <= MAX_DRIVE")
    if not STOP_PWM <= TURN_DRIVE_PWM <= MIN_DRIVE_PWM:
        errors.append("TURN_DRIVE_PWM must be between STOP_PWM and MIN_DRIVE_PWM")
    if DRIVER_SPEED_SCALE <= 0 or CONTROL_RATE <= 0 or TERMINAL_LOG_RATE <= 0:
        errors.append("control scales and rates must be greater than zero")
    if STEERING_DIRECTION not in (-1.0, 1.0):
        errors.append("STEERING_DIRECTION must be -1.0 or 1.0")
    if STEERING_LIMIT <= 0:
        errors.append("STEERING_LIMIT must be greater than zero")
    if STEERING_GAIN < 0:
        errors.append("STEERING_GAIN must be >= 0")
    if not 0.0 <= STEERING_SMOOTHING <= 1.0:
        errors.append("STEERING_SMOOTHING must be between 0 and 1")
    if not 0.0 <= CENTER_DEADBAND < 1.0:
        errors.append("CENTER_DEADBAND must be in [0, 1)")
    if not 0.0 <= TURN_SPEED_THRESHOLD <= STEERING_LIMIT:
        errors.append("TURN_SPEED_THRESHOLD must be between 0 and STEERING_LIMIT")
    if LOST_LINE_RECOVERY_SECONDS < 0:
        errors.append("LOST_LINE_RECOVERY_SECONDS must be >= 0")
    if not 0.0 <= RECOVERY_TRIGGER_STEERING <= STEERING_LIMIT:
        errors.append("RECOVERY_TRIGGER_STEERING is outside the steering limit")
    if not RECOVERY_TRIGGER_STEERING <= RECOVERY_MIN_STEERING <= STEERING_LIMIT:
        errors.append("RECOVERY_MIN_STEERING is outside the recovery range")
    return errors


class LatestFrameReader:
    """Continuously capture frames and retain only the newest one."""

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


class SingleColorLineFollower(Node):
    def __init__(self):
        super().__init__("single_color_line_follower")
        self.publisher = self.create_publisher(Twist, CONTROL_TOPIC, 5)
        self.filtered_steering = 0.0
        self.last_publish_time = 0.0
        self.last_log_time = 0.0
        self.last_speed_pwm = STOP_PWM
        self.last_steering = 0.0
        self.last_line_seen_time = 0.0
        self.control_state = "LOST"

    def make_color_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_HSV_RANGES[TARGET_COLOR]:
            interval_mask = cv2.inRange(
                hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
            )
            mask = cv2.bitwise_or(mask, interval_mask)

        kernel = np.ones((MORPH_KERNEL, MORPH_KERNEL), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        height = frame.shape[0]
        roi_top = int(height * ROI_TOP_RATIO)
        roi_bottom = int(height * ROI_BOTTOM_RATIO)
        roi_mask = np.zeros_like(mask)
        roi_mask[roi_top:roi_bottom, :] = 255
        return cv2.bitwise_and(mask, roi_mask), roi_top, roi_bottom

    def detect_line(self, frame):
        mask, roi_top, roi_bottom = self.make_color_mask(frame)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, mask, roi_top, roi_bottom

        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < MIN_LINE_AREA:
                break

            x, y, width, height = cv2.boundingRect(contour)
            touches_bottom = y + height >= roi_bottom - BOTTOM_CONTACT_MARGIN
            if not touches_bottom:
                continue
            if height < MIN_LINE_HEIGHT:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue

            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            return (
                (center_x, center_y, area, (x, y, width, height)),
                mask,
                roi_top,
                roi_bottom,
            )

        return None, mask, roi_top, roi_bottom

    def calculate_command(self, detection, frame_width):
        now = time.monotonic()
        if detection is None:
            recovery_age = now - self.last_line_seen_time
            if (
                self.last_line_seen_time > 0.0
                and recovery_age <= LOST_LINE_RECOVERY_SECONDS
                and abs(self.filtered_steering) >= RECOVERY_TRIGGER_STEERING
            ):
                direction = 1.0 if self.filtered_steering > 0.0 else -1.0
                recovery_steering = direction * max(
                    abs(self.filtered_steering), RECOVERY_MIN_STEERING
                )
                recovery_steering = clamp(
                    recovery_steering, -STEERING_LIMIT, STEERING_LIMIT
                )
                self.filtered_steering = recovery_steering
                self.control_state = "RECOVERY"
                return TURN_DRIVE_PWM, recovery_steering

            self.filtered_steering = 0.0
            self.control_state = "LOST"
            return STOP_PWM, 0.0

        self.last_line_seen_time = now
        self.control_state = "TRACKING"
        center_x = detection[0]
        error = (center_x - frame_width / 2.0) / (frame_width / 2.0)
        if abs(error) < CENTER_DEADBAND:
            error = 0.0

        desired_steering = STEERING_DIRECTION * STEERING_GAIN * error
        desired_steering = clamp(
            desired_steering, -STEERING_LIMIT, STEERING_LIMIT
        )
        self.filtered_steering = (
            STEERING_SMOOTHING * desired_steering
            + (1.0 - STEERING_SMOOTHING) * self.filtered_steering
        )

        turn_ratio = abs(self.filtered_steering) / STEERING_LIMIT
        speed_pwm = MAX_DRIVE_PWM - turn_ratio * (MAX_DRIVE_PWM - MIN_DRIVE_PWM)
        speed_pwm = clamp(speed_pwm, MIN_DRIVE_PWM, MAX_DRIVE_PWM)
        if abs(self.filtered_steering) >= TURN_SPEED_THRESHOLD:
            speed_pwm = TURN_DRIVE_PWM
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
            state = "MOVE" if speed_pwm > STOP_PWM else "STOP"
            steering_pwm = 800.0 + steering * 400.0
            print(
                f"{state} mode={self.control_state} color={TARGET_COLOR} "
                f"speed_pwm={speed_pwm:.0f} "
                f"linear.x={linear_command:.3f} angular.z={steering:+.3f} "
                f"steering_pwm={steering_pwm:.0f}",
                flush=True,
            )
            self.last_log_time = now

    def stop_vehicle(self):
        self.filtered_steering = 0.0
        self.control_state = "STOPPED"
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


def draw_overlay(
    preview, follower, detection, roi_top, roi_bottom, measured_fps
):
    height, width = preview.shape[:2]
    cv2.rectangle(preview, (0, roi_top), (width - 1, roi_bottom), (0, 255, 255), 1)
    cv2.line(
        preview, (width // 2, roi_top), (width // 2, roi_bottom), (0, 255, 0), 1
    )

    if detection is None:
        line_text = f"line={follower.control_state}"
    else:
        center_x, center_y, area, (x, y, box_width, box_height) = detection
        cv2.rectangle(
            preview, (x, y), (x + box_width, y + box_height), (0, 0, 255), 2
        )
        cv2.circle(preview, (center_x, center_y), 6, (0, 255, 0), -1)
        line_text = f"line_area={area:.0f}"

    display_mode = "CAMERA" if SHOW_CAMERA_IMAGE else "MASK"
    lines = (
        f"ACTIVE color={TARGET_COLOR.upper()} mode={display_mode} {line_text}",
        f"PWM={follower.last_speed_pwm:.0f} turn={follower.last_steering:+.3f} FPS={measured_fps:.1f}",
        "Single-color line following - Q/Esc: stop and quit",
    )
    for index, line in enumerate(lines):
        y = 26 + index * 25
        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        color = (0, 255, 0) if index < 2 else (255, 255, 255)
        cv2.putText(
            preview,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
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
    follower = SingleColorLineFollower()
    capture = open_camera()
    if capture is None:
        print(f"camera open failed: /dev/video{CAMERA_INDEX}", file=sys.stderr)
        follower.stop_vehicle()
        follower.destroy_node()
        rclpy.shutdown()
        return 1

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    mode = "camera" if SHOW_CAMERA_IMAGE else "mask"
    print(f"camera opened: /dev/video{CAMERA_INDEX} ({actual_width}x{actual_height})")
    print(f"following {TARGET_COLOR} line; preview={mode}; topic={CONTROL_TOPIC}")
    print("autonomous line following is ACTIVE; Q or Esc stops and exits")

    reader = LatestFrameReader(capture)
    reader.start()
    window_name = "Racecar Single Color Line Follower"
    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    measured_fps = 0.0
    detection = None
    mask = None
    roi_top = 0
    roi_bottom = actual_height
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
                detection, mask, roi_top, roi_bottom = follower.detect_line(frame)
                speed_pwm, steering = follower.calculate_command(
                    detection, frame.shape[1]
                )
                follower.publish_command(speed_pwm, steering)
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

                    if SHOW_CAMERA_IMAGE:
                        preview = frame.copy()
                    else:
                        preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    draw_overlay(
                        preview,
                        follower,
                        detection,
                        roi_top,
                        roi_bottom,
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
        cv2.destroyAllWindows()
        follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
