#!/usr/bin/env python3

"""Low-latency red/blue object tracking for the racecar.

The node publishes normalized Twist commands to /car_cmd_vel.  The chassis
driver converts linear.x with ``pwm = linear.x * 100 + 1500``.
"""

import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# Preview mode: True displays the camera image; False displays the color mask.
SHOW_CAMERA_IMAGE = False


def hsv_triplet(text):
    try:
        values = tuple(int(item.strip()) for item in text.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("HSV must be H,S,V integers") from exc
    if len(values) != 3:
        raise argparse.ArgumentTypeError("HSV must contain exactly three values")
    h, s, v = values
    if not 0 <= h <= 179 or not 0 <= s <= 255 or not 0 <= v <= 255:
        raise argparse.ArgumentTypeError("HSV ranges are H=0..179, S/V=0..255")
    return values


def odd_positive(value):
    value = int(value)
    if value < 1 or value % 2 == 0:
        raise argparse.ArgumentTypeError("value must be a positive odd integer")
    return value


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track a red or blue object and control the racecar."
    )
    camera = parser.add_argument_group("camera and X11 preview")
    camera.add_argument("--camera", type=int, default=20)
    camera.add_argument("--width", type=int, default=640)
    camera.add_argument("--height", type=int, default=480)
    camera.add_argument("--camera-fps", type=float, default=30.0)
    camera.add_argument("--display-scale", type=float, default=0.5)
    camera.add_argument("--display-fps", type=float, default=15.0)

    detection = parser.add_argument_group("color detection")
    detection.add_argument("--color", choices=("red", "blue"), default="red")
    detection.add_argument("--red-lower-1", type=hsv_triplet, default=(0, 100, 70))
    detection.add_argument("--red-upper-1", type=hsv_triplet, default=(10, 255, 255))
    detection.add_argument("--red-lower-2", type=hsv_triplet, default=(170, 100, 70))
    detection.add_argument("--red-upper-2", type=hsv_triplet, default=(179, 255, 255))
    detection.add_argument("--blue-lower", type=hsv_triplet, default=(95, 90, 60))
    detection.add_argument("--blue-upper", type=hsv_triplet, default=(135, 255, 255))
    detection.add_argument("--blur-kernel", type=odd_positive, default=5)
    detection.add_argument("--morph-kernel", type=odd_positive, default=5)
    detection.add_argument("--min-area", type=float, default=500.0)

    control = parser.add_argument_group("vehicle control")
    control.add_argument("--topic", default="/car_cmd_vel")
    control.add_argument("--neutral-pwm", type=float, default=1500.0)
    control.add_argument("--min-forward-pwm", type=float, default=1548.0)
    control.add_argument("--max-forward-pwm", type=float, default=1548.0)
    control.add_argument("--driver-speed-scale", type=float, default=100.0)
    control.add_argument("--steering-gain", type=float, default=0.45)
    control.add_argument("--steering-limit", type=float, default=0.60)
    control.add_argument(
        "--steering-direction",
        type=float,
        choices=(-1.0, 1.0),
        default=-1.0,
        help="steering polarity; this car uses -1 (default)",
    )
    control.add_argument("--steering-smoothing", type=float, default=0.35)
    control.add_argument("--center-deadband", type=float, default=0.05)
    control.add_argument("--slow-area-ratio", type=float, default=0.025)
    control.add_argument("--stop-area-ratio", type=float, default=0.10)
    control.add_argument("--lost-frames", type=int, default=0)
    control.add_argument("--control-rate", type=float, default=15.0)
    control.add_argument("--terminal-log-rate", type=float, default=5.0)
    args, ros_args = parser.parse_known_args()
    return args, ros_args


def clamp(value, low, high):
    return max(low, min(high, value))


def validate_args(args):
    errors = []
    for name in ("width", "height"):
        if getattr(args, name) <= 0:
            errors.append(f"--{name} must be greater than zero")
    for name in (
        "camera_fps",
        "display_fps",
        "display_scale",
        "control_rate",
        "terminal_log_rate",
    ):
        if getattr(args, name) <= 0:
            errors.append(f"--{name.replace('_', '-')} must be greater than zero")
    if args.min_area <= 0:
        errors.append("--min-area must be greater than zero")
    if args.max_forward_pwm < args.min_forward_pwm:
        errors.append("--max-forward-pwm must be >= --min-forward-pwm")
    if args.min_forward_pwm < args.neutral_pwm:
        errors.append("--min-forward-pwm must be >= --neutral-pwm")
    if args.driver_speed_scale <= 0:
        errors.append("--driver-speed-scale must be greater than zero")
    if args.steering_gain < 0:
        errors.append("--steering-gain must be >= 0")
    if args.steering_limit <= 0:
        errors.append("--steering-limit must be greater than zero")
    if not 0.0 <= args.steering_smoothing <= 1.0:
        errors.append("--steering-smoothing must be between 0 and 1")
    if not 0.0 <= args.center_deadband < 1.0:
        errors.append("--center-deadband must be in [0, 1)")
    if not 0.0 <= args.slow_area_ratio < args.stop_area_ratio < 1.0:
        errors.append("area ratios must satisfy 0 <= slow < stop < 1")
    if args.lost_frames < 0:
        errors.append("--lost-frames must be >= 0")
    return errors


class LatestFrameReader:
    """Continuously read the camera and expose only the newest frame."""

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


class ColorTracker(Node):
    def __init__(self, args):
        super().__init__("color_object_tracker")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.topic, 5)
        self.target_color = args.color
        self.lost_count = 0
        self.filtered_steering = 0.0
        self.last_publish_time = 0.0
        self.last_log_time = 0.0
        self.last_speed = args.neutral_pwm
        self.last_steering = 0.0

    def make_mask(self, hsv):
        if self.target_color == "red":
            mask_1 = cv2.inRange(
                hsv, np.array(self.args.red_lower_1), np.array(self.args.red_upper_1)
            )
            mask_2 = cv2.inRange(
                hsv, np.array(self.args.red_lower_2), np.array(self.args.red_upper_2)
            )
            mask = cv2.bitwise_or(mask_1, mask_2)
        else:
            mask = cv2.inRange(
                hsv, np.array(self.args.blue_lower), np.array(self.args.blue_upper)
            )
        kernel = np.ones(
            (self.args.morph_kernel, self.args.morph_kernel), dtype=np.uint8
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def detect(self, frame):
        blurred = cv2.GaussianBlur(
            frame, (self.args.blur_kernel, self.args.blur_kernel), 0
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = self.make_mask(hsv)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, mask
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.args.min_area:
            return None, mask
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, mask
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        x, y, width, height = cv2.boundingRect(contour)
        area_ratio = area / float(frame.shape[0] * frame.shape[1])
        return (center_x, center_y, area_ratio, (x, y, width, height)), mask

    def calculate_command(self, detection, frame_width):
        if detection is None:
            return self.args.neutral_pwm, 0.0

        center_x, _, area_ratio, _ = detection
        error = (center_x - frame_width / 2.0) / (frame_width / 2.0)
        if abs(error) < self.args.center_deadband:
            error = 0.0

        # The physical steering servo on this car turns left for positive
        # angular.z, so its default polarity is the inverse of image error.
        desired_steering = (
            self.args.steering_direction * self.args.steering_gain * error
        )
        desired_steering = clamp(
            desired_steering, -self.args.steering_limit, self.args.steering_limit
        )
        alpha = self.args.steering_smoothing
        self.filtered_steering = (
            alpha * desired_steering + (1.0 - alpha) * self.filtered_steering
        )

        if area_ratio >= self.args.stop_area_ratio:
            speed = self.args.neutral_pwm
        elif area_ratio <= self.args.slow_area_ratio:
            speed = self.args.max_forward_pwm
        else:
            span = self.args.stop_area_ratio - self.args.slow_area_ratio
            remaining = (self.args.stop_area_ratio - area_ratio) / span
            speed = self.args.min_forward_pwm + remaining * (
                self.args.max_forward_pwm - self.args.min_forward_pwm
            )
        return speed, self.filtered_steering

    def publish(self, speed, steering, force=False):
        now = time.monotonic()
        if not force and now - self.last_publish_time < 1.0 / self.args.control_rate:
            return
        message = Twist()
        linear_command = (speed - self.args.neutral_pwm) / self.args.driver_speed_scale
        message.linear.x = float(linear_command)
        message.angular.z = float(steering)
        self.publisher.publish(message)
        self.last_publish_time = now
        self.last_speed = speed
        self.last_steering = steering

        if now - self.last_log_time >= 1.0 / self.args.terminal_log_rate:
            state = "MOVE" if speed > self.args.neutral_pwm else "STOP"
            steering_pwm = 1500.0 + steering * 1300.0
            print(
                f"{state} speed_pwm={speed:.0f} linear.x={linear_command:.3f} "
                f"angular.z={steering:+.3f} steering_pwm={steering_pwm:.0f}",
                flush=True,
            )
            self.last_log_time = now

    def stop_vehicle(self):
        self.filtered_steering = 0.0
        for _ in range(5):
            self.publish(self.args.neutral_pwm, 0.0, force=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

def open_camera(args):
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        return None
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def draw_overlay(frame, tracker, detection, measured_fps):
    height, width = frame.shape[:2]
    cv2.line(frame, (width // 2, 0), (width // 2, height), (100, 100, 100), 1)
    if detection is not None:
        center_x, center_y, area_ratio, (x, y, box_width, box_height) = detection
        box_color = (0, 0, 255) if tracker.target_color == "red" else (255, 0, 0)
        cv2.rectangle(frame, (x, y), (x + box_width, y + box_height), box_color, 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
        target_text = f"target={area_ratio * 100:.1f}%"
    else:
        target_text = "target=LOST"

    lines = (
        f"ACTIVE color={tracker.target_color.upper()} {target_text}",
        f"PWM={tracker.last_speed:.0f} turn={tracker.last_steering:+.3f} FPS={measured_fps:.1f}",
        "Autonomous tracking - Q/Esc: stop and quit",
    )
    for index, line in enumerate(lines):
        y = 26 + index * 25
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        color = (0, 255, 0) if index < 2 else (255, 255, 255)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def main():
    args, ros_args = parse_args()
    errors = validate_args(args)
    if errors:
        for error in errors:
            print(f"error: {error}", file=sys.stderr)
        return 2
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        print("warning: DISPLAY is not set; enable SSH X11 forwarding")

    rclpy.init(args=ros_args)
    tracker = ColorTracker(args)
    capture = open_camera(args)
    if capture is None:
        print(f"camera open failed: /dev/video{args.camera}", file=sys.stderr)
        tracker.stop_vehicle()
        tracker.destroy_node()
        rclpy.shutdown()
        return 1

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera opened: /dev/video{args.camera} ({actual_width}x{actual_height})")
    print(f"publishing normalized Twist commands to {args.topic}")
    print(
        f"chassis PWM targets: stop={args.neutral_pwm:.0f}, "
        f"forward={args.min_forward_pwm:.0f}..{args.max_forward_pwm:.0f}"
    )
    print("autonomous tracking is ACTIVE; Q or Esc stops the vehicle and exits")
    print("keep the wheels off the ground for the first control test")

    reader = LatestFrameReader(capture)
    reader.start()
    window_name = "Racecar Red/Blue Tracker"
    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    measured_fps = 0.0
    detection = None
    mask = None
    window_has_frame = False

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        first_frame_deadline = time.monotonic() + 5.0
        while rclpy.ok():
            rclpy.spin_once(tracker, timeout_sec=0.0)
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
                detection, mask = tracker.detect(frame)
                if detection is None:
                    tracker.lost_count += 1
                    if tracker.lost_count > args.lost_frames:
                        speed, steering = args.neutral_pwm, 0.0
                    else:
                        speed, steering = tracker.last_speed, tracker.last_steering
                else:
                    tracker.lost_count = 0
                    speed, steering = tracker.calculate_command(detection, frame.shape[1])
                tracker.publish(speed, steering)
                last_sequence = sequence

                if now >= next_display_time:
                    if last_display_time > 0.0:
                        instant_fps = 1.0 / max(now - last_display_time, 1e-6)
                        measured_fps = instant_fps if measured_fps == 0.0 else measured_fps * 0.85 + instant_fps * 0.15
                    last_display_time = now
                    if SHOW_CAMERA_IMAGE:
                        preview = frame.copy()
                    else:
                        preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    draw_overlay(preview, tracker, detection, measured_fps)
                    if args.display_scale != 1.0:
                        preview = cv2.resize(
                            preview,
                            None,
                            fx=args.display_scale,
                            fy=args.display_scale,
                            interpolation=cv2.INTER_AREA,
                        )
                    cv2.imshow(window_name, preview)
                    next_display_time = now + 1.0 / args.display_fps
                    window_has_frame = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if window_has_frame and cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
            time.sleep(0.001)
    except cv2.error as exc:
        print(f"OpenCV X11 window error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop_vehicle()
        reader.stop()
        cv2.destroyAllWindows()
        tracker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
