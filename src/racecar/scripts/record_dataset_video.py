#!/usr/bin/env python3

"""Record low-latency camera video for making a vision dataset."""

import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2


# ======================== User configuration ========================

CAMERA_INDEX = 20
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30.0

# Videos are written here. "~" means the current user's home directory.
OUTPUT_DIRECTORY = "/home/bianbu/racecar/src/racecar/scripts/video"
VIDEO_FILENAME_PREFIX = "dataset"
VIDEO_CODEC = "MJPG"
VIDEO_EXTENSION = ".avi"

# The preview is only for observation. Its text is not saved in the video.
SHOW_PREVIEW = True
DISPLAY_SCALE = 0.5
DISPLAY_FPS = 15.0
AUTO_START_RECORDING = True

# ===================================================================


def validate_configuration():
    errors = []
    if CAMERA_INDEX < 0:
        errors.append("CAMERA_INDEX must be >= 0")
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0 or CAMERA_FPS <= 0:
        errors.append("camera width, height and fps must be greater than zero")
    if len(VIDEO_CODEC) != 4:
        errors.append("VIDEO_CODEC must contain exactly four characters")
    if not VIDEO_EXTENSION.startswith("."):
        errors.append("VIDEO_EXTENSION must start with a dot")
    if DISPLAY_SCALE <= 0 or DISPLAY_FPS <= 0:
        errors.append("display scale and fps must be greater than zero")
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


def make_output_path():
    output_directory = Path(OUTPUT_DIRECTORY).expanduser()
    output_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_directory / (
        f"{VIDEO_FILENAME_PREFIX}_{timestamp}{VIDEO_EXTENSION}"
    )


def draw_preview_overlay(frame, recording, saved_frames, measured_fps):
    status = "RECORDING" if recording else "PAUSED"
    status_color = (0, 0, 255) if recording else (0, 255, 255)
    elapsed = saved_frames / CAMERA_FPS
    lines = (
        (f"{status}  saved={saved_frames}  time={elapsed:.1f}s", status_color),
        (f"FPS={measured_fps:.1f}", (0, 255, 0)),
        ("Space: pause/resume   Q/Esc: save and quit", (255, 255, 255)),
    )
    for index, (text, color) in enumerate(lines):
        position = (10, 28 + index * 27)
        cv2.putText(
            frame,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )


def main():
    errors = validate_configuration()
    if errors:
        for error in errors:
            print(f"configuration error: {error}", file=sys.stderr)
        return 2

    preview_enabled = SHOW_PREVIEW
    if (
        preview_enabled
        and sys.platform.startswith("linux")
        and not os.environ.get("DISPLAY")
    ):
        print("warning: DISPLAY is not set; recording without an X11 preview")
        preview_enabled = False

    capture = open_camera()
    if capture is None:
        print(f"camera open failed: /dev/video{CAMERA_INDEX}", file=sys.stderr)
        return 1

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if actual_width <= 0 or actual_height <= 0:
        capture.release()
        print("camera returned an invalid resolution", file=sys.stderr)
        return 1

    output_path = make_output_path()
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*VIDEO_CODEC),
        CAMERA_FPS,
        (actual_width, actual_height),
    )
    if not writer.isOpened():
        capture.release()
        print(f"video writer open failed: {output_path}", file=sys.stderr)
        return 1

    print(f"camera opened: /dev/video{CAMERA_INDEX} ({actual_width}x{actual_height})")
    print(f"saving video to: {output_path}")
    if preview_enabled:
        print("Space pauses/resumes recording; Q or Esc saves and exits")
    else:
        print("press Ctrl+C to save and exit")

    reader = LatestFrameReader(capture)
    reader.start()
    recording = AUTO_START_RECORDING
    saved_frames = 0
    last_sequence = -1
    last_display_time = 0.0
    next_display_time = 0.0
    measured_fps = 0.0
    first_frame_deadline = time.monotonic() + 5.0
    window_name = "Racecar Dataset Video Recorder"
    window_has_frame = False

    try:
        if preview_enabled:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
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
                if recording:
                    writer.write(frame)
                    saved_frames += 1
                last_sequence = sequence

                if preview_enabled and now >= next_display_time:
                    if last_display_time > 0.0:
                        instant_fps = 1.0 / max(now - last_display_time, 1e-6)
                        measured_fps = (
                            instant_fps
                            if measured_fps == 0.0
                            else measured_fps * 0.85 + instant_fps * 0.15
                        )
                    last_display_time = now
                    preview = frame.copy()
                    draw_preview_overlay(
                        preview, recording, saved_frames, measured_fps
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

            if preview_enabled:
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    recording = not recording
                    print("recording resumed" if recording else "recording paused")
                elif key in (ord("q"), ord("Q"), 27):
                    break
                if (
                    window_has_frame
                    and cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
                ):
                    break

            time.sleep(0.001)
    except cv2.error as exc:
        print(f"OpenCV window/video error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        writer.release()
        if preview_enabled:
            cv2.destroyAllWindows()

    if saved_frames == 0:
        print(f"warning: no frames were saved; empty file: {output_path}")
    else:
        print(
            f"video saved: {output_path} "
            f"({saved_frames} frames, {saved_frames / CAMERA_FPS:.1f}s)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
