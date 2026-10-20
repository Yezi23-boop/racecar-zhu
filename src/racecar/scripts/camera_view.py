#!/usr/bin/env python3

"""Open a local camera and display its live image through OpenCV."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
import socket
import sys
import threading
import time

import cv2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Open a camera preview window. Press q or Esc to exit."
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=20,
        help="camera index, for example 20 means /dev/video20 (default: 20)",
    )
    parser.add_argument("--width", type=int, default=640, help="requested width")
    parser.add_argument("--height", type=int, default=480, help="requested height")
    parser.add_argument(
        "--display-scale",
        type=float,
        default=0.5,
        help="preview scale; 0.5 reduces bandwidth and latency (default: 0.5)",
    )
    parser.add_argument(
        "--display-fps",
        type=float,
        default=15.0,
        help="maximum preview frame rate (default: 15)",
    )
    parser.add_argument(
        "--http",
        action="store_true",
        help="use a low-latency MJPEG web preview instead of an X11 window",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="HTTP preview port (default: 8080)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=60,
        help="HTTP JPEG quality from 1 to 100 (default: 60)",
    )
    return parser.parse_args()


class LatestFrameReader:
    """Continuously capture frames and retain only the newest one."""

    def __init__(self, cap):
        self.cap = cap
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
            ok, frame = self.cap.read()
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
        self.cap.release()


def add_fps_overlay(frame, measured_fps):
    fps_text = f"FPS: {measured_fps:.1f}"
    cv2.putText(
        frame,
        fps_text,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        fps_text,
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )


def make_http_handler(reader, args):
    class CameraHttpHandler(BaseHTTPRequestHandler):
        def log_message(self, format_string, *format_args):
            # Keep the terminal clean while the browser reconnects.
            return

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>Racecar Camera</title>
<style>html,body{margin:0;background:#111;height:100%;display:grid;place-items:center}
img{max-width:100vw;max-height:100vh;image-rendering:auto}</style></head>
<body><img src="/stream.mjpg" alt="Racecar camera"></body></html>"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(page)
                return

            if self.path != "/stream.mjpg":
                self.send_error(404)
                return

            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()

            display_interval = 1.0 / args.display_fps
            next_frame_time = 0.0
            last_sequence = -1
            last_send_time = None
            measured_fps = 0.0

            try:
                while True:
                    now = time.monotonic()
                    sequence, frame, _ = reader.latest()
                    if (
                        frame is None
                        or sequence == last_sequence
                        or now < next_frame_time
                    ):
                        time.sleep(0.002)
                        continue

                    if last_send_time is not None:
                        elapsed = now - last_send_time
                        if elapsed > 0:
                            instant_fps = 1.0 / elapsed
                            measured_fps = (
                                instant_fps
                                if measured_fps == 0.0
                                else measured_fps * 0.85 + instant_fps * 0.15
                            )
                    last_send_time = now

                    if args.display_scale != 1.0:
                        frame = cv2.resize(
                            frame,
                            None,
                            fx=args.display_scale,
                            fy=args.display_scale,
                            interpolation=cv2.INTER_AREA,
                        )
                    else:
                        frame = frame.copy()

                    add_fps_overlay(frame, measured_fps)
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        frame,
                        [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                    )
                    if not ok:
                        continue

                    data = encoded.tobytes()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(data)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()

                    last_sequence = sequence
                    next_frame_time = now + display_interval
            except (BrokenPipeError, ConnectionResetError):
                pass

    return CameraHttpHandler


def run_http_preview(reader, args):
    server = ThreadingHTTPServer(
        ("0.0.0.0", args.port), make_http_handler(reader, args)
    )
    server.daemon_threads = True
    print(f"open this address in your PC browser: http://192.168.43.112:{args.port}")
    print("press Ctrl+C to stop")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main():
    args = parse_args()

    if args.display_scale <= 0:
        print("display scale must be greater than 0")
        return 2
    if args.display_fps <= 0:
        print("display fps must be greater than 0")
        return 2
    if not 1 <= args.jpeg_quality <= 100:
        print("jpeg quality must be between 1 and 100")
        return 2
    if not 1 <= args.port <= 65535:
        print("port must be between 1 and 65535")
        return 2

    if (
        not args.http
        and sys.platform.startswith("linux")
        and not os.environ.get("DISPLAY")
    ):
        print("warning: DISPLAY is not set; the OpenCV window cannot use X11")
        print("connect with MobaXterm X11 forwarding enabled, then run again")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        # Some OpenCV builds select the correct backend more reliably in auto mode.
        cap.release()
        cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        print(f"camera open failed: /dev/video{args.camera}")
        print("check the camera index and whether another program is using it")
        return 1

    # The icSpring USB camera on this car provides MJPG at 30 fps.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # Not every V4L2 backend honors this setting, so the background reader below
    # also prevents stale frames from accumulating.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    window_name = f"Racecar Camera /dev/video{args.camera}"

    print(f"camera opened: /dev/video{args.camera}")
    print(f"resolution: {actual_width}x{actual_height}")
    preview_type = "HTTP" if args.http else "X11"
    print(
        f"low-latency {preview_type} preview: scale={args.display_scale:g}, "
        f"max_fps={args.display_fps:g}"
    )

    reader = LatestFrameReader(cap)
    reader.start()

    if args.http:
        try:
            run_http_preview(reader, args)
        finally:
            reader.stop()
        return 0

    print("press q or Esc to close the window")

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        first_frame_deadline = time.monotonic() + 5.0
        display_interval = 1.0 / args.display_fps
        next_display_time = 0.0
        last_sequence = -1
        last_display_time = None
        measured_fps = 0.0
        window_has_frame = False

        while True:
            sequence, frame, last_frame_time = reader.latest()
            now = time.monotonic()

            if frame is None:
                if now >= first_frame_deadline:
                    print("read frame failed")
                    return 1
            elif now - last_frame_time > 2.0:
                print("camera stream stopped")
                return 1
            elif sequence != last_sequence and now >= next_display_time:
                if last_display_time is not None:
                    elapsed = now - last_display_time
                    if elapsed > 0:
                        instant_fps = 1.0 / elapsed
                        if measured_fps == 0.0:
                            measured_fps = instant_fps
                        else:
                            measured_fps = measured_fps * 0.85 + instant_fps * 0.15
                last_display_time = now

                if args.display_scale != 1.0:
                    frame = cv2.resize(
                        frame,
                        None,
                        fx=args.display_scale,
                        fy=args.display_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    frame = frame.copy()

                add_fps_overlay(frame, measured_fps)

                cv2.imshow(window_name, frame)
                last_sequence = sequence
                next_display_time = now + display_interval
                window_has_frame = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break

            # Also exit cleanly when the user closes the window with the mouse.
            if (
                window_has_frame
                and cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
            ):
                break

            time.sleep(0.001)
    except cv2.error as exc:
        print(f"OpenCV window error: {exc}")
        print("please confirm MobaXterm X11 forwarding is enabled and DISPLAY is set")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
