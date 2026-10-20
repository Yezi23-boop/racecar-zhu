#!/usr/bin/env python3

"""Extract JPEG images from a recorded video for dataset annotation."""

import json
import sys
from pathlib import Path

import cv2


# ======================== User configuration ========================

# Leave empty to use the newest video in VIDEO_DIRECTORY. When only a filename
# is entered, such as "red.avi", it is searched for in VIDEO_DIRECTORY.
INPUT_VIDEO = "red.avi"
VIDEO_DIRECTORY = "/home/bianbu/racecar/src/racecar/scripts/video"

# YOLOv5 dataset directories. Images and labels use matching filenames.
DATASET_ROOT = "/home/bianbu/racecar/src/racecar/scripts/dataset"
IMAGE_DIRECTORY = "/home/bianbu/racecar/src/racecar/scripts/dataset/images"
LABEL_DIRECTORY = "/home/bianbu/racecar/src/racecar/scripts/dataset/labels"
DATASET_YAML = "/home/bianbu/racecar/src/racecar/scripts/dataset/dataset.yaml"

# Change these names to match the objects that will be annotated.
CLASS_NAMES = ["red_light", "green_light"]

# The first 80% of the video becomes train data and the final 20% becomes
# validation data. A time-based split avoids mixing adjacent frames at random.
TRAIN_RATIO = 0.8

# Save one image for every N source frames. At 30 fps, 5 means 6 images/second.
EXTRACT_EVERY_N_FRAMES = 5
START_TIME_SECONDS = 0.0
# Set to 0 to extract until the end of the video.
END_TIME_SECONDS = 0.0

IMAGE_EXTENSION = ".jpg"
JPEG_QUALITY = 95
IMAGE_NUMBER_WIDTH = 6

# ===================================================================


def validate_configuration():
    errors = []
    if EXTRACT_EVERY_N_FRAMES < 1:
        errors.append("EXTRACT_EVERY_N_FRAMES must be at least 1")
    if START_TIME_SECONDS < 0:
        errors.append("START_TIME_SECONDS must be >= 0")
    if END_TIME_SECONDS < 0:
        errors.append("END_TIME_SECONDS must be >= 0")
    if END_TIME_SECONDS > 0 and END_TIME_SECONDS <= START_TIME_SECONDS:
        errors.append("END_TIME_SECONDS must be greater than START_TIME_SECONDS")
    if IMAGE_EXTENSION.lower() not in (".jpg", ".jpeg", ".png"):
        errors.append("IMAGE_EXTENSION must be .jpg, .jpeg or .png")
    if not 1 <= JPEG_QUALITY <= 100:
        errors.append("JPEG_QUALITY must be between 1 and 100")
    if IMAGE_NUMBER_WIDTH < 1:
        errors.append("IMAGE_NUMBER_WIDTH must be at least 1")
    if not 0.0 < TRAIN_RATIO < 1.0:
        errors.append("TRAIN_RATIO must be between 0 and 1")
    if not CLASS_NAMES or any(not str(name).strip() for name in CLASS_NAMES):
        errors.append("CLASS_NAMES must contain at least one non-empty name")
    return errors


def find_input_video():
    if INPUT_VIDEO.strip():
        input_path = Path(INPUT_VIDEO).expanduser()
        if input_path.is_absolute():
            return input_path
        return Path(VIDEO_DIRECTORY).expanduser() / input_path

    video_directory = Path(VIDEO_DIRECTORY).expanduser()
    candidates = []
    for pattern in ("*.avi", "*.mp4", "*.mov", "*.mkv"):
        candidates.extend(video_directory.glob(pattern))
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def prepare_dataset_directories():
    image_root = Path(IMAGE_DIRECTORY).expanduser()
    label_root = Path(LABEL_DIRECTORY).expanduser()
    directories = {
        "train": image_root / "train",
        "val": image_root / "val",
    }
    for directory in directories.values():
        directory.mkdir(parents=True, exist_ok=True)
    (label_root / "train").mkdir(parents=True, exist_ok=True)
    (label_root / "val").mkdir(parents=True, exist_ok=True)
    return directories


def write_dataset_yaml():
    yaml_path = Path(DATASET_YAML).expanduser()
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    names = json.dumps(CLASS_NAMES, ensure_ascii=False)
    content = (
        f"path: {Path(DATASET_ROOT).expanduser()}\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(CLASS_NAMES)}\n"
        f"names: {names}\n"
    )
    yaml_path.write_text(content, encoding="utf-8")
    return yaml_path


def image_write_parameters():
    if IMAGE_EXTENSION.lower() in (".jpg", ".jpeg"):
        return [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    return [cv2.IMWRITE_PNG_COMPRESSION, 3]


def main():
    errors = validate_configuration()
    if errors:
        for error in errors:
            print(f"configuration error: {error}", file=sys.stderr)
        return 2

    input_video = find_input_video()
    if input_video is None:
        print(
            f"no video found in: {Path(VIDEO_DIRECTORY).expanduser()}",
            file=sys.stderr,
        )
        return 1
    if not input_video.is_file():
        print(f"input video does not exist: {input_video}", file=sys.stderr)
        return 1

    capture = cv2.VideoCapture(str(input_video))
    if not capture.isOpened():
        print(f"cannot open video: {input_video}", file=sys.stderr)
        return 1

    source_fps = float(capture.get(cv2.CAP_PROP_FPS))
    if source_fps <= 0:
        source_fps = 30.0
        print("warning: video FPS is unavailable; using 30 fps for time limits")
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    start_frame = int(round(START_TIME_SECONDS * source_fps))
    end_frame = (
        int(round(END_TIME_SECONDS * source_fps))
        if END_TIME_SECONDS > 0
        else total_frames
    )
    if total_frames > 0:
        end_frame = min(end_frame, total_frames)
    if end_frame > 0 and start_frame >= end_frame:
        capture.release()
        print("the selected time range contains no frames", file=sys.stderr)
        return 1

    image_directories = prepare_dataset_directories()
    yaml_path = write_dataset_yaml()
    write_parameters = image_write_parameters()
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    frame_index = start_frame
    saved_count = 0
    train_count = 0
    val_count = 0

    selected_frame_count = max(0, end_frame - start_frame)
    expected_image_count = (
        (selected_frame_count + EXTRACT_EVERY_N_FRAMES - 1)
        // EXTRACT_EVERY_N_FRAMES
        if selected_frame_count > 0
        else 0
    )
    train_image_limit = int(round(expected_image_count * TRAIN_RATIO))
    if expected_image_count >= 2:
        train_image_limit = min(
            expected_image_count - 1, max(1, train_image_limit)
        )

    print(f"input video: {input_video}")
    print(f"image directory: {Path(IMAGE_DIRECTORY).expanduser()}")
    print(f"label directory: {Path(LABEL_DIRECTORY).expanduser()}")
    print(f"dataset config: {yaml_path}")
    print(
        f"source: {source_fps:.2f} fps, {total_frames} frames; "
        f"extract every {EXTRACT_EVERY_N_FRAMES} frames"
    )

    try:
        while end_frame <= 0 or frame_index < end_frame:
            ok, frame = capture.read()
            if not ok or frame is None:
                break

            if (frame_index - start_frame) % EXTRACT_EVERY_N_FRAMES == 0:
                timestamp_ms = int(round(frame_index * 1000.0 / source_fps))
                filename = (
                    f"{input_video.stem}_"
                    f"{saved_count + 1:0{IMAGE_NUMBER_WIDTH}d}_"
                    f"{timestamp_ms:010d}ms{IMAGE_EXTENSION.lower()}"
                )
                subset = (
                    "train"
                    if expected_image_count == 0 or saved_count < train_image_limit
                    else "val"
                )
                output_path = image_directories[subset] / filename
                if not cv2.imwrite(str(output_path), frame, write_parameters):
                    print(f"failed to write image: {output_path}", file=sys.stderr)
                    return 1
                saved_count += 1
                if subset == "train":
                    train_count += 1
                else:
                    val_count += 1
                if saved_count % 100 == 0:
                    print(f"saved {saved_count} images")

            frame_index += 1
    except KeyboardInterrupt:
        print("extraction interrupted; already written images are retained")
    finally:
        capture.release()

    if saved_count == 0:
        print("no images were extracted", file=sys.stderr)
        return 1
    print(
        f"finished: {saved_count} images "
        f"(train={train_count}, val={val_count})"
    )
    print("put YOLO annotations with matching filenames in labels/train and labels/val")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
