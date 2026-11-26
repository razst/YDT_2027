import argparse
import threading
import time

import cv2
import numpy as np
from gz.msgs.image_pb2 import Image
from gz.transport import Node


CAMERA_TOPIC = "/iris/downward_camera"
RGB_INT8 = Image.DESCRIPTOR.fields_by_name[
    "pixel_format_type"
].enum_type.values_by_name["RGB_INT8"].number


def image_to_bgr(image: Image) -> np.ndarray:
    if image.pixel_format_type != RGB_INT8:
        raise ValueError(
            f"Unsupported Gazebo camera pixel format: {image.pixel_format_type}"
        )
    if image.width <= 0 or image.height <= 0 or image.step < image.width * 3:
        raise ValueError("Gazebo camera published invalid image dimensions")

    pixels = np.frombuffer(image.data, dtype=np.uint8)
    if pixels.size != image.height * image.step:
        raise ValueError("Gazebo camera image data does not match the frame size")

    rgb = pixels.reshape(image.height, image.step)[:, : image.width * 3]
    rgb = rgb.reshape(image.height, image.width, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def detect_red(frame: np.ndarray) -> tuple[list[tuple[int, int, int, float]], np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
    upper_red = cv2.inRange(hsv, (168, 90, 70), (179, 255, 255))
    mask = cv2.morphologyEx(
        lower_red | upper_red,
        cv2.MORPH_OPEN,
        np.ones((5, 5), np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((9, 9), np.uint8),
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    candidates: list[tuple[int, int, int, float]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 120:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue

        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        circle_area = np.pi * radius * radius
        fill_ratio = area / circle_area if circle_area > 0 else 0
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if radius >= 5 and fill_ratio >= 0.55 and circularity >= 0.55:
            candidates.append(
                (int(cx), int(cy), int(radius), circularity)
            )

    candidates.sort(key=lambda candidate: candidate[2], reverse=True)
    return candidates, mask


def annotate(frame: np.ndarray) -> np.ndarray:
    candidates, mask = detect_red(frame)
    height, width = frame.shape[:2]
    frame_center_x = width // 2
    frame_center_y = height // 2

    cv2.line(
        frame,
        (frame_center_x, 0),
        (frame_center_x, height),
        (255, 255, 255),
        1,
    )
    cv2.line(
        frame,
        (0, frame_center_y),
        (width, frame_center_y),
        (255, 255, 255),
        1,
    )

    for index, (cx, cy, radius, circularity) in enumerate(candidates):
        color = (0, 255, 0) if index == 0 else (0, 200, 255)
        cv2.circle(frame, (cx, cy), radius, color, 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"red ball | dx={cx - frame_center_x} dy={cy - frame_center_y}",
            (cx - radius, max(20, cy - radius - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
        )

    cv2.putText(
        frame,
        f"Detected: {len(candidates)}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.imshow("Red Ball Detector", frame)
    cv2.imshow("Red Mask", mask)
    return frame


def run_gazebo() -> None:
    frame_lock = threading.Lock()
    frame_ready = threading.Event()
    latest_frame: np.ndarray | None = None
    callback_error: Exception | None = None

    def on_image(image: Image) -> None:
        nonlocal latest_frame, callback_error
        try:
            frame = image_to_bgr(image)
        except (ValueError, cv2.error) as error:
            with frame_lock:
                callback_error = error
            frame_ready.set()
            return

        with frame_lock:
            latest_frame = frame
            callback_error = None
        frame_ready.set()

    node = Node()
    if not node.subscribe(Image, CAMERA_TOPIC, on_image):
        raise RuntimeError(
            f"Could not subscribe to Gazebo camera topic {CAMERA_TOPIC}"
        )
    if not frame_ready.wait(timeout=10):
        raise RuntimeError(
            f"No camera frames received on {CAMERA_TOPIC}; is Gazebo running?"
        )

    try:
        while True:
            with frame_lock:
                frame = latest_frame
                error = callback_error
            if error is not None:
                raise RuntimeError(f"Could not decode Gazebo camera frame: {error}")
            if frame is not None:
                annotate(frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


def run_webcam(camera_index: int) -> None:
    capture = cv2.VideoCapture(camera_index)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("Camera stream ended or failed")
            annotate(frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone downward-camera red-ball detector"
    )
    parser.add_argument(
        "--gazebo",
        action="store_true",
        help="Subscribe to the Gazebo /iris/downward_camera topic",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="OpenCV camera index for webcam mode",
    )
    args = parser.parse_args()

    if args.gazebo:
        run_gazebo()
    else:
        run_webcam(args.camera)


if __name__ == "__main__":
    main()
