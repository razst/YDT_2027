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
        raise ValueError("Gazebo camera image data does not match its dimensions")

    rgb = pixels.reshape(image.height, image.step)[:, : image.width * 3]
    rgb = rgb.reshape(image.height, image.width, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def main() -> None:
    frame_lock = threading.Lock()
    frame_ready = threading.Event()
    latest_frame: np.ndarray | None = None
    callback_error: Exception | None = None
    last_frame_time: float | None = None
    fps = 0.0

    def on_image(image: Image) -> None:
        nonlocal latest_frame, callback_error, last_frame_time, fps
        try:
            frame = image_to_bgr(image)
        except (ValueError, cv2.error) as error:
            with frame_lock:
                callback_error = error
            frame_ready.set()
            return

        now = time.perf_counter()
        with frame_lock:
            if last_frame_time is not None:
                elapsed = now - last_frame_time
                if elapsed > 0:
                    instant_fps = 1.0 / elapsed
                    fps = instant_fps if fps == 0.0 else 0.9 * fps + 0.1 * instant_fps
            last_frame_time = now
            latest_frame = frame
            callback_error = None
        frame_ready.set()

    node = Node()
    if not node.subscribe(Image, CAMERA_TOPIC, on_image):
        raise RuntimeError(f"Could not subscribe to Gazebo camera topic {CAMERA_TOPIC}")

    if not frame_ready.wait(timeout=10):
        raise RuntimeError(
            f"No camera frames received on {CAMERA_TOPIC}; is Gazebo running?"
        )
    with frame_lock:
        error = callback_error
    if error is not None:
        raise RuntimeError(f"Could not decode Gazebo camera frame: {error}")

    try:
        while True:
            with frame_lock:
                frame = latest_frame
                error = callback_error
                current_fps = fps
            if error is not None:
                raise RuntimeError(f"Could not decode Gazebo camera frame: {error}")
            if frame is not None:
                display_frame = frame.copy()
                fps_text = f"FPS: {current_fps:.1f}" if current_fps else "FPS: --"
                cv2.putText(
                    display_frame,
                    fps_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Gazebo Camera", display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
