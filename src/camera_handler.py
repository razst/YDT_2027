import cv2
import threading
import time
import os
from logger_handler import logger
# queue import removed!

class Camera:
    # Changed frame_queue to frame_buffer to match your updated main script
    def __init__(self, cam_idx, frame_buffers, auto_start=False, loop=False):
        self.source = cam_idx
        # Use the actual camera source passed in (index for webcam/simulator, or file path)
        self.cap = cv2.VideoCapture(cam_idx)
        self.is_file_source = isinstance(cam_idx, str) and os.path.exists(cam_idx)
        self.loop = loop

        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open camera/video stream: {cam_idx}")

        # Support a single buffer or a list of buffers
        self.frame_buffers = frame_buffers if isinstance(frame_buffers, list) else [frame_buffers]
        self.stopped = False

        if auto_start:
            self.start()

    def update(self):
        """
        Reads frames from 'cap' and appends them into 'frame_buffer'.
        """
        prev_time = 0
        while not self.stopped:
            ret, frame = self.cap.read()
            current_time = time.time()
            fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
            prev_time = current_time
                
            if not ret:
                if self.is_file_source and self.loop:
                    logger.info("Video file ended; reopening for loop.")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self.source)
                    if not self.cap.isOpened():
                        logger.error("Unable to reopen video file for looping.")
                        self.stop()
                        break
                    continue

                if self.is_file_source:
                    logger.info("Video file ended.")
                else:
                    logger.error("Stream ended or failed.")
                self.stop()
                break

            # --- THE MAGIC OF DEQUE ---
            # All the "if full, try/except get_nowait()" logic is DELETED.
            # Because we initialized deque with maxlen=1 in main.py,
            # append() automatically and instantly drops the old frame for you!
            for buf in self.frame_buffers:
                buf.append(frame)

    def start(self):
        thread = threading.Thread(target=self.update, daemon=True) 
        thread.start()
        logger.info("Camera thread started") # Fixed a small typo here (it previously said logger.info("stop"))
        return self

    def stop(self):
        self.stopped = True
        if self.cap.isOpened():
            self.cap.release()