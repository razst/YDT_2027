import threading
import os
import cv2
import time
from datetime import datetime
from collections import deque 
import constants

class RecordVideo: 
    def __init__(self, frame_queue, auto_start=False): 
        self.frame_queue = frame_queue
        # Removed hardcoded width and height!
        self.fps = constants.VIDEO_FPS 
        self.segment_duration = constants.SEGMENT_DURATION
        self.recording = True

        if constants.SEGMENT_DURATION != -1 and auto_start:
            thread_target = threading.Thread(target=self.record_main)
            thread_target.start()            

    # Pass the actual frame into this function
    def create_new_writer(self, frame, filename="v_out_"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Ensure output directory exists
        output_dir = constants.OUTPUT_DIR if hasattr(constants, "OUTPUT_DIR") else '.'
        os.makedirs(output_dir, exist_ok=True)
        output_file_name = f"{filename}{timestamp}.avi"
        output_file = os.path.join(output_dir, output_file_name)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        # FIX: Extract the EXACT height and width from the live frame
        height, width = frame.shape[:2]
        
        cwd = os.getcwd()
        print(f"Video writer cwd={cwd}")
        print("output_file:", output_file)
        writer = cv2.VideoWriter(output_file, fourcc, self.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"Unable to open VideoWriter for {output_file}")
        return writer, output_file

    def record_main(self):
        out = None
        current_output_file = ""
        print("Video recorder started")
        
        last_frame_id = None 
        
        try:
            # FIX: Wait until we actually have the VERY FIRST frame before creating the file
            first_frame = None
            while first_frame is None and self.recording:
                try:
                    # (Use pop() here if you are using the dual-buffer architecture, 
                    # or [-1] if you went with the single shared queue)
                    first_frame = self.frame_queue.popleft()
                except IndexError:
                    time.sleep(0.001)
                    
            if not self.recording:
                return

            # Now that we have a frame, we know the exact size! Create the writer.
            out, current_output_file = self.create_new_writer(first_frame, "PiCam_")
            print(f"Started recording to {current_output_file} at {first_frame.shape[1]}x{first_frame.shape[0]}")
            
            # Don't forget to write that very first frame we just grabbed!
            out.write(first_frame)
            last_frame_id = id(first_frame)
            segment_start_time = time.time()
            
            while self.recording:
                try:
                    frame = self.frame_queue.popleft()
                except IndexError:
                    time.sleep(0.001)
                    continue
                
                if id(frame) == last_frame_id:
                    time.sleep(0.001)
                    continue
                    
                last_frame_id = id(frame)

                if frame is None:
                    break

                elapsed_time = time.time() - segment_start_time
                if elapsed_time >= self.segment_duration:
                    out.release()
                    print(f"Finished recording to {current_output_file}")
                    
                    # Pass the current frame in to keep the exact same dimensions
                    out, current_output_file = self.create_new_writer(frame, "PiCam_")
                    print(f"Started recording to {current_output_file}")
                    segment_start_time = time.time()
                
                out.write(frame)

        except Exception as e:
            print(f"An error occurred: {e}")

        finally:
            print("Cleaning up resources...")
            if out:
                out.release()
            cv2.destroyAllWindows()
            if current_output_file:
                print(f"Last video saved as {current_output_file}")