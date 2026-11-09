import logging
import os

# MAVLINK params
# H7 over GPIO = '/dev/serial0', baud=921600
# H7 over USB = '/dev/ttyACM0', baud=921600
# H7 windows SITL com "tcp:localhost:5763" baud=115200 

# MAV_COM="/dev/ttyACM0"
# MAV_COM="/dev/serial0"

# MAV_COM="COM12"
# BAUD_RATE=115200
# MAV_COM="tcp:localhost:5763" # SITL com
BAUD_RATE=921600
MAV_COM="tcp:localhost:5763" # SITL com
MAV_MSG_FREQ = 10 # how often do we want to get messages from mavlink (in Hz)
MAX_ALLOWED_ALT = 8  # Maximum allowed altitude in meters to prevent flyaways

# General params
LOGGING_LEVEL = logging.DEBUG  # You can set to INFO, WARNING, ERROR etc.
IS_HEADLESS = False
# Camera params
CAMERA_IDX=0

#video out params
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "recordings")
SEGMENT_DURATION =  15 # Duration of each video segment in seconds. set -1 to disable video output
VIDEO_FPS = 5
active_color=[]
# Detect params
FRAMES_CENTERED = 5 # Number of consecutive frames the target must be centered to fire. This helps ensure we have a stable lock before firing, reducing false positives.

# fire params
FIRE_ALTITUDE = 3.0  # meters, target altitude for hold_altitude
VELOCITY_Z = -0.5  # m/s, vertical velocity to maintain during firing (positive for down, negative for up)
DRONE_MOVE_ANGLE = 1
SERVO_SPEED = 0.25
SERVO_CHANNEL = 9
PUMP_RELAY=0 #(0 for Relay1, 1 for Relay2, etc.)