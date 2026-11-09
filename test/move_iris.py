import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mavLink_handler import MavLinkHandler
from constants import *

try:
    mavLink:MavLinkHandler = MavLinkHandler(MAV_COM,MAV_MSG_FREQ)  # Initialize mavLink with empty parameters for testing
except Exception as e:
    print("Unable to connect to FC")
    print(f"Error: {e}")
    quit()
print("Connected to FC, about to ARM...")
mavLink.change_flight_mode(4)  # Change flight mode to GUIDED
mavLink.arm()  # Attempt to arm the drone
mavLink.takeoff(3)  # Attempt to take off to an altitude of 3 meters
time.sleep(5)  # Wait for 5 seconds to ensure the drone has taken off
mavLink.send_ned_velocity(2, 0, 0, 5)  # Move downwards for 5 seconds
mavLink.land()  # Attempt to land the drone