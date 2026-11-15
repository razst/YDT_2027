import time
import sys
from pathlib import Path
from venv import logger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mavLink_handler import MavLinkHandler
from constants import *



logger.info("Starting test script...")
try:
    mavLink:MavLinkHandler = MavLinkHandler(MAV_COM,MAV_MSG_FREQ)  # Initialize mavLink with empty parameters for testing
except Exception as e:
    logger.error("Unable to connect to FC")
    logger.error(f"Error: {e}")
    quit()
logger.info("Connected to FC, about to ARM...")
mavLink.change_flight_mode(4)  # Change flight mode to GUIDED
mavLink.arm()  # Attempt to arm the drone
mavLink.takeoff(6,True,0.1)  # Attempt to take off to an altitude of 3 meters
logger.info("Takeoff completed.")
mavLink.send_ned_velocity(2, 0, 0, 500)  # Move downwards for 5 seconds
mavLink.land(True)  # Attempt to land the drone
logger.info("Landing completed.")
