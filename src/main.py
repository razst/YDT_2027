from turtle import speed
from logger_handler import logger
from mavLink_handler import *
from constants import *
from camera_handler import *
from collections import deque # FIX 1: Import deque instead of queue
import time
from record_video import *


if __name__ == "__main__":
   logger.info("Starting autonumi script 2026...")
   # Connect the FC
   try:
      mavLink:MavLinkHandler = MavLinkHandler(MAV_COM,MAV_MSG_FREQ) 
   except Exception as e:
      logger.error("Unable to connect to FC")
      logger.error(f"Error: {e}")
      quit()
      
   # wait 4 GUIDED mode
   mavLink.check_until_guided()
   


   quit()