import logging
from logging.handlers import RotatingFileHandler
from constants import *

# Set up logger
logger = logging.getLogger(__name__)
logger.setLevel(LOGGING_LEVEL)

# Formatter with time, function name, and message
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(funcName)s - %(message)s')

# Console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(LOGGING_LEVEL)
console_handler.setFormatter(formatter)

# File handler
# Create a RotatingFileHandler
# Rotates when the log file reaches 1MB (maxBytes) and keeps 5 backups (backupCount)
file_handler = RotatingFileHandler('droneV2.log', maxBytes=1024 * 1024, backupCount=5)
file_handler.setLevel(LOGGING_LEVEL)
file_handler.setFormatter(formatter)

# Avoid duplicate logs
if not logger.hasHandlers():
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)