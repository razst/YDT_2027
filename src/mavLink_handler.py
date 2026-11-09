from pymavlink import mavutil
import math
import time
from constants import*
from logger_handler import logger

class NotGuidedException(Exception):
    def __init__(self, message="Drone not in GUIDED mode"):
        self.message = message
        super().__init__(self.message)


class MavLinkCommandError(Exception):
    def __init__(self, message="MavLink command failed"):
        self.message = message
        super().__init__(self.message)

class MavLinkHandler:
    def __init__(self, connection_string,msg_frq=1): # msg freq in Hz
        self._last_pos = 0
        self._connection = self._connect(connection_string,msg_frq)
        
    def _check_guided_mode(self):
        mode = self.get_curr_mode()
        logger.debug(f"mode={mode}")
        if mode!=4:
            raise NotGuidedException
        else:
            return True 
    def check_until_guided(self):
        mode = self.get_curr_mode()
        logger.debug(f"mode={mode}")
        while mode != 4:
            mode = self.get_curr_mode()
            logger.debug(f"mode={mode}")
    def _connect(self,connection_string,msg_frq):
        the_connection = mavutil.mavlink_connection(connection_string,source_system=1,baud=BAUD_RATE)
        the_connection.wait_heartbeat()
        logger.info("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

        # ask to get all messages in a rate of msg_frq Hz
        data_rate = msg_frq # Hz
        the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, data_rate, 1)
        return the_connection


    # wait for the next message of type. msg_type can be str or list of str. passing empty string will return the next message of any type
    # for list of message type in ardupilot: https://ardupilot.org/copter/docs/ArduCopter_MAVLink_Messages.html#requestable-messages
    # some common ones: ATTITUDE, RAW_IMU, HEARTBEAT, WIND, RC_CHANNELS
    def _get_message(self, msg_type:str):
        if msg_type == "" or msg_type is None:
            msg = self._connection.recv_match(blocking=True)
            if msg is None:
                logger.warning("Timeout waiting for any message")
                return None
            logger.debug(msg.get_type())
            return msg

        # Get the first message of the type (blocking)
        msg = self._connection.recv_match(type=msg_type, blocking=True)
        if msg is None:
            logger.warning(f"Timeout waiting for message of type {msg_type}")
            return None

        # Drain the buffer for the same type to get the most recent one
        while True:
            next_msg = self._connection.recv_match(type=msg_type, blocking=False)
            if next_msg is None:
                break
            msg = next_msg

        return msg        

    def set_motor_relay(self,relay,state):
        """
        state: 1 for ON, 0 for OFF
        """
        # MAV_CMD_DO_SET_RELAY (index 181)
        # Param 1: Relay number (0 for Relay1, 1 for Relay2, etc.)
        # Param 2: Setting (1 for ON, 0 for OFF)
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
            0,      # Confirmation
            relay,      # Relay Instance (0 = RELAY1)
            state,  # 1 = ON, 0 = OFF
            0, 0, 0, 0, 0
        )
        status = "ON" if state == 1 else "OFF"
        logger.info(f"Motor Relay set to {status}")

    def move_servo(self, servo_channel, servo_angle):
        try:
            # ******* Send TEXT messages ********
            # Moves a servo connected to the ardupilot
            # aux_ch - use 7 for main_out 7, 8 for main_out 8
            # pwm - 900 = 0 degree, 2000 = 90 degree  
            logger.debug("requsting to move servo in FC")
            pwm_angle = int((2000 - 900) * (servo_angle / 90) + 900) 
            logger.debug(f"sending command move servo to FC with value: {pwm_angle}, channel: {servo_channel}, angle: {servo_angle}")
            self._connection.mav.command_long_send(self._connection.target_system, self._connection.target_component
            ,mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0,servo_channel, pwm_angle, 0,0,0,0,0)
            # self.logger.debug("waiting to recev command ack from pix")
            # msg = self._connction.recv_match(type='COMMAND_ACK', blocking=True)
            # if msg.result != 0:
            #     self.logger.critical("Unable to move servo.")
            # else:
            #     self.logger.info("servo moved")
        except Exception as error:
            self.logger.exception("Failed to send move servo to FC")

    def send_text(self,msg):
        # ******* Send TEXT messages ********
        # how to send text that will show up on mission planner
        # use MAV_SEVERITY_WARNING. if you use MAV_SEVERITY_CRITICAL than it will not arm / takeoff. and MAV_SEVERITY_INFO will not always show up
        # can also use as b"Hello"
        self._connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, bytes(msg,'utf-8'))
        logger.debug(msg)

    def change_flight_mode(self,mode:int):
        # ******* Change flight mode ********
        # Change flight mode. 0-Stabilize, 4-Guided, 5-Loiter, 6-RTL, 9-Land
        # mode values: https://ardupilot.org/copter/docs/parameters.html#fltmode1
        # see also https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html &
        self._connection.mav.command_long_send(self._connection.target_system, self._connection.target_component,
                                                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0)
        msg = self._connection.recv_match(type='COMMAND_ACK',blocking=True)
        if msg is None:
            error_msg = f"Timeout waiting for ACK to change mode to {mode}"
            self.send_text(error_msg)
            raise MavLinkCommandError(error_msg)
        elif msg.result != 0:
            error_msg = f"Unable to change to mode {mode}"
            self.send_text(error_msg)
            raise MavLinkCommandError(error_msg)
        else:
            self.send_text(f"Mode changed to {mode}")

    def arm(self):
        # ******* ARM ********
        self._connection.mav.command_long_send(self._connection.target_system, self._connection.target_component,
                                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        msg = self._connection.recv_match(type='COMMAND_ACK',blocking=True)
        if msg is None:
            error_msg = "Timeout waiting for ARM ACK"
            self.send_text(error_msg)
            raise MavLinkCommandError(error_msg)
        elif msg.result != 0:
            error_msg = "Unable to ARM"
            self.send_text(f"{error_msg}. Exiting !!!")
            raise MavLinkCommandError(error_msg)
        else:
            self.send_text("ARMED !!!!")

    def get_curr_mode(self):
        msg=self._get_message("HEARTBEAT")
        while msg.type ==6: # 6 is type GCS, we want to ignore these
            msg=self._get_message("HEARTBEAT")
        return msg.custom_mode
        
    def get_curr_height(self):
        # msg = self._connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg = self._get_message('GLOBAL_POSITION_INT')
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                alt_meter = msg.relative_alt / 1000
                self.send_text(f"Current height is {alt_meter} meters")
                return alt_meter
            else:
                self.send_text(f'didnt get the right message type')


    def ensure_height(self, target_height, tolerance=0.5, timeout=30):
        """
        Moves the drone up or down until it reaches the target height within tolerance.
        Returns True if target reached, False if timeout occurs.
        """
        start_time = time.time()
        logger.info(f"Ensuring height {target_height}m (tol: {tolerance}m)...")

        while time.time() - start_time < timeout:
            curr_h = self.get_curr_height()
            if curr_h is None:
                continue

            if abs(curr_h - target_height) <= tolerance:
                self.send_text(f"Height target reached: {curr_h:.2f}m")
                # Stop vertical movement
                self.send_ned_velocity(0, 0, 0, 100)
                return True

            # Determine velocity: -0.5 m/s (up) if too low, 0.5 m/s (down) if too high
            vel_z = -0.5 if curr_h < target_height else 0.5

            # Send velocity command
            self.send_ned_velocity(0, 0, vel_z, 100)  # Move vertically for 1 second


        self.send_text(f"Timed out reaching height {target_height}m")
        return False

    def takeoff(self,takeoff_alt_meters): 
        # ******* TAKEOFF ******** 
        self._connection.mav.command_long_send(self._connection.target_system, self._connection.target_component,
                                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt_meters)
        msg = self._connection.recv_match(type='COMMAND_ACK',blocking=True)
        if msg.result != 0:
            self.send_text("Unable to Take off")
        else:
            self.send_text("Taking off...")


    def fly_gps_pos(self,lat,lon):
        self._check_guided_mode()
        # ******* Fly to lat long ********
        # in mission planner in full prarameter list, update WPNAV_SPEED !! to make sure we don't fly too fast
        # see https://www.youtube.com/watch?v=yyt4VjBRG_Y
        # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        self._connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, self._connection.target_system,
                                self._connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(lat * 10 ** 7), int(lon * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

        self.send_text("Flying to target...")
        # we notice we get a few zero wp_dist - so wait until we get non zero wp_dist 
        msg = self._connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        wp_dist = msg.wp_dist
        max_tries=0
        while max_tries<20 and wp_dist==0 and self._check_guided_mode():
            max_tries+=1
            # msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            msg = self._connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = msg.wp_dist
            # logger.debug(msg)
            self.send_text("distance to target:"+str(msg.wp_dist))

        # wait until we reach the wp (wp_dist=0)
        while wp_dist>0 and self._check_guided_mode():
            # msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            msg = self._connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            wp_dist = msg.wp_dist
            # logger.debug(msg)
            self.send_text("distance to target:"+str(msg.wp_dist))

        self.send_text("Target reached !")


    def fly_by_angles(self,roll_angle,pitch_angle,yaw_angle,duration,send_text=True):
        self._check_guided_mode()
        if send_text:
            self.send_text(f"ATT chnage r:{roll_angle},p:{pitch_angle}")
        # ******* Fly to using set attitude - doesnt requier GPS ********
        # e.g. to fly forward, pitch_angle=-5
        # see https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#set-attitude-target
        # see https://github.com/dronekit/dronekit-python/blob/master/examples/set_attitude_target/set_attitude_target.py
        q = to_quaternion(roll_angle, pitch_angle, yaw_angle) # Quaternion
        self._connection.mav.send(mavutil.mavlink.MAVLink_set_attitude_target_message(10, self._connection.target_system,
                                self._connection.target_component, int(0b00000111), q, 0,0,0,0.5))

        start = time.time()
        while time.time() - start < duration:
            #self.send_text("attitude for "+str(int(duration - (time.time() - start)))+" sec")
            self._connection.mav.send(mavutil.mavlink.MAVLink_set_attitude_target_message(10, self._connection.target_system,
                                    self._connection.target_component, int(0b00000111), q, 0,0,0,0.5))
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        q = to_quaternion(0,0,0) # Quaternion
        self._connection.mav.send(mavutil.mavlink.MAVLink_set_attitude_target_message(10, self._connection.target_system,
                                self._connection.target_component, int(0b00000111), q, 0,0,0,0.5))



    def land(self):
        # ******* Land ********
        self._connection.mav.command_long_send(self._connection.target_system, self._connection.target_component,
                                                mavutil.mavlink.MAV_CMD_NAV_LAND , 0, 0, 0, 0, 0, 0, 0, 0)
        msg = self._connection.recv_match(type='COMMAND_ACK',blocking=True)
        if msg.result != 0:
            self.send_text("Unable to land")
        else:
            self.send_text("Landing...")
    
    def lock_heading(self, heading_degrees):
        self._connection.mav.command_long_send(
            self._connection.target_system,
            self._connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading_degrees,  # target angle (0=North, 90=East, 180=South, 270=West)
            10,               # rotation speed deg/s
            1,                # direction: 1=CW, -1=CCW
            0,                # 0=absolute heading
            0, 0, 0
        )

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        NED frame: +x is North (forward), +y is East (right), +z is Down (down).
        DURATION IN 1/100 SEC, SO 100 = 1 SECOND
        """
        
        if velocity_z <0 and self.get_curr_height()> MAX_ALLOWED_ALT:
            velocity_z = 0 # This is to prevent the drone from flying too high during the search phase, which can cause it
            self.send_text(f"Moving up at {abs(velocity_z)} m/s")
            logger.info(f"reach max altitude, stop moving up and maintain current altitude")

            if (velocity_x == 0 and velocity_y ==0 and velocity_z ==0):
                return

        att = self._connection.recv_match(type='ATTITUDE', blocking=True)
        current_yaw = att.yaw  # radians, passed directly into the message

        msg = self._connection.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            self._connection.target_system,
            self._connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            0b0000101111000111,  # velocity + yaw enabled; bit 10 cleared = use yaw field
            0, 0, 0,             # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # velocities in m/s
            0, 0, 0,             # acceleration (not used)
            current_yaw, 0       # hold current heading, yaw_rate=0
        )
        for _ in range(duration):
            self._connection.mav.send(msg)
            time.sleep(0.01)
        msg = self._connection.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            self._connection.target_system,
            self._connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            0b0000101111000111,  # velocity + yaw enabled; bit 10 cleared = use yaw field
            0, 0, 0,             # x, y, z positions (not used)
            0, 0, 0,  # velocities in m/s
            0, 0, 0,             # acceleration (not used)
            current_yaw, 0       # hold current heading, yaw_rate=0
        )
        self._connection.mav.send(msg)
        time.sleep(0.01)

            
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]