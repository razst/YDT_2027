import math
import subprocess
import threading
import time
from typing import TypeVar

from google.protobuf import text_format
from google.protobuf.message import Message
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_pb2 import Entity
from gz.msgs.pose_pb2 import Pose
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node

from constants import MAV_COM, MAV_MSG_FREQ
from mavLink_handler import FlightMode, MavLinkHandler


WORLD_NAME = "iris_runway"
DRONE_MODEL_NAME = "iris_with_ardupilot"
PICKUP_BALL_NAME = "pickup_ball"
PICKUP_BALL_X_METERS = 0.0
PICKUP_BALL_Y_METERS = 2.0
PICKUP_BALL_Z_METERS = 0.035
POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
REMOVE_SERVICE = f"/world/{WORLD_NAME}/remove"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

FORWARD_DISTANCE_METERS = 2.0
FLIGHT_ALTITUDE_METERS = 2.0
FLIGHT_POSITION_TOLERANCE_METERS = 0.5
CAPTURE_ALIGNMENT_TOLERANCE_METERS = 0.10
BALL_CAVITY_OFFSET_Z_METERS = -0.105
BALL_ENTRY_STEPS = 12
EARTH_RADIUS_METERS = 6_371_000
ResponseMessage = TypeVar("ResponseMessage", bound=Message)


def gazebo_request(
    service: str,
    request: Message,
    response_type: type[ResponseMessage],
    timeout_ms: int = 5000,
) -> ResponseMessage:
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            request.DESCRIPTOR.full_name,
            "--reptype",
            response_type.DESCRIPTOR.full_name,
            "--timeout",
            str(timeout_ms),
            "--req",
            text_format.MessageToString(request),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_ms / 1000 + 2,
    )
    response = response_type()
    try:
        text_format.Parse(result.stdout, response)
    except text_format.ParseError as error:
        raise RuntimeError(
            f"Gazebo returned an invalid response for {service}"
        ) from error
    return response


def forward_gps_target(
    latitude: float, longitude: float, distance_meters: float
) -> tuple[float, float]:
    target_latitude = latitude + math.degrees(
        distance_meters / EARTH_RADIUS_METERS
    )
    return target_latitude, longitude


class GazeboBallPickup:
    def __init__(self) -> None:
        self._node = Node()
        self._pose_lock = threading.Lock()
        self._drone_pose: Pose | None = None
        self._node.subscribe(Pose_V, POSE_TOPIC, self._record_poses)

    def _record_poses(self, poses: Pose_V) -> None:
        for pose in poses.pose:
            if pose.name == DRONE_MODEL_NAME or pose.name.startswith(
                f"{DRONE_MODEL_NAME}::"
            ):
                with self._pose_lock:
                    self._drone_pose = Pose()
                    self._drone_pose.CopyFrom(pose)

    def wait_for_world(self, timeout: float = 30) -> None:
        required_services = {REMOVE_SERVICE, SET_POSE_SERVICE}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if required_services.issubset(set(self._node.service_list())):
                return
            time.sleep(0.2)
        raise TimeoutError(f"Gazebo world {WORLD_NAME} services did not become ready")

    def wait_for_drone_pose(self, timeout: float = 15) -> Pose:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._pose_lock:
                if self._drone_pose is not None:
                    pose = Pose()
                    pose.CopyFrom(self._drone_pose)
                    return pose
            time.sleep(0.1)
        raise TimeoutError(f"Gazebo did not publish the {DRONE_MODEL_NAME} pose")

    def collect_ball(self) -> None:
        drone_pose = self.wait_for_drone_pose()
        ball_pose = Pose()
        ball_pose.position.x = PICKUP_BALL_X_METERS
        ball_pose.position.y = PICKUP_BALL_Y_METERS
        ball_pose.position.z = PICKUP_BALL_Z_METERS
        ball_distance = math.hypot(
            drone_pose.position.x - ball_pose.position.x,
            drone_pose.position.y - ball_pose.position.y,
        )
        if ball_distance > CAPTURE_ALIGNMENT_TOLERANCE_METERS:
            raise RuntimeError(
                f"Drone is {ball_distance:.2f}m from the pickup ball; "
                "the catcher opening requires closer alignment"
            )

        target_pose = Pose()
        target_pose.name = PICKUP_BALL_NAME
        target_pose.position.x = drone_pose.position.x
        target_pose.position.y = drone_pose.position.y
        target_pose.position.z = (
            drone_pose.position.z + BALL_CAVITY_OFFSET_Z_METERS
        )
        target_pose.orientation.CopyFrom(ball_pose.orientation)
        for step in range(1, BALL_ENTRY_STEPS + 1):
            ratio = step / BALL_ENTRY_STEPS
            entering_pose = Pose()
            entering_pose.name = PICKUP_BALL_NAME
            entering_pose.position.x = (
                ball_pose.position.x
                + (target_pose.position.x - ball_pose.position.x) * ratio
            )
            entering_pose.position.y = (
                ball_pose.position.y
                + (target_pose.position.y - ball_pose.position.y) * ratio
            )
            entering_pose.position.z = (
                ball_pose.position.z
                + (target_pose.position.z - ball_pose.position.z) * ratio
            )
            entering_pose.orientation.CopyFrom(target_pose.orientation)
            response = gazebo_request(
                SET_POSE_SERVICE, entering_pose, Boolean, timeout_ms=1000
            )
            if not response.data:
                raise RuntimeError("Gazebo failed while moving the ball into the catcher")
            time.sleep(0.05)

        if not self._remove_model(PICKUP_BALL_NAME):
            raise RuntimeError("Gazebo failed to remove the pickup ball from the catcher")

    def _remove_model(self, name: str) -> bool:
        request = Entity()
        request.name = name
        request.type = Entity.MODEL
        response = gazebo_request(REMOVE_SERVICE, request, Boolean)
        return response.data


def wait_for_gps_position(mavlink: MavLinkHandler) -> tuple[float, float, float]:
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        position = mavlink.get_curr_gps_position(timeout=1)
        if position is not None and position[0] != 0 and position[1] != 0:
            return position
    raise TimeoutError("Timed out waiting for the SITL GPS position")


def main() -> None:
    if not MAV_COM.startswith("udp:127.0.0.1:"):
        raise RuntimeError(
            "This mission is simulation-only; configure MAV_COM to a local SITL UDP endpoint"
        )

    pickup = GazeboBallPickup()
    pickup.wait_for_world()
    pickup.wait_for_drone_pose()

    mavlink = MavLinkHandler(MAV_COM, MAV_MSG_FREQ)
    home_latitude, home_longitude, _ = wait_for_gps_position(mavlink)
    target_latitude, target_longitude = forward_gps_target(
        home_latitude, home_longitude, FORWARD_DISTANCE_METERS
    )

    print("Starting the SITL ball-collection mission.", flush=True)
    mavlink.change_flight_mode(FlightMode.GUIDED)
    mavlink.arm()
    mavlink.takeoff(FLIGHT_ALTITUDE_METERS, wait_until_reached=True, tolerance=0.2)
    mavlink.fly_gps_pos(
        target_latitude,
        target_longitude,
        altitude_meters=FLIGHT_ALTITUDE_METERS,
        tolerance=FLIGHT_POSITION_TOLERANCE_METERS,
        timeout=60,
    )
    mavlink.land(wait_until_reached=True)

    pickup.collect_ball()
    print("Ball disappeared into the catcher; taking off.", flush=True)
    mavlink.change_flight_mode(FlightMode.GUIDED)
    if not mavlink.is_armed():
        mavlink.arm()
    mavlink.takeoff(FLIGHT_ALTITUDE_METERS)
    print("Ball removed; drone taking off.", flush=True)


if __name__ == "__main__":
    main()
