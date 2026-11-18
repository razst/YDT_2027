import math
import subprocess
import threading
import time
from typing import TypeVar

from google.protobuf import text_format
from google.protobuf.message import Message
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.entity_pb2 import Entity
from gz.msgs.pose_pb2 import Pose
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node

from constants import MAV_COM, MAV_MSG_FREQ
from mavLink_handler import MavLinkHandler


WORLD_NAME = "iris_runway"
DRONE_MODEL_NAME = "iris_with_ardupilot"
PICKUP_BALL_NAME = "pickup_ball"
CARRIED_BALL_NAME = "carried_ball"
POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
CREATE_SERVICE = f"/world/{WORLD_NAME}/create"
REMOVE_SERVICE = f"/world/{WORLD_NAME}/remove"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

FORWARD_DISTANCE_METERS = 2.0
FLIGHT_ALTITUDE_METERS = 2.0
LANDING_DISTANCE_TOLERANCE_METERS = 0.5
CARRIED_BALL_OFFSET_METERS = 0.15
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
        self._picked_up = False
        self._node.subscribe(Pose_V, POSE_TOPIC, self._record_poses)

    def _record_poses(self, poses: Pose_V) -> None:
        for pose in poses.pose:
            if pose.name == DRONE_MODEL_NAME or pose.name.startswith(
                f"{DRONE_MODEL_NAME}::"
            ):
                with self._pose_lock:
                    self._drone_pose = Pose()
                    self._drone_pose.CopyFrom(pose)
                return

    def wait_for_world(self, timeout: float = 30) -> None:
        required_services = {CREATE_SERVICE, REMOVE_SERVICE, SET_POSE_SERVICE}
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
        ball_distance = math.hypot(
            drone_pose.position.x,
            drone_pose.position.y - FORWARD_DISTANCE_METERS,
        )
        if ball_distance > LANDING_DISTANCE_TOLERANCE_METERS:
            raise RuntimeError(
                f"Drone is {ball_distance:.2f}m from the pickup ball; "
                "refusing to collect it"
            )

        initial_pose = Pose()
        initial_pose.name = CARRIED_BALL_NAME
        initial_pose.position.CopyFrom(drone_pose.position)
        initial_pose.position.z -= CARRIED_BALL_OFFSET_METERS
        initial_pose.orientation.CopyFrom(drone_pose.orientation)

        create_request = EntityFactory()
        create_request.sdf = self._carried_ball_sdf(initial_pose)
        response = gazebo_request(CREATE_SERVICE, create_request, Boolean)
        if not response.data:
            raise RuntimeError("Gazebo failed to create the carried-ball visual")

        if not self._remove_model(PICKUP_BALL_NAME):
                if not self._remove_model(CARRIED_BALL_NAME):
                    raise RuntimeError(
                        "Gazebo failed to remove the pickup ball and its carried-ball visual"
                    )
                raise RuntimeError("Gazebo failed to remove the ground pickup ball")

        self._picked_up = True
        self.update_carried_ball()

    def update_carried_ball(self) -> None:
        if not self._picked_up:
            raise RuntimeError("The pickup ball has not been collected")

        drone_pose = self.wait_for_drone_pose(timeout=2)
        carried_pose = Pose()
        carried_pose.name = CARRIED_BALL_NAME
        carried_pose.position.CopyFrom(drone_pose.position)
        carried_pose.position.z -= CARRIED_BALL_OFFSET_METERS
        carried_pose.orientation.CopyFrom(drone_pose.orientation)

        response = gazebo_request(SET_POSE_SERVICE, carried_pose, Boolean, 1000)
        if not response.data:
            raise RuntimeError("Gazebo failed to update the carried-ball position")

    @staticmethod
    def _carried_ball_sdf(pose: Pose) -> str:
        return f"""<sdf version="1.9">
  <model name="{CARRIED_BALL_NAME}">
    <static>true</static>
    <pose>{pose.position.x} {pose.position.y} {pose.position.z} 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>0.035</radius></sphere></geometry>
        <material>
          <ambient>0.95 0.15 0.15 1</ambient>
          <diffuse>0.95 0.15 0.15 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

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


def climb_with_carried_ball(
    mavlink: MavLinkHandler,
    pickup: GazeboBallPickup,
    target_altitude: float,
    timeout: float = 30,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pickup.update_carried_ball()
        altitude = mavlink.get_curr_height(timeout=0.2)
        if altitude is not None and altitude >= target_altitude - 0.2:
            pickup.update_carried_ball()
            return
        time.sleep(0.1)
    raise TimeoutError(f"Drone did not climb back to {target_altitude:.1f}m")


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
    mavlink.change_flight_mode(4)
    mavlink.arm()
    mavlink.takeoff(FLIGHT_ALTITUDE_METERS, wait_until_reached=True, tolerance=0.2)
    mavlink.fly_gps_pos(
        target_latitude,
        target_longitude,
        altitude_meters=FLIGHT_ALTITUDE_METERS,
        tolerance=LANDING_DISTANCE_TOLERANCE_METERS,
        timeout=60,
    )
    mavlink.land(wait_until_reached=True)

    pickup.collect_ball()
    mavlink.change_flight_mode(4)
    if not mavlink.is_armed():
        mavlink.arm()
    mavlink.takeoff(FLIGHT_ALTITUDE_METERS)
    climb_with_carried_ball(mavlink, pickup, FLIGHT_ALTITUDE_METERS)
    print("Ball collected; drone climbed back to 2m with the ball.", flush=True)


if __name__ == "__main__":
    main()
