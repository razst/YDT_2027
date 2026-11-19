import math
import threading
import time

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_pb2 import Entity
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.pose_pb2 import Pose
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node

from constants import MAV_COM, MAV_MSG_FREQ
from mavLink_handler import FlightMode, MavLinkHandler


WORLD_NAME = "iris_runway"
DRONE_MODEL_NAME = "iris_with_ardupilot"
RED_BALL_PREFIX = "random_ball_red_"
POSE_TOPIC = f"/world/{WORLD_NAME}/dynamic_pose/info"
CREATE_SERVICE = f"/world/{WORLD_NAME}/create"
REMOVE_SERVICE = f"/world/{WORLD_NAME}/remove"
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"

EARTH_RADIUS_METERS = 6_371_000
FLIGHT_ALTITUDE_METERS = 2.0
LANDING_DISTANCE_TOLERANCE_METERS = 0.5
CARRIED_BALL_OFFSET_METERS = 0.15
RTL_TIMEOUT_SECONDS = 180


def world_xy_to_gps(
    origin_latitude: float,
    origin_longitude: float,
    x: float,
    y: float,
) -> tuple[float, float]:
    latitude = origin_latitude + math.degrees(y / EARTH_RADIUS_METERS)
    longitude = origin_longitude + math.degrees(
        x / (EARTH_RADIUS_METERS * math.cos(math.radians(origin_latitude)))
    )
    return latitude, longitude


def wait_for_gps_position(mavlink: MavLinkHandler) -> tuple[float, float, float]:
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        position = mavlink.get_curr_gps_position(timeout=1)
        if position is not None and position[0] != 0 and position[1] != 0:
            return position
    raise TimeoutError("Timed out waiting for the SITL GPS position")


class RandomRedBallPickup:
    def __init__(self) -> None:
        self._node = Node()
        self._pose_lock = threading.Lock()
        self._drone_pose: Pose | None = None
        self._red_ball_poses: dict[str, Pose] = {}
        self._carried_ball_name = f"carried_red_ball_{time.time_ns():x}"
        self._picked_up = False
        self._node.subscribe(Pose_V, POSE_TOPIC, self._record_poses)

    def _record_poses(self, poses: Pose_V) -> None:
        with self._pose_lock:
            for pose in poses.pose:
                if pose.name == DRONE_MODEL_NAME or pose.name.startswith(
                    f"{DRONE_MODEL_NAME}::"
                ):
                    self._drone_pose = Pose()
                    self._drone_pose.CopyFrom(pose)
                elif pose.name.startswith(RED_BALL_PREFIX):
                    ball_pose = Pose()
                    ball_pose.CopyFrom(pose)
                    self._red_ball_poses[pose.name] = ball_pose

    def wait_for_world(self, timeout: float = 30) -> None:
        required_services = {
            CREATE_SERVICE,
            REMOVE_SERVICE,
            SET_POSE_SERVICE,
        }
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

    def choose_nearest_red_ball(self) -> tuple[str, Pose]:
        drone_pose = self.wait_for_drone_pose()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            with self._pose_lock:
                if self._red_ball_poses:
                    break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                "No randomly spawned red ball is visible in Gazebo. "
                "Start gazebo/run_sim_with_balls.py first."
            )

        with self._pose_lock:
            ball_name, ball_pose = min(
                self._red_ball_poses.items(),
                key=lambda item: math.hypot(
                    item[1].position.x - drone_pose.position.x,
                    item[1].position.y - drone_pose.position.y,
                ),
            )
            selected_pose = Pose()
            selected_pose.CopyFrom(ball_pose)
            return ball_name, selected_pose

    def collect_ball(self, ball_name: str) -> None:
        drone_pose = self.wait_for_drone_pose()
        with self._pose_lock:
            ball_pose = self._red_ball_poses.get(ball_name)
            if ball_pose is None:
                raise RuntimeError(f"Target ball {ball_name} is no longer in Gazebo")
            ball_x = ball_pose.position.x
            ball_y = ball_pose.position.y

        distance = math.hypot(
            drone_pose.position.x - ball_x,
            drone_pose.position.y - ball_y,
        )
        if distance > LANDING_DISTANCE_TOLERANCE_METERS:
            raise RuntimeError(
                f"Drone is {distance:.2f}m from {ball_name}; refusing to collect it"
            )

        carried_pose = Pose()
        carried_pose.name = self._carried_ball_name
        carried_pose.position.CopyFrom(drone_pose.position)
        carried_pose.position.z -= CARRIED_BALL_OFFSET_METERS
        carried_pose.orientation.CopyFrom(drone_pose.orientation)

        create_request = EntityFactory()
        create_request.sdf = self._carried_ball_sdf(carried_pose)
        self._request(CREATE_SERVICE, create_request, EntityFactory)

        remove_request = Entity()
        remove_request.name = ball_name
        remove_request.type = Entity.MODEL
        try:
            self._request(REMOVE_SERVICE, remove_request, Entity)
        except RuntimeError as error:
            cleanup_request = Entity()
            cleanup_request.name = self._carried_ball_name
            cleanup_request.type = Entity.MODEL
            self._request(REMOVE_SERVICE, cleanup_request, Entity)
            raise RuntimeError(f"Failed to remove target ball {ball_name}") from error

        self._picked_up = True
        self.update_carried_ball()

    def update_carried_ball(self) -> None:
        if not self._picked_up:
            raise RuntimeError("The red ball has not been collected")

        drone_pose = self.wait_for_drone_pose(timeout=2)
        carried_pose = Pose()
        carried_pose.name = self._carried_ball_name
        carried_pose.position.CopyFrom(drone_pose.position)
        carried_pose.position.z -= CARRIED_BALL_OFFSET_METERS
        carried_pose.orientation.CopyFrom(drone_pose.orientation)
        self._request(SET_POSE_SERVICE, carried_pose, Pose, timeout_ms=1000)

    def _request(
        self,
        service: str,
        request: EntityFactory | Entity | Pose,
        request_type: type[EntityFactory] | type[Entity] | type[Pose],
        timeout_ms: int = 5000,
    ) -> None:
        sent, response = self._node.request(
            service, request, request_type, Boolean, timeout_ms
        )
        if not sent or not response.data:
            raise RuntimeError(f"Gazebo service {service} failed")

    def _carried_ball_sdf(self, pose: Pose) -> str:
        return f"""<sdf version="1.9">
  <model name="{self._carried_ball_name}">
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


def follow_ball_until_stopped(
    pickup: RandomRedBallPickup,
    stop_event: threading.Event,
    errors: list[Exception],
) -> None:
    while not stop_event.wait(0.1):
        try:
            pickup.update_carried_ball()
        except Exception as error:
            errors.append(error)
            stop_event.set()


def raise_if_follow_failed(errors: list[Exception]) -> None:
    if errors:
        raise RuntimeError("Gazebo could not keep the red ball attached") from errors[0]


def wait_for_rtl(
    mavlink: MavLinkHandler,
    pickup_errors: list[Exception],
    timeout: float = RTL_TIMEOUT_SECONDS,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raise_if_follow_failed(pickup_errors)
        if not mavlink.is_armed():
            return
        time.sleep(0.5)
    raise TimeoutError("SITL did not complete RTL and disarm before the timeout")


def main() -> None:
    if not MAV_COM.startswith("udp:127.0.0.1:"):
        raise RuntimeError(
            "This mission is simulation-only; configure MAV_COM to a local SITL UDP endpoint"
        )

    pickup = RandomRedBallPickup()
    pickup.wait_for_world()
    ball_name, ball_pose = pickup.choose_nearest_red_ball()

    mavlink = MavLinkHandler(MAV_COM, MAV_MSG_FREQ)
    home_latitude, home_longitude, _ = wait_for_gps_position(mavlink)
    target_latitude, target_longitude = world_xy_to_gps(
        home_latitude,
        home_longitude,
        ball_pose.position.x,
        ball_pose.position.y,
    )

    print(f"Flying to random red ball {ball_name}.", flush=True)
    mavlink.change_flight_mode(FlightMode.GUIDED)
    if not mavlink.is_armed():
        mavlink.arm()
    mavlink.takeoff(
        FLIGHT_ALTITUDE_METERS,
        wait_until_reached=True,
        tolerance=0.2,
    )
    mavlink.fly_gps_pos(
        target_latitude,
        target_longitude,
        altitude_meters=FLIGHT_ALTITUDE_METERS,
        tolerance=LANDING_DISTANCE_TOLERANCE_METERS,
        timeout=120,
    )
    mavlink.land(wait_until_reached=True, timeout=90)

    pickup.collect_ball(ball_name)
    print(f"Collected {ball_name}; taking off with the ball.", flush=True)

    stop_event = threading.Event()
    pickup_errors: list[Exception] = []
    follower = threading.Thread(
        target=follow_ball_until_stopped,
        args=(pickup, stop_event, pickup_errors),
        daemon=True,
    )
    try:
        mavlink.change_flight_mode(FlightMode.GUIDED)
        if not mavlink.is_armed():
            mavlink.arm()
        follower.start()
        mavlink.takeoff(
            FLIGHT_ALTITUDE_METERS,
            wait_until_reached=True,
            tolerance=0.2,
        )
        raise_if_follow_failed(pickup_errors)
        mavlink.change_flight_mode(FlightMode.RTL)
        print("RTL started with the red ball attached.", flush=True)
        wait_for_rtl(mavlink, pickup_errors)
        pickup.update_carried_ball()
        print("RTL complete; SITL disarmed with the red ball at home.", flush=True)
    finally:
        stop_event.set()
        if follower.is_alive():
            follower.join(timeout=2)


if __name__ == "__main__":
    main()
