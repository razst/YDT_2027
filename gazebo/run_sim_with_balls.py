import random
import shutil
import subprocess
import time
from pathlib import Path

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.transport import Node


WORLD_NAME = "iris_runway"
CREATE_SERVICE = f"/world/{WORLD_NAME}/create"
WORLD_SDF = Path(__file__).with_name("ydt_2027_iris.sdf")

MIN_BALLS = 10
MAX_BALLS = 30
SPAWN_INTERVAL_SECONDS = 2
BALL_RADIUS = 0.035
BALL_MASS_KG = 0.10
BALL_INERTIA_KG_M2 = 4.9e-5
MIN_SPACING = 0.4
FIELD_MIN = -8.0
FIELD_MAX = 8.0
FIELD_THIRD_WIDTH = (FIELD_MAX - FIELD_MIN) / 3
BALL_AREA_X_MIN = FIELD_MIN
BALL_AREA_X_MAX = BALL_AREA_X_MIN + FIELD_THIRD_WIDTH
BALL_AREA_Y_MIN = FIELD_MIN
BALL_AREA_Y_MAX = FIELD_MAX
INITIAL_POSITIONS = [(-7.25, -6.0), (-7.25, 0.0), (-7.25, 6.0)]
BALL_COLORS = {
    "red": (0.95, 0.15, 0.15),
    "blue": (0.15, 0.25, 0.95),
    "white": (0.95, 0.95, 0.95),
}


def random_positions(count: int) -> list[tuple[float, float]]:
    positions = INITIAL_POSITIONS.copy()

    for _ in range(count):
        for _ in range(10_000):
            position = (
                random.uniform(BALL_AREA_X_MIN, BALL_AREA_X_MAX),
                random.uniform(BALL_AREA_Y_MIN, BALL_AREA_Y_MAX),
            )
            if all(
                (position[0] - other[0]) ** 2 + (position[1] - other[1]) ** 2
                >= MIN_SPACING**2
                for other in positions
            ):
                positions.append(position)
                break
        else:
            raise RuntimeError("Could not find enough non-overlapping ball positions")

    return positions[len(INITIAL_POSITIONS) :]


def ball_sdf(
    name: str,
    position: tuple[float, float],
    color: tuple[float, float, float],
) -> str:
    x, y = position
    red, green, blue = color
    return f"""<sdf version="1.9">
  <model name="{name}">
    <pose>{x:.3f} {y:.3f} {BALL_RADIUS} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{BALL_MASS_KG}</mass>
        <inertia>
          <ixx>{BALL_INERTIA_KG_M2}</ixx>
          <iyy>{BALL_INERTIA_KG_M2}</iyy>
          <izz>{BALL_INERTIA_KG_M2}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>{BALL_RADIUS}</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{BALL_RADIUS}</radius></sphere></geometry>
        <material>
          <ambient>{red} {green} {blue} 1</ambient>
          <diffuse>{red} {green} {blue} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def wait_for_world(node: Node, process: subprocess.Popen[bytes] | None) -> None:
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if CREATE_SERVICE in node.service_list():
            return
        if process is not None and process.poll() is not None:
            raise RuntimeError(
                f"Gazebo exited before service {CREATE_SERVICE} became available"
            )
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for Gazebo service {CREATE_SERVICE}")


def spawn_balls(node: Node) -> None:
    count = random.randint(MIN_BALLS, MAX_BALLS)
    positions = random_positions(count)
    colors = list(BALL_COLORS.items())
    run_id = f"{random.getrandbits(32):08x}"
    print(f"Spawning {count} balls at random positions.", flush=True)

    for index, position in enumerate(positions, start=1):
        color_name, color = random.choice(colors)
        name = f"random_ball_{run_id}_{index:02d}"
        request = EntityFactory()
        request.sdf = ball_sdf(name, position, color)

        sent, response = node.request(
            CREATE_SERVICE, request, EntityFactory, Boolean, 5000
        )
        if not sent or not response.data:
            raise RuntimeError(f"Gazebo failed to create {name}")

        print(
            f"Created {name} ({color_name}) at "
            f"x={position[0]:.2f}, y={position[1]:.2f}",
            flush=True,
        )
        if index < count:
            time.sleep(SPAWN_INTERVAL_SECONDS)


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main() -> None:
    node = Node()
    process: subprocess.Popen[bytes] | None = None

    try:
        if CREATE_SERVICE not in node.service_list():
            gz = shutil.which("gz")
            if gz is None:
                raise RuntimeError("Could not find the Gazebo `gz` command")

            print(f"Starting Gazebo with {WORLD_SDF}", flush=True)
            process = subprocess.Popen(
                [gz, "sim", "-v4", "-r", str(WORLD_SDF)],
                cwd=WORLD_SDF.parent,
            )

        wait_for_world(node, process)
        spawn_balls(node)

        if process is not None:
            print("Balls are ready. Press Ctrl+C to stop Gazebo.", flush=True)
            process.wait()
            if process.returncode:
                raise RuntimeError(f"Gazebo exited with code {process.returncode}")
    except KeyboardInterrupt:
        print("Stopping Gazebo.", flush=True)
    finally:
        if process is not None:
            stop_process(process)


if __name__ == "__main__":
    main()
