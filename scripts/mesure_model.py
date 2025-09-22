#!/usr/bin/env python3
"""Measure Tesla Model 3 dimensions inside a running CARLA simulation."""

import json
import math
import os
import time
from pathlib import Path
from typing import Iterable, List, Tuple

import carla

HOST = "172.22.208.1"
PORT = 2000
TIMEOUT_SEC = 5.0
EPS = 1e-6
MODEL_BLUEPRINT = "vehicle.tesla.model3"
CARLA_OBJECTS_PATH = Path(__file__).resolve().parents[1] / "config/carla/carla_objects.json"
DEFAULT_ROLE_NAME = "ego_vehicle"
FIND_RETRY_ATTEMPTS = 20
FIND_RETRY_DELAY_SEC = 0.5
WHEEL_POSITION_SCALE = 0.01  # CARLA WheelPhysicsControl positions are in centimeters
DEBUG = os.environ.get("MESURE_MODEL_DEBUG", "0") not in {"0", "", "false", "False"}



def _resolve_role_name(default: str = DEFAULT_ROLE_NAME) -> str:
    """Return the configured role_name for the Tesla Model 3."""

    try:
        with CARLA_OBJECTS_PATH.open("r", encoding="utf-8") as cfg_file:
            data = json.load(cfg_file)
        for obj in data.get("objects", []):
            if obj.get("type") == MODEL_BLUEPRINT and obj.get("id"):
                return str(obj["id"])
    except FileNotFoundError:
        print(f"[warn] CARLA objects file not found at {CARLA_OBJECTS_PATH}, using default role_name '{default}'")
    except json.JSONDecodeError as exc:
        print(f"[warn] Failed to parse {CARLA_OBJECTS_PATH}: {exc}. Using default role_name '{default}'")
    except Exception as exc:
        print(f"[warn] Unexpected error reading {CARLA_OBJECTS_PATH}: {exc}. Using default role_name '{default}'")
    return default


def find_model_3(
    world: carla.World,
    attempts: int = FIND_RETRY_ATTEMPTS,
    delay_sec: float = FIND_RETRY_DELAY_SEC,
) -> carla.Vehicle:
    role_name = _resolve_role_name()
    last_seen = []

    for attempt in range(1, attempts + 1):
        try:
            vehicles = [actor for actor in world.get_actors().filter("vehicle.*") if actor.is_alive]
        except Exception as exc:
            if attempt == attempts:
                raise RuntimeError(f"Failed to query CARLA actors: {exc}") from exc
            print(
                f"[warn] Failed to query CARLA actors on attempt {attempt}/{attempts}: {exc}. Retrying in {delay_sec:.1f}s..."
            )
            time.sleep(delay_sec)
            continue

        if vehicles:
            last_seen = [
                (actor.id, actor.type_id, actor.attributes.get("role_name"))
                for actor in vehicles
            ]

        by_role = [actor for actor in vehicles if actor.attributes.get("role_name") == role_name]
        if by_role:
            if len(by_role) > 1:
                print(
                    f"[info] Found {len(by_role)} vehicles with role_name '{role_name}', using the first (id={by_role[0].id})"
                )
            return by_role[0]

        by_model = [actor for actor in vehicles if actor.type_id == MODEL_BLUEPRINT]
        if by_model:
            if len(by_model) > 1:
                print(
                    f"[info] Found {len(by_model)} Model 3 vehicles, using the first one (id={by_model[0].id})"
                )
            return by_model[0]

        if attempt < attempts:
            if vehicles:
                print(
                    f"[info] Vehicles present but no {MODEL_BLUEPRINT} (role_name '{role_name}') yet. Attempt {attempt}/{attempts}; retrying in {delay_sec:.1f}s..."
                )
            else:
                print(
                    f"[info] No vehicles found in world (attempt {attempt}/{attempts}). Retrying in {delay_sec:.1f}s..."
                )
            time.sleep(delay_sec)

    if last_seen:
        details = ", ".join(
            f"id={actor_id} type={type_id} role={role or '<none>'}"
            for actor_id, type_id, role in last_seen
        )
        raise RuntimeError(
            f"Could not find a {MODEL_BLUEPRINT} actor or a vehicle with role_name '{role_name}' after {attempts} attempts. "
            f"Last seen vehicles: {details}"
        )

    raise RuntimeError(
        f"Could not find a {MODEL_BLUEPRINT} actor or a vehicle with role_name '{role_name}' after {attempts} attempts; "
        "no vehicles were present in the world."
    )
def split_axles(wheel_xy: Iterable[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    points = list(wheel_xy)
    if len(points) < 2:
        raise RuntimeError("Need at least two wheels to split into axles")

    sorted_by_x = sorted(points, key=lambda xy: xy[0])
    half = len(sorted_by_x) // 2
    if half == 0 or half == len(sorted_by_x):
        raise RuntimeError("Could not split wheels into front/rear axles")

    rear = sorted_by_x[:half]
    front = sorted_by_x[half:]
    if front and rear and front[0][0] < rear[-1][0]:
        # All x values identical—warn so downstream logic can catch near-zero wheelbase
        print("[warn] Wheel x-positions overlap between axles; wheelbase may be inaccurate")
    return front, rear


def split_sides(axle_xy: Iterable[Tuple[float, float]]) -> Tuple[float, float]:
    points = list(axle_xy)
    if len(points) < 2:
        raise RuntimeError("Need at least two wheels on the axle to measure track width")

    ys = [y for _, y in points]
    left = min(ys)
    right = max(ys)
    if math.isclose(left, right, abs_tol=EPS):
        raise RuntimeError("Left/right wheel positions are identical; track width is zero")
    return left, right


def world_to_vehicle_frame(vehicle: carla.Vehicle, scale: float = 1.0):
    tf = vehicle.get_transform()
    origin = tf.location
    yaw = math.radians(tf.rotation.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    def convert(vec) -> Tuple[float, float]:
        if hasattr(vec, "x") and hasattr(vec, "y"):
            vx = float(vec.x)
            vy = float(vec.y)
        else:
            vx, vy = vec
        vx *= scale
        vy *= scale
        dx = vx - origin.x
        dy = vy - origin.y
        body_x = cos_yaw * dx + sin_yaw * dy
        body_y = -sin_yaw * dx + cos_yaw * dy
        return body_x, body_y

    return convert


def main() -> None:
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT_SEC)
    world = client.get_world()
    for actor in world.get_actors().filter("vehicle.*"):
        print(actor.id, actor.type_id, actor.attributes.get("role_name"))

    vehicle = find_model_3(world)

    world.wait_for_tick()

    bbox = vehicle.bounding_box
    length = 2.0 * bbox.extent.x
    width = 2.0 * bbox.extent.y
    height = 2.0 * bbox.extent.z

    to_body = world_to_vehicle_frame(vehicle, scale=WHEEL_POSITION_SCALE)
    physics = vehicle.get_physics_control()
    wheel_xy: List[Tuple[float, float]] = []
    for idx, wheel in enumerate(physics.wheels):
        pos = getattr(wheel, "position", None)
        if pos is None:
            continue
        raw_x = float(pos.x)
        raw_y = float(pos.y)
        raw_z = float(pos.z)
        body_x, body_y = to_body((raw_x, raw_y))
        if DEBUG:
            world_x = raw_x * WHEEL_POSITION_SCALE
            world_y = raw_y * WHEEL_POSITION_SCALE
            world_z = raw_z * WHEEL_POSITION_SCALE
            print(
                "[debug] wheel {idx}: raw=({rx:.3f}, {ry:.3f}, {rz:.3f}) cm -> world=({wx:.3f}, {wy:.3f}, {wz:.3f}) m -> body=({bx:.3f}, {by:.3f}) m".format(
                    idx=idx,
                    rx=raw_x,
                    ry=raw_y,
                    rz=raw_z,
                    wx=world_x,
                    wy=world_y,
                    wz=world_z,
                    bx=body_x,
                    by=body_y,
                )
            )
        wheel_xy.append((body_x, body_y))

    if not wheel_xy:
        raise RuntimeError("Vehicle physics returned no wheel positions")

    front_axle, rear_axle = split_axles(wheel_xy)

    front_center = sum(x for x, _ in front_axle) / len(front_axle)
    rear_center = sum(x for x, _ in rear_axle) / len(rear_axle)
    wheelbase = abs(front_center - rear_center)

    front_left, front_right = split_sides(front_axle)
    rear_left, rear_right = split_sides(rear_axle)

    front_track = abs(front_right - front_left)
    rear_track = abs(rear_right - rear_left)

    print("[Tesla Model 3 in CARLA]")
    print(f"Bounding-box length : {length:.3f} m")
    print(f"Bounding-box width  : {width:.3f} m")
    print(f"Bounding-box height : {height:.3f} m")
    print(f"Wheelbase           : {wheelbase:.3f} m")
    print(f"Track width (front) : {front_track:.3f} m")
    print(f"Track width (rear)  : {rear_track:.3f} m")


if __name__ == "__main__":
    main()
