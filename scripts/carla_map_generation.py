#!/usr/bin/env python3
import random
import rospy
import carla
from parking_position import (
    parking_vehicle_locations_Town04,
    town04_bound,
)



def valid_vehicle(bp: carla.ActorBlueprint) -> bool:
    try:
        return int(bp.get_attribute('number_of_wheels')) == 4
    except Exception:
        return False


 


def spawn_static_vehicles(world: carla.World, num_static: int) -> list:
    blueprint_library = world.get_blueprint_library()
    vehicle_bps = [bp for bp in blueprint_library.filter('vehicle') if valid_vehicle(bp)]
    if not vehicle_bps:
        raise RuntimeError('No valid 4-wheel vehicle blueprints found')

    positions = parking_vehicle_locations_Town04
    num_to_spawn = min(num_static, len(positions))
    chosen_positions = random.sample(positions, k=num_to_spawn)

    yaw_candidates = [0.0, 180.0]
    spawned = []
    for pos in chosen_positions:
        rot = carla.Rotation(yaw=random.choice(yaw_candidates))
        transform = carla.Transform(pos, rot)
        bp = random.choice(vehicle_bps)
        npc = world.try_spawn_actor(bp, transform)
        if npc is not None:
            try:
                npc.set_simulate_physics(False)
            except Exception:
                pass
            spawned.append(npc)
    return spawned


def destroy_actors(actors: list) -> None:
    for actor in actors:
        try:
            actor.destroy()
        except Exception:
            pass


def set_bird_eye_view(world: carla.World, center: carla.Location = None, height: float = 60.0) -> None:
    """주차 구역 상공에서 내려다보는 시점으로 Spectator 설정.

    center가 None이면 parking_position.town04_bound의 중앙을 사용.
    """
    try:
        spectator = world.get_spectator()
        if center is None:
            cx = 0.5 * (town04_bound["x_min"] + town04_bound["x_max"])
            cy = 0.5 * (town04_bound["y_min"] + town04_bound["y_max"])
            center = carla.Location(x=cx, y=cy, z=height)
        else:
            center = carla.Location(x=center.x, y=center.y, z=height)
        spectator.set_transform(carla.Transform(center, carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)))
    except Exception:
        pass


def run():
    host = rospy.get_param('~host', '127.0.0.1')
    port = int(rospy.get_param('~port', 2000))
    num_static = int(rospy.get_param('~num_static', 16))
    seed = int(rospy.get_param('~seed', 0))
    timeout = float(rospy.get_param('~timeout', 5.0))
    unload_parked_layer = bool(rospy.get_param('~unload_parked_layer', True))
    exit_after_spawn = bool(rospy.get_param('~exit_after_spawn', False))
    world_ready_timeout = float(rospy.get_param('~world_ready_timeout', 30.0))
    bridge_service_timeout = float(rospy.get_param('~bridge_service_timeout', 60.0))
    expected_town = rospy.get_param('~expected_town', '')

    random.seed(seed)

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    # 1) 브리지 서비스 준비 대기 (브리지가 월드 설정을 끝냈는지 신호로 활용)
    try:
        rospy.loginfo('Waiting for CARLA bridge service /carla/get_blueprints (timeout=%.1fs)...', bridge_service_timeout)
        rospy.wait_for_service('/carla/get_blueprints', timeout=bridge_service_timeout)
    except Exception:
        rospy.logwarn('Bridge service /carla/get_blueprints not available within timeout; proceeding anyway.')

    rospy.loginfo('Using existing CARLA world from carla_ros_bridge (no map load).')
    world = client.get_world()
    deadline = rospy.Time.now().to_sec() + world_ready_timeout
    while not rospy.is_shutdown():
        try:
            cur_map = world.get_map()
            if cur_map is not None:
                # expected_town이 주어지면 맵 이름 일치까지 대기
                if expected_town:
                    name = cur_map.name
                    if name == expected_town:
                        break
                else:
                    break
        except Exception:
            pass
        if rospy.Time.now().to_sec() >= deadline:
            rospy.logwarn('Timeout waiting for CARLA world/map (expected_town=%s); proceeding anyway.', expected_town if expected_town else '(none)')
            break
        rospy.sleep(0.2)

    if unload_parked_layer:
        try:
            world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        except Exception:
            rospy.logwarn('Failed to unload parked vehicles layer')

    actors = []
    npcs = spawn_static_vehicles(world, num_static)
    actors.extend(npcs)

    rospy.loginfo('Spawned static vehicles=%d', len(npcs))

    # Bird's-eye view 시점 설정 (주차 구역 중심 상공)
    set_bird_eye_view(world)

    if exit_after_spawn:
        rospy.loginfo('exit_after_spawn=true: 노드를 종료하지만 CARLA 월드의 액터는 유지합니다.')
        return

    def _shutdown():
        rospy.loginfo('Shutting down. Destroying %d actors...', len(actors))
        destroy_actors(actors)
    rospy.on_shutdown(_shutdown)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('carla_map_generation')
    try:
        run()
    except Exception as e:
        rospy.logerr('Fatal error: %s', str(e))
        raise


