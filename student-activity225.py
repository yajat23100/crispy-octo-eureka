import glob
import os
import sys
import time
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []

object_id = {"None": 0,
             "Buildings": 1,
             "Fences": 2,
             "Other": 3,
             "Pedestrians": 4,
             "Poles": 5,
             "RoadLines": 6,
             "Roads": 7,
             "Sidewalks": 8,
             "Vegetation": 9,
             "Vehicles": 10,
             "Wall": 11,
             "TrafficsSigns": 12,
             "Sky": 13,
             "Ground": 14,
             "Bridge": 15,
             "RailTrack": 16,
             "GuardRail": 17,
             "TrafficLight": 18,
             "Static": 19,
             "Dynamic": 20,
             "Water": 21,
             "Terrain": 22
             }
key_list = list(object_id.keys())
val_list = list(object_id.values())


def generate_lidar_blueprint(blueprint_library):
    lidar_blueprint = blueprint_library.find('sensor.lidar.ray_cast_semantic')
    lidar_blueprint.set_attribute('channels', str(64))
    lidar_blueprint.set_attribute('points_per_second', str(56000))
    lidar_blueprint.set_attribute('rotation_frequency', str(40))
    lidar_blueprint.set_attribute('range', str(100))
    return lidar_blueprint


#write the code for generate_radar_blueprint function


def check_traffic_lights():
    if dropped_vehicle.is_at_traffic_light():
        traffic_light = dropped_vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            print(traffic_light.get_state())
            dropped_vehicle.apply_control(carla.VehicleControl(hand_brake=True))
    else:
        dropped_vehicle.apply_control(carla.VehicleControl(throttle=0.51))


def semantic_lidar_data1(point_cloud_data):
    distance_name_data = {}
    for detection in point_cloud_data:
        position = val_list.index(detection.object_tag)
        distance = math.sqrt((detection.point.x ** 2) + (detection.point.y ** 2) + (detection.point.z ** 2))
        distance_name_data["distance"] = distance
        print("LIDAR distance:", distance_name_data["distance"])
        distance_name_data["name"] = key_list[position]
        for i in object_id:
            if distance_name_data['name'] == i and distance_name_data["distance"] > 3 and distance_name_data[
                "distance"] < 6:

                dropped_vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.Brake |
                                                                        carla.VehicleLightState.LeftBlinker |
                                                                        carla.VehicleLightState.LowBeam))

                print(f"Object near by our car: {i} and the distance is: {distance_name_data['distance']}")
                break

            else:
                print(f"Object near by our car: {i} and the distance is: {distance_name_data['distance']}")


#define radar_callback function here


try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    get_blueprint_of_world = world.get_blueprint_library()
    car_model = get_blueprint_of_world.filter('model3')[0]
    spawn_point = (world.get_map().get_spawn_points()[57])
    dropped_vehicle = world.spawn_actor(car_model, spawn_point)

    simulator_camera_location_rotation = carla.Transform(spawn_point.location, spawn_point.rotation)
    simulator_camera_location_rotation.location += spawn_point.get_forward_vector() * 30
    simulator_camera_location_rotation.rotation.yaw += 180
    simulator_camera_view = world.get_spectator()
    simulator_camera_view.set_transform(simulator_camera_location_rotation)
    actor_list.append(dropped_vehicle)

    radar_sensor = generate_radar_blueprint(get_blueprint_of_world)
    sensor_radar_spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.8))
    radar_sensor_data = world.spawn_actor(radar_sensor, sensor_radar_spawn_point, attach_to=dropped_vehicle)

    radar_sensor_data.listen(lambda radar_data: _Radar_callback(radar_data))

    lidar_sensor = generate_lidar_blueprint(get_blueprint_of_world)
    sensor_lidar_spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2.0),
                                               carla.Rotation(pitch=0.000000, yaw=90.0, roll=0.000000))
    lidar_sensor_data = world.spawn_actor(lidar_sensor, sensor_lidar_spawn_point, attach_to=dropped_vehicle)

    lidar_sensor_data.listen(lambda data2: semantic_lidar_data1(data2))

    check_traffic_lights()

    actor_list.append(radar_sensor_data)
    actor_list.append(lidar_sensor_data)
    check_traffic_lights()
    time.sleep(1000)
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
