import carla
import math
import random
import keyboard
import sys

import PotentialField as pf




def spectator_follow(view):
    if view == "third person":
        new_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
                                        ego_vehicle.get_transform().rotation)
        world.get_spectator().set_transform(new_transform)

    if view == "top":
        print(ego_vehicle.get_transform().rotation)
        # new_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(z=10)),
        #                                 carla.Rotation(pitch=-90, yaw = ego_vehicle.get_transform().rotation.yaw))
        # world.get_spectator().set_transform(new_transform)
        new_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
                                        ego_vehicle.get_transform().rotation)
        world.get_spectator().set_transform(new_transform)



if __name__ == '__main__':



    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    ego_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    spectator = world.get_spectator()
    transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)), ego_vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    ego_vehicle.set_autopilot(True)

    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_init_trans = carla.Transform(carla.Location(z=2))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to = ego_vehicle)

    # keyboard.on_press_key("z", lambda _: exit_application(world.get_actors()))

    # potential_field = pf.APF(ego_vehicle)

    while True:
        spectator_follow("top")
        # pf.APF.write_ego_data(ego_vehicle)
