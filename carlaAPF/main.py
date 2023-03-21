import carla
import random


import PotentialField as pf

def spectator_follow(view):
    if view == "third person":
        new_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)),
                                        ego_vehicle.get_transform().rotation)
        world.get_spectator().set_transform(new_transform)

    if view == "top":
        # print(ego_vehicle.get_transform().rotation)

        new_transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(z=10)),
                                        carla.Rotation(pitch=-90, yaw = 0))
        world.get_spectator().set_transform(new_transform)



if __name__ == '__main__':



    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle_bp.set_attribute('role_name', 'ego_vehicle')
    ego_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    spectator = world.get_spectator()
    transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4, z=2.5)), ego_vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    # ego_vehicle.set_autopilot(True)

    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_init_trans = carla.Transform(carla.Location(z=2))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to = ego_vehicle)

    potential_field = pf.APF()

    try:
        client.get_world()
        while True:
            spectator_follow("top")
            print("heading", round(ego_vehicle.get_transform().rotation.yaw, 4))

    finally:
        ego_vehicle.destroy()