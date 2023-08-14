import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
import TrafficGenerators.GenerateTraffic
import TrafficGenerators.ActorSpawn
from time import sleep
import multiprocessing
import carla


def convert_point_to_carla_transform(x, y, z, pitch, roll, yaw):
    return carla.Transform(carla.Location(x=x, y=y, z=z),
                           carla.Rotation(pitch=pitch, yaw=roll, roll=yaw))


if __name__ == "__main__":
    car1 = [-28.581730, 140.535553, 0.600000, 0.000000, 0.352127, 0.000000]  # x,y,z,pitch,roll,yaw
    car_spawn_points = [car1]

    pedestrian1 = [-24.581730, 143.535553, 0.600000, 0.000000, 0.352127, 0.000000]  # x,y,z,pitch,roll,yaw
    pedestrian_spawn_points = [pedestrian1]

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args = ("easy_track",))
    p1.start()
    p2 = multiprocessing.Process(target=EgoVehicle.Autopilot.main)
    p2.start()
    p3 = multiprocessing.Process(target=TrafficGenerators.ActorSpawn.main, args=(car_spawn_points, pedestrian_spawn_points))
    p3.start()

    p1.join()
    p2.join()
    p3.join()
