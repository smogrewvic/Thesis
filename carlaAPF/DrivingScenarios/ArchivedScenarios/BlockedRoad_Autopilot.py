import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
from Tools.SpawnPoints import Spawn_Points
import TrafficGenerators.Archived.ActorSpawn_Behavior
import multiprocessing
import carla


def convert_point_to_carla_transform(x, y, z, pitch, roll, yaw):
    return carla.Transform(carla.Location(x=x, y=y, z=z),
                           carla.Rotation(pitch=pitch, yaw=roll, roll=yaw))


if __name__ == "__main__":
    car1 = Spawn_Points.points.value['id_64']
    car2 = Spawn_Points.points.value['id_88']
    car3 = Spawn_Points.points.value['id_54']
    car_spawn_points = [car1, car2, car3]

    pedestrian1 = [-24.581730, 143.535553, 0.600000, 0.000000, 0.352127, 0.000000]  # x,y,z,pitch,roll,yaw
    pedestrian_spawn_points = [pedestrian1]

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=('id_16',))


    p1.start()
    p2 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(True,  # autopilot_on
                                                                         False,  # holonomic
                                                                         True,  # display apf
                                                                         False,  # display actors
                                                                         True  # display control system
                                                                         ))
    p2.start()
    p3 = multiprocessing.Process(target=TrafficGenerators.Archived.ActorSpawn.main, args=(car_spawn_points, pedestrian_spawn_points, False))
    p3.start()

    p1.join()
    p2.join()
    p3.join()


    # p2 = multiprocessing.Process(target=TrafficGenerators.GenerateTraffic_Behavior.main, args = (True,  # autopilot state
    #                                                                                              30,    #percent speed limit
    #                                                                                              0.01)) #sim_timestep
    #
    # p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(True,  # autopilot_on
    #                                                                      False,  # holonomic
    #                                                                      True,  # display apf
    #                                                                      False,  # display actors
    #                                                                      True,  # display control system
    #                                                                      False   # ego position
    #                                                                      ))
    #
    # p1.start()
    # p2.start()
    # p3.start()
    #
