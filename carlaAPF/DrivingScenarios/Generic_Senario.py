import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
# import TrafficGenerators.GenerateTraffic_Behavior
from time import sleep
import multiprocessing
import TrafficGenerators.ScenarioBuilder
from TrafficGenerators.ActorInfo import VehicleInfo

if __name__ == "__main__":
    cars = []
    cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
    cars.append(VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
    cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)

    pedestrian_spawns = []

    # p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=("id_113",))  # 98, 113, 66
    p2 = multiprocessing.Process(target=TrafficGenerators.ScenarioBuilder.main, args=(cars,
                                                                                      pedestrian_spawns,
                                                                                      True,  # autopilot
                                                                                      100,  # percent speed limit
                                                                                      0.05))  # sim timestep    0.01 slow mo
    #
    # p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(False,  # autopilot_on
    #                                                                      False,  # holonomic
    #                                                                      True,  # display apf
    #                                                                      False,  # display actors
    #                                                                      False,  # display control system
    #                                                                      False  # ego position
    #                                                                      ))

    # p1.start()
    p2.start()
    # p3.start()
