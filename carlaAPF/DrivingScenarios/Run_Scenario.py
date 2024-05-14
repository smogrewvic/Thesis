import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot

import multiprocessing
import TrafficGenerators.ScenarioBuilder
from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
from DrivingScenarios.Scenarios import Scenario
import time

import keyboard

if __name__ == "__main__":

    scenario = Scenario('pedestrian_crossing')

    cars = scenario.get_cars()
    pedestrians = scenario.get_pedestrians()
    origin = scenario.get_origin()
    destinations = scenario.get_destination()
    ego_delay = scenario.get_ego_delay()
    traffic_delay = scenario.get_traffic_delay()

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=(origin,))  # spawn point

    p2 = multiprocessing.Process(target=TrafficGenerators.ScenarioBuilder.main, args=(cars,
                                                                                      pedestrians,
                                                                                      True,  # autopilot
                                                                                      100,  # percent of max speed limit
                                                                                      0.005))  # sim timestep    0.005 slow mo,  0.001 super slow mo

    p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(destinations, # destination
                                                                         True,  # autopilot_on
                                                                         True,  # display apf
                                                                         False,  # display control system and actor positions
                                                                         'type_2'))  # svo estimation type ('none', 'generic' 'type_1', 'type_2'

    p1.start()

    # time.sleep(traffic_delay)
    p2.start()

    # time.sleep(ego_delay)
    p3.start()

