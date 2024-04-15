import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot

import multiprocessing
import TrafficGenerators.ScenarioBuilder
from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
from DrivingScenarios.Scenarios import Scenario

import keyboard

if __name__ == "__main__":

    scenario = Scenario('basic_merge')

    cars = scenario.get_cars()
    pedestrians = scenario.get_pedestrians()
    origin = scenario.get_origin()
    destination = scenario.get_destination()

    results = multiprocessing.Queue()
    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=(origin,))  # spawn point

    p2 = multiprocessing.Process(target=TrafficGenerators.ScenarioBuilder.main, args=(cars,
                                                                                      pedestrians,
                                                                                      True,  # autopilot
                                                                                      100,  # percent of max speed limit
                                                                                      0.005))  # sim timestep    0.005 slow mo

    p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(destination, # destination
                                                                         True,  # autopilot_on
                                                                         True,  # display apf
                                                                         False,  # display control system and actor positions
                                                                         'none',
                                                                         results))  # svo estimation type ('none', 'generic' 'type_1', 'type_2'

    p1.start()
    p2.start()
    p3.start()

    # keyboard.wait('q')
    # p1.terminate()
    # p2.terminate()
    # p3.terminate()

    plot_data = []
    while not results.empty():
        plot_data.append(results.get())

    print(plot_data)
