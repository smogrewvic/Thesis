import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot

import multiprocessing
import TrafficGenerators.ScenarioBuilder
from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
from DrivingScenarios.Scenarios import Scenario

if __name__ == "__main__":

    scenario = Scenario('pedestrian_crossing')

    cars = scenario.get_cars()
    pedestrians = scenario.get_pedestrians()
    origin = scenario.get_origin()
    destination = scenario.get_destination()

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=(origin,))  # spawn point         98, 113, 66
    p2 = multiprocessing.Process(target=TrafficGenerators.ScenarioBuilder.main, args=(cars,
                                                                                      pedestrians,
                                                                                      True,  # autopilot
                                                                                      100,  # percent of max speed limit
                                                                                      0.005))  # sim timestep    0.005 slow mo

    p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(destination, # destination
                                                                         True,  # autopilot_on
                                                                         True,  # display apf
                                                                         False,  # display actors
                                                                         False,  # display control system
                                                                         False))  # SVO override

    p1.start()
    p2.start()
    p3.start()































    # cars = []
    # cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
    # cars.append(VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
    # cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)
    #
    # pedestrians = []
    # pedestrians.append(PedestrianInfo(spawn_point_id='id_172', destination_point_id= 'id_754', model_category='adult', behavior_type='cooperative').data)
    # pedestrians.append(PedestrianInfo(spawn_point_id='id_52',  destination_point_id= 'id_172', model_category='adult', behavior_type='sadistic').data)
    # pedestrians.append(PedestrianInfo(spawn_point_id='id_13', model_category='adult', behavior_type='cooperative').data)