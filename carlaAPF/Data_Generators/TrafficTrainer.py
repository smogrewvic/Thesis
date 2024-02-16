import EgoVehicle.EgoVehicle_Pygame
import Data_Generators.AutopilotTrainer
import Data_Generators.EgoVehiclePygameTrainer
import TrafficGenerators.GenerateTraffic_Behavior
from time import sleep
import multiprocessing


if __name__ == "__main__":


    p1 = multiprocessing.Process(target=Data_Generators.EgoVehiclePygameTrainer.main, args = ("id_98",)) #98, 113

    p2 = multiprocessing.Process(target=TrafficGenerators.GenerateTraffic_Behavior.main, args = (True,  # autopilot state
                                                                                                 30,    #percent speed limit
                                                                                                 0.01)) #sim_timestep

    p3 = multiprocessing.Process(target=Data_Generators.AutopilotTrainer.main, args=(True,  # autopilot_on
                                                                                     False,  # holonomic
                                                                                     True,  # display apf
                                                                                     False,  # display actors
                                                                                     False,  # display control system
                                                                                     False   # ego position
                                                                                     ))

    p1.start()
    p2.start()
    p3.start()

