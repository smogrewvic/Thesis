import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
import TrafficGenerators.GenerateTraffic
from time import sleep
import multiprocessing

if __name__ == "__main__":
    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args = ("id_83",))
    # p1.start()
    p2 = multiprocessing.Process(target=TrafficGenerators.GenerateTraffic.main, args = (True,  # actor autopilot
                                                                                        30     # percentage of speed limit
                                                                                        ))
    # p2.start()
    p3 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(False,  # autopilot_on
                                                                         False,  # holonomic
                                                                         False,  # display apf
                                                                         False,  # display actors
                                                                         False  # display control system
                                                                         ))

    p1.start()
    p2.start()
    p3.start()

