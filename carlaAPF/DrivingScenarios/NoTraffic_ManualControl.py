import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
import TrafficGenerators.GenerateTraffic
from time import sleep
import multiprocessing

if __name__ == "__main__":
    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args=("simple_lane_change",))
    p1.start()
    p2 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args=(False, False, True, False, False))
    p2.start()

    p1.join()
    p2.join()
