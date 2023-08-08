import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
from time import sleep
import multiprocessing


if __name__ == "__main__":

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main)
    p1.start()
    p2 = multiprocessing.Process(target=EgoVehicle.Autopilot.main)
    p2.start()
    p1.join()
    p2.join()


