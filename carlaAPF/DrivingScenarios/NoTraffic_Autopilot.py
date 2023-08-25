import EgoVehicle.EgoVehicle_Pygame
import EgoVehicle.Autopilot
from time import sleep
import multiprocessing


if __name__ == "__main__":

    p1 = multiprocessing.Process(target=EgoVehicle.EgoVehicle_Pygame.main, args = ("id_83",))
    p1.start()
    # autopilot_on, holonomic, display_apf, display_actors, display_control_sys
    p2 = multiprocessing.Process(target=EgoVehicle.Autopilot.main, args= (True, False, True, False, False))
    p2.start()
    p1.join()
    p2.join()


