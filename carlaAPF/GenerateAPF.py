import PotentialField as pf
import carla
import time
from multiprocessing import Pool
import os

if __name__ == '__main__':
    # client = carla.Client('localhost', 2000)
    # world = client.get_world()
    potential_field = pf.APF()

    while True:
        # potential_field.set_actor_states(world.get_actors())
        potential_field.generate_APF()
        potential_field.save_image_APF()
        potential_field.show_APF()

        # potential_field.plot_actor_positions()
