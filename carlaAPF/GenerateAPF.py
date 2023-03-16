import PotentialField as pf
import time
from multiprocessing import Pool
import os

if __name__ == '__main__':
    potential_field = pf.APF()

    while True:
        potential_field.generate_APF()
        potential_field.save_image_APF()
        potential_field.show_APF()
        potential_field.plot_actor_positions()
