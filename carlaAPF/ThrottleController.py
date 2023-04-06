import numpy as np
import carla
import collections
from PIL import Image


class ThrottleController:
    def __init__(self, potential_field, granularity):
        self.potential_field = potential_field
        self.granularity = granularity
        self.gradient_field = np.zeros((len(potential_field), len(potential_field[0])))  # 2D field + [grad x, grad y, grad z, norm]

    def calculate_total_gradient_field(self):
        ego_x = len(self.potential_field // 2)
        ego_y = len(self.potential_field[0] // 2)
        ego_z = self.potential_field[ego_x][ego_y]

        for y in range(len(self.potential_field)):
            for x in range(len(self.potential_field[0])):
                dx = x - ego_x
                dy = y - ego_y
                dz = self.potential_field[x][y] - ego_z
                norm = np.linalg.norm([dx, dy, dz])

                self.gradient_field[x][y] = [dx, dy, dz, norm]

    def get_gradient(self, position):
        ego_x = len(self.potential_field)//2
        ego_y = len(self.potential_field[0])//2
        ego_z = self.potential_field[ego_x][ego_y]

        dx = position[0] - ego_x
        dy = position[1] - ego_y
        dz = self.potential_field[position[0]][position[1]] - ego_z
        norm = np.linalg.norm([dx, dy, dz])

        return norm

    def get_throttle(self, steer_apf_position):

        throttle = min(self.get_gradient(steer_apf_position), 255)  # clip values at after 255

        # normalize throttle from -1 to 1 y=(x-a)/(b-a)*(d-c)+c
        throttle = min(throttle, 255)  # clip values at after 255
        low = 0.6  # inversed low high because math
        high = -1
        normalized_throttle = ((throttle + 0) / 255 * (high - low) + low)  #

        print("throttle", normalized_throttle)
        return normalized_throttle
