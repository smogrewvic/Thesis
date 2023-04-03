import numpy as np
import carla
import collections
from PIL import Image


class SteeringController:
    def __init__(self, potential_field, granularity, actor):
        self.potential_field = potential_field
        self.granularity = granularity
        self.max_steering_angle = actor.get_physics_control().wheels[0].max_steer_angle

        self.low_pass_queue = collections.deque(maxlen=10)
        self.low_pass_queue.append(0)

    def set_apf(self, potential_field, granularity):
        self.potential_field = potential_field
        self.granularity = granularity

    def search_lowest_potential(self, search_radius=2):

        ego_x, ego_y = len(self.potential_field) // 2, len(self.potential_field[0]) // 2
        self.lowest_potential = {"value": float('inf'),
                                 "relative_position": [ego_x, ego_y],
                                 "absolute_position": [ego_x, ego_y],
                                 "angle": 0,
                                 "normalized_angle": 0}

        # search for lowest gradient
        search_box = search_radius / self.granularity

        for y in range(int(-search_box // 2), int(search_box // 2)):
            for x in range(int(-search_box // 2), 0):
                potential = self.potential_field[x + ego_x][y + ego_y]
                # print("checked", x + ego_x, y + ego_y)

                if potential < self.lowest_potential["value"]:
                    self.lowest_potential["value"] = potential
                    self.lowest_potential["relative_position"] = [-x, y]  # indexed from top left of 2d apf
                    self.lowest_potential["absolute_position"] = [x + ego_x, y + ego_y]

        vector1 = [0, 1]
        vector2 = self.lowest_potential["relative_position"]
        self.lowest_potential["angle"] = np.degrees(
            np.pi / 2 - np.arccos(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))))

        steering_angle = self.lowest_potential["angle"]

        # normalize steering angle to -1 to 1   y=(x-a)/(b-a)*(d-c)+c
        low = -1
        high = 1
        normalized_steering_angle = (steering_angle + self.max_steering_angle) / (2 * self.max_steering_angle) * (high - low) + low
        normalized_steering_angle = self.low_pass_filter(normalized_steering_angle)  # smooth input

        self.lowest_potential["normalized_angle"] = normalized_steering_angle

        return self.lowest_potential

    def low_pass_filter(self, value, alpha=0.1):

        filtered_value = value * (1 - alpha) + self.low_pass_queue[-1] * alpha
        self.low_pass_queue.append(filtered_value)
        return filtered_value

    # def get_steering_info(self, actor):
    #
    #
    #     return normalized_steering_angle,  # carla doesn't seem to allow me to steer from here

    def draw_steering_apf(self):
        grayscale = np.array(self.potential_field, dtype=np.uint8)

        steering_point = self.search_lowest_potential()["absolute_position"]

        grayscale[steering_point[0], steering_point[1]] = 255

        apf_image = Image.fromarray(grayscale, mode="L")
        apf_image.save("APF_Image.bmp")
