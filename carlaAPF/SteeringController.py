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

    def low_pass_filter(self, value, alpha=0.5):

        filtered_value = value * (1 - alpha) + self.low_pass_queue[-1] * alpha
        self.low_pass_queue.append(filtered_value)
        return filtered_value

    # def get_steering_info(self, actor):
    #
    #
    #     return normalized_steering_angle,  # carla doesn't seem to allow me to steer from here

    # def draw_steering_apf(self):
    #     grayscale = np.array(self.potential_field, dtype=np.uint8)
    #
    #     steering_point = self.search_lowest_potential()["absolute_position"]
    #
    #     grayscale[steering_point[0], steering_point[1]] = 255
    #
    #     apf_image = Image.fromarray(grayscale, mode="L")
    #     apf_image.save("APF_Image.bmp")
