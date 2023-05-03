import numpy as np
import collections
from TransformMatrix import rotate2D, stretch2D
from APF_Object import APF_Object


class VehicleAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 2
        self.length = 3
        self.safety_radius = 3

    def static_APF(self, x, y):
        # todo: remember to update relative state before calling
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        velocity = self.scaled_egocentric_state["speed"]

        term1 = (((x - i - 0.25 * velocity) / (self.length + velocity)) ** 4 + ((y - j) / (self.width + 0.05 * velocity)) ** 6)
        term2 = (self.safety_radius + term1) / (self.safety_radius*0.01)
        discrete_apf = (abs(self.safety_radius - term1) + (self.safety_radius - term1)) / term2
        gain = 5000
        print(gain*discrete_apf)
        return min(gain * discrete_apf, 255)


