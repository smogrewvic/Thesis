import numpy as np
import collections
from TransformMatrix import rotate2D, stretch2D
from APF_Object import APF_Object


class VehicleAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 2
        self.length = 3
        self.safety_radius = 3/potential_field_granularity

    def dynamic_APF(self, x, y):
        # todo: remember to update relative state before calling
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed = self.relative_state["speed"]

        long_term = ((x*np.cos(theta)+y*np.sin(theta) - (i*np.cos(theta)+j*np.sin(theta)) - 0.25 * speed) / (self.length + speed)) ** 4
        lat_term = ((y*np.cos(theta)-x*np.sin(theta) - (j*np.cos(theta)-i*np.sin(theta))) / (self.width + 0.05 * speed)) ** 4
        term1 = long_term+lat_term
        term2 = (self.safety_radius + term1) / (self.safety_radius*0.01)
        discrete_apf = (abs(self.safety_radius - term1) + (self.safety_radius - term1)) / term2
        gain = 5000

        # if x == i and y == j: return 0
        return min(gain * discrete_apf, 255)


