import numpy as np
from ApfObjects.APF_Object import APF_Object


class VehicleAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 1 / potential_field_granularity
        self.length = 2.5 / potential_field_granularity
        self.safety_radius = 3/potential_field_granularity


    def dynamic_APF(self, x, y):
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed = self.relative_state["speed"]

        lat_term = ((x*np.cos(theta)+y*np.sin(theta) - (i*np.cos(theta)+j*np.sin(theta))-0.659*speed) / (2*self.length + speed)) ** 4
        long_term = ((y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) / (2*self.width + 0.01*speed)) ** 4
        exponent = lat_term+long_term

        potential = 255*np.e**(-exponent)  # gaussian eq defined from 0 to 255

        return potential