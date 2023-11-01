import numpy as np
from ApfObjects.APF_Object import APF_Object


class PedestrianAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 3
        self.length = 4
        # self.safety_radius = 5 / potential_field_granularity
        self.alpha = 0.2  # svo effect

    def dynamic_APF(self, x, y):
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed_factor = 2*abs(self.relative_state["speed"]) / 3.6  # distance traveled in 2 seconds from meters/second velocity
        sigma_x = (self.length + speed_factor)
        sigma_y = (self.width + 0.01 * speed_factor)

        long_term = (x * np.cos(theta) + y * np.sin(theta) - (i * np.cos(theta) + j * np.sin(theta)) - 0.659 * speed_factor) ** 2 / (2 * sigma_x ** 2)
        lat_term = (y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) ** 2 / (2 * sigma_y ** 2)
        exponent = long_term + lat_term

        potential = 255 * np.e ** (-exponent)  # defined from 0 to 255

        return potential

    def dynamic_APF_sigma_SVO(self, x, y, svo):
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed_factor = 2*abs(self.relative_state["speed"]) / 3.6  # distance traveled in 2 seconds from meters/second velocity
        sigma_x = (self.length + speed_factor) - self.alpha * svo
        sigma_y = (self.width + 0.01 * speed_factor) - self.alpha * svo

        long_term = (x * np.cos(theta) + y * np.sin(theta) - (i * np.cos(theta) + j * np.sin(theta)) - 0.659 * speed_factor) ** 2 / (2 * sigma_x ** 2)
        lat_term = (y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) ** 2 / (2 * sigma_y ** 2)
        exponent = long_term + lat_term

        potential = 255 * np.e ** (-exponent)  # defined from 0 to 255

        return potential

    def dynamic_APF_hann_SVO(self, x, y, svo):
        pass

    def dynamic_APF_gauss_SVO(self, x, y, svo):
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed_factor = 2*abs(self.relative_state["speed"]) / 3.6  # distance traveled in 2 seconds from meters/second velocity
        sigma_x = (self.length + speed_factor)
        sigma_y = (self.width + 0.01 * speed_factor)

        # Base APF Gaussian
        base_apf_long = (x * np.cos(theta) + y * np.sin(theta) - (i * np.cos(theta) + j * np.sin(theta)) - 0.659 * speed_factor) ** 2 / (2 * sigma_x ** 2)
        base_apf_lat = (y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) ** 2 / (2 * sigma_y ** 2)
        base_apf_exponent = base_apf_long + base_apf_lat

        # SVO Gaussian
        svo_long = (x * np.cos(theta) + y * np.sin(theta) - (i * np.cos(theta) + j * np.sin(theta)) - 0.659 * speed_factor) ** 2 / (2 * sigma_x ** 2)
        svo_lat = (y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) ** 2 / (2 * sigma_y ** 2)
        svo_exponent = svo_long + svo_lat

        base_apf = np.e ** (-base_apf_exponent)  # defined from 0 to 1
        svo_apf = np.e ** (-svo_exponent)

        gain = 255*(1-self.alpha)  # gain to define from 0-255
        potential = gain*(base_apf + svo_apf)

        return potential
