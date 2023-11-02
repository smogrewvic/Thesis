import numpy as np
from ApfObjects.APF_Object import APF_Object


class TrafficLightAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity, traffic_light_actor):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 3.5 / potential_field_granularity
        self.length = 1 / potential_field_granularity

        self.traffic_light_actor = traffic_light_actor
        self.light_state = None

    def dynamic_APF(self, x, y):
        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        theta = np.radians(self.relative_state["heading"])
        speed_factor = 2*abs(self.relative_state["speed"]) / 3.6  # distance traveled in 2 seconds from meters/second velocity
        sigma_x = (self.length + speed_factor)
        sigma_y = (self.width + 0.01 * speed_factor)

        long_term = (x * np.cos(theta) + y * np.sin(theta) - (i * np.cos(theta) + j * np.sin(theta)) - 0.659 * speed_factor) ** 2 / (2 * sigma_x ** 2)
        lat_term = (y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) ** 10 / (2 * sigma_y ** 10)
        exponent = long_term + lat_term

        potential = 255 * np.e ** (-exponent)  # defined from 0 to 255

        return potential





    # def dynamic_APF(self, x, y):
    #
    #     i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
    #     theta = np.radians(self.relative_state["heading"])
    #     speed = 2*abs(self.relative_state["speed"]) / 3.6  # distance traveled in 2 seconds from meters/second velocity
    #
    #     lat_term = ((x*np.cos(theta)+y*np.sin(theta) - (i*np.cos(theta)+j*np.sin(theta))-0.659*speed) / (2*self.length + speed)) ** 10
    #     long_term = ((y * np.cos(theta) - x * np.sin(theta) - (j * np.cos(theta) - i * np.sin(theta))) / (2*self.width + 0.01*speed)) ** 10
    #     exponent = lat_term+long_term
    #
    #     potential = 255*np.e**(-exponent)  # gaussian eq defined from 0 to 255
    #
    #     return potential

    def get_light_state(self):
        self.light_state = self.traffic_light_actor.get_state()
        return self.light_state

    def set_size(self, width, length):
        self.width = width
        self.length = length