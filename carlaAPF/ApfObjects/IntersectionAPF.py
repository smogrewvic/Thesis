import numpy as np
from ApfObjects.APF_Object import APF_Object


class IntersectionAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity, traffic_light_actor):
        super().__init__(potential_field_size, potential_field_granularity)
        self.width = 1 / potential_field_granularity
        self.length = 2.5 / potential_field_granularity

        self.traffic_light_actor = traffic_light_actor
        self.light_state = None


    def dynamic_APF(self, x, y):
        position = self.scaled_egocentric_state["position"]
        if x == int(position[0]) and y == int(position[1]):
            return 255
        else:
            return 0


    def set_light_state(self, state):
        self.light_state = state

    def get_light_state(self):
        return self.light_state

    def set_size(self, width, length):
        self.width = width
        self.length = length