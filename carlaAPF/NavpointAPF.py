import numpy as np
from APF_Object import APF_Object



class NavpointAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__( potential_field_size, potential_field_granularity)
        self.safety_radius = 1 / potential_field_granularity

    #
    # def static_APF(self, x, y):
    #
    #     # todo: remember to update relative state before calling
    #
    #     i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
    #     num = self.safety_radius - (((x-i)**2 + (y-j)**2)/self.safety_radius)
    #     denom = - self.safety_radius - (((x-i)**2 + (y-j)**2)/(0.01*self.safety_radius))
    #
    #     # max(+100*(abs(num)+num)/denom, -255)
    #     return max(+100*(abs(num)+num)/denom, -255)
