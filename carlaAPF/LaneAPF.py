import numpy as np
from APF_Object import APF_Object


class LaneAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__( potential_field_size, potential_field_granularity)
        self.safety_radius = 5 / potential_field_granularity

    def static_APF(self, x, y):
        return 0
