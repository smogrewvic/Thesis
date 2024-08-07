import numpy as np
from ApfObjects.APF_Object import APF_Object



class NavpointAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__( potential_field_size, potential_field_granularity)
        self.safety_radius = 10 / potential_field_granularity

    def dynamic_APF(self, x, y):
        return 0
