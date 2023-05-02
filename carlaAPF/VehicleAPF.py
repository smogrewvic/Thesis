import numpy as np
import collections
from TransformMatrix import rotate2D, stretch2D
from APF_Object import APF_Object
class VehicleAPF(APF_Object):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
