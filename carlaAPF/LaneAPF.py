import numpy as np
from VehicleAPF import VehicleAPF


class LaneAPF(VehicleAPF):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__( potential_field_size, potential_field_granularity)
        self.safety_radius = 5 / potential_field_granularity
