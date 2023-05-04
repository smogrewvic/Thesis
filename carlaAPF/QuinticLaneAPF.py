import numpy as np
import collections
from TransformMatrix import rotate2D, stretch2D
from APF_Object import APF_Object


class Quintic_Lane_APF():
    # def __init__(self, potential_field_size, potential_field_granularity):
    #     super().__init__(potential_field_size, potential_field_granularity)
    #     self.navpoints = []
    def __init__(self):
        self.navpoints = []
    def set_navpoints(self, navpoints):
        self.navpoints = navpoints


    def closest_navpoint_index(self):

        closest_index = 0
        closest_distance = float('inf')
        for i in range(len(self.navpoints)):
            distance = np.linalg.norm(self.navpoints[i].get_relative_state()["position"])
            if closest_distance > distance:
                closest_distance = distance
                closest_index = i

        return closest_index
    def quartic_regression_coefficients(self):
        closest_index = self.closest_navpoint_index()

        # perform quartic regression on 5 points
        start = closest_index - 1
        end = closest_index + 4

        if closest_index - 1 < 1:
            start = 0
        if closest_index + 4 > len(self.navpoints):
            end = len(self.navpoints)
    def dynamic_APF(self, x, y):

        coeffs = self.quartic_regression_coefficients()
        fy = coeffs[0] * y ^ 4 + coeffs[1] * y ^ 3 + coeffs[2] * y ^ 2 + coeffs[3] * y + coeffs[4]
        dfy = 4*coeffs[0] * y ^ 3 + 3*coeffs[1] * y ^ 2 + 2*coeffs[2] * y + coeffs[3]

        fxy = ((fy + x)^2)/10

        return fxy