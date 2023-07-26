import numpy as np
import warnings
import TransformMatrix

class Regression_Lane_APF():

    def __init__(self, potential_field_size, potential_field_granularity, ego_vehicle_state):
        self.potential_field_size = potential_field_size
        self.potential_field_granularity = potential_field_granularity
        self.navpoints = []
        self.coeffs = []
        self.ego_vehicle_state = ego_vehicle_state
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
    def regression_coefficients(self):
        closest_index = self.closest_navpoint_index()

        # perform quartic regression
        start = max(0,closest_index - 12)
        end = min(closest_index + 20, len(self.navpoints)-1)

        local_navpoints_x = []
        local_navpoints_y = []
        for i in range(start, end+1):

            local_navpoints_x.append(self.navpoints[i].get_scaled_egocentric_state()["position"][0])
            local_navpoints_y.append(self.navpoints[i].get_scaled_egocentric_state()["position"][1])

        print("start:end", start, end, "closest_index", closest_index)


        coeffs = []
        coeffs = np.polyfit(local_navpoints_x, local_navpoints_y, 4)
        # with warnings.catch_warnings():
        #     warnings.filterwarnings('error')
        #     try:
        #         coeffs = np.polyfit(local_navpoints_x, local_navpoints_y, 4)
        #     except np.RankWarning:
        #         print("caught warning")
        #         coeffs = np.polyfit(local_navpoints_x, local_navpoints_y, 4)

        return coeffs

    def update_lane(self):
        self.coeffs = self.regression_coefficients()


    def static_APF(self, x, y):

        # todo: remember to call update_lane() when drawing the apf

        fx = self.coeffs[0] * x**4 + self.coeffs[1] * x**3 + self.coeffs[2] * x**2 + self.coeffs[3] * x + self.coeffs[4]
        dfx = 4*self.coeffs[0] * x**3 + 3*self.coeffs[1] * x**2 + 2*self.coeffs[2] * x + self.coeffs[3]

        slope = -255/(self.potential_field_size/self.potential_field_granularity)*x + 255 #- y*np.sin(np.arctan(dfx))

        ### second order lane equation
        # fxy = ((y -fx)**2)/1 + slope

        ### Gaussian lane equation
        sigma = 10
        exponent = -((y-fx)**2)/(2*sigma**2)
        fxy = (1-np.e**exponent)*128 + slope

        return fxy