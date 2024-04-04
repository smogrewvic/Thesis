import numpy as np


class Regression_Lane_APF():

    def __init__(self, potential_field_size, potential_field_granularity, ego_vehicle_state):
        self.potential_field_size = potential_field_size
        self.potential_field_granularity = potential_field_granularity
        self.navpoints = []
        self.local_navpoints = []  # for dnn training
        self.coeffs = []
        self.ego_vehicle_state = ego_vehicle_state
        self.sub_goal = []

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

    def regression_coefficients(self, poly_order=4):
        closest_index = self.closest_navpoint_index()

        # perform regression
        start = max(0, closest_index - 8)
        end = min(closest_index + 30, len(self.navpoints) - 1)


        self.local_navpoints = [] # for dnn training
        local_navpoints_x = []
        local_navpoints_y = []
        for i in range(start, end + 1):
            self.local_navpoints.append(self.navpoints[i])
            local_navpoints_x.append(self.navpoints[i].get_scaled_egocentric_state()["position"][0])
            local_navpoints_y.append(self.navpoints[i].get_scaled_egocentric_state()["position"][1])

        self.sub_goal = [local_navpoints_x[-1], local_navpoints_y[-1]]
        coeffs = np.polyfit(local_navpoints_x, local_navpoints_y, poly_order)


        return coeffs

    def update_lane(self):
        self.coeffs = self.regression_coefficients()

    def get_local_lane_data(self):
        return self.coeffs, self.local_navpoints  # remember to call update_lane

    def static_APF2(self, x, y):

        # todo: remember to call update_lane() when drawing the apf
        fx, dfx = 0, 0
        for i, c_i in enumerate(self.coeffs[::-1]):
            fx += c_i * x ** i

        for i, c_i in enumerate(self.coeffs[:-1][::-1]):
            dfx += c_i * x ** i

        slope = -128 / (self.potential_field_size / self.potential_field_granularity) * x + 128  # - y*np.sin(np.arctan(dfx))

        ### second order lane equation
        # fxy = ((y -fx)**2)/1 + slope

        ### Gaussian lane equation
        sigma = 1.5 / self.potential_field_granularity
        exponent = -((y - fx) ** 2) / (2 * sigma ** 2)
        fxy = (1 - np.e ** exponent) * 128 + slope

        return fxy

    def static_APF(self, x, y):

        # todo: remember to call update_lane() when drawing the apf
        fx, dfx = 0, 0
        for i, c_i in enumerate(self.coeffs[::-1]):
            fx += c_i * x ** i

        for i, c_i in enumerate(self.coeffs[:-1][::-1]):
            dfx += c_i * x ** i

        # slope = -128 / (self.potential_field_size / self.potential_field_granularity) * x + 128  # - y*np.sin(np.arctan(dfx))

        ### second order lane equation
        # fxy = ((y -fx)**2)/1 + slope

        ### Gaussian lane equation
        gain = 64
        sigma = 1.5 / self.potential_field_granularity
        exponent = -((y - fx) ** 2) / (2 * sigma ** 2)
        lane = gain*(1 - np.e ** exponent)

        a_width = 2.8/self.potential_field_granularity
        b_width = 2.8/self.potential_field_granularity
        # print('SUB_GOAL_XY', self.sub_goal)
        goal = ((x-self.sub_goal[0])/a_width)**2 + ((y-self.sub_goal[1])/b_width)**2


        # fxy = gain*(1 - np.e ** exponent)
        fxy = goal +lane
        return fxy
