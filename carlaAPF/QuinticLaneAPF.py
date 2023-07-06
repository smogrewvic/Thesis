import numpy as np
import warnings
import TransformMatrix

class Quintic_Lane_APF():

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

        # perform quartic regression on 5 points
        start = max(0,closest_index - 1)
        end = min(closest_index + 8, len(self.navpoints)-1)

        # if start < 1:
        #     start = 0
        # if end > len(self.navpoints):
        #     end = len(self.navpoints)

        local_navpoints_x = []
        local_navpoints_y = []
        for i in range(start, end+1):
            # x = self.navpoints[i].get_relative_state()["position"][0]
            # y = self.navpoints[i].get_relative_state()["position"][1]
            # ego_heading = np.radians(self.ego_vehicle_state["heading"])
            # x = x*np.cos(ego_heading) + y*np.sin(ego_heading)
            # y = y*np.cos(ego_heading) - x*np.sin(ego_heading)

            # x = self.navpoints[i].get_relative_state()["position"][0]
            # y = self.navpoints[i].get_relative_state()["position"][1]
            # x, y = TransformMatrix.rotate2D([x,y], self.ego_vehicle_state['heading'])

            # local_navpoints_x.append(x)
            # local_navpoints_y.append(y)
            #
            # local_navpoints_x.append(self.navpoints[i].get_egocentric_state()["position"][0])
            # local_navpoints_y.append(self.navpoints[i].get_egocentric_state()["position"][1])

            ## ORIGINAL ###
            local_navpoints_x.append(self.navpoints[i].get_relative_state()["position"][0])
            local_navpoints_y.append(self.navpoints[i].get_relative_state()["position"][1])



        print("start:end", start, end, "closest_index", closest_index)
        # print("local_x", local_navpoints_x)
        # print("local_y", local_navpoints_y)
        # print("\n ________ \n")

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
        x = x-(self.potential_field_size/self.potential_field_granularity)/2  # move to center of APF (using relative_state())
        y = y-(self.potential_field_size/self.potential_field_granularity)/2
        # todo: remember to call update_lane() when drawing the apf
        # ego_heading =np.radians(self.ego_vehicle_state["heading"])
        # x = x*np.cos(ego_heading) + y*np.sin(ego_heading)
        # y = y*np.cos(ego_heading) - x*np.sin(ego_heading)
        # x = x-(self.potential_field_size/self.potential_field_granularity)/2  # move to center of APF (using relative_state())
        # y = y-(self.potential_field_size/self.potential_field_granularity)/2

        fx = self.coeffs[0] * x**4 + self.coeffs[1] * x**3 + self.coeffs[2] * x**2 + self.coeffs[3] * x + self.coeffs[4]
        dfx = 4*self.coeffs[0] * x**3 + 3*self.coeffs[1] * x**2 + 2*self.coeffs[2] * x + self.coeffs[3]

        slope_center = (self.potential_field_size/self.potential_field_granularity)/2
        slope = -2*x+100 #- y*np.sin(np.arctan(dfx)) # -x-slope_center is a flat slope
        fxy = ((y -fx)**2)/1 + slope



        return fxy