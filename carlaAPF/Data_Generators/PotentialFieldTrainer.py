from ApfObjects.PotentialField import APF

import numpy as np
import carla

from PIL import Image
from ApfObjects.PedestrianAPF import PedestrianAPF
from ApfObjects.VehicleAPF import VehicleAPF
from ApfObjects.NavpointAPF import NavpointAPF
from ApfObjects.RegressionLaneAPF import Regression_Lane_APF
from ApfObjects.TrafficLightAPF import TrafficLightAPF
from Tools.Traffic_Light_Info import Traffic_Light_Info
from Tools.Crosswalk_Info import Crosswalk_Info
import cv2
import matplotlib.pyplot as plt


class APF_Trainer(APF):

    def __init__(self,field_size=20, granularity=0.6):
        super().__init__(field_size, granularity)
        self.local_actor_ids = []
        self.lane_coefficients = []
        self.local_navpoints = []
    def set_actor_APF(self):
        self.local_actor_ids = []
        self.update_actor_states()

        for id in self.actor_ids:
            if id == "ego_vehicle": continue  # ignore ego_vehicle APF

            # update egocentric actor state to center in APF relative to ego vehicle
            self.actor_ids[id].update_alternate_states(self.actor_ids["ego_vehicle"].get_state(),
                                                       len(self.potential_field) // 2, len(self.potential_field) // 2)

            distance = np.linalg.norm(self.actor_ids[id].get_relative_state()["position"])
            if abs(distance) >= self.field_size:  # skip actors out of field
                continue

            if type(self.actor_ids[id]) == NavpointAPF: continue  # skip navpoints after this point

            ignore_traffic_light = type(self.actor_ids[id]) == TrafficLightAPF and self.actor_ids[id].get_light_state() != carla.TrafficLightState.Red
            if ignore_traffic_light: continue

            for y in range(len(self.potential_field)):
                for x in range(len(self.potential_field[0])):
                    # indexed from top left
                    self.potential_field[-x - 1][y] = min(
                        self.potential_field[-x - 1][y] + self.actor_ids[id].dynamic_APF(x, y), 255)

            self.local_actor_ids.append(self.actor_ids[id])
        return self.local_actor_ids

    def set_lane_APF(self):
        # get all navpoint actors and send to laneAPF
        # todo: check that navpoints are stored in actor_ids in order
        lane = Regression_Lane_APF(self.field_size, self.field_granularity, self.actor_ids["ego_vehicle"].get_state())
        lane.set_navpoints(self.navpoint_actors)
        lane.update_lane()
        for y in range(len(self.potential_field)):
            for x in range(len(self.potential_field[0])):
                ## indexed from top left
                self.potential_field[-x - 1][y] = min(
                    self.potential_field[-x - 1][y] + lane.static_APF(x, y), 255)

        self.lane_coefficients, self.local_navpoints = lane.get_local_lane_data()
        return self.lane_coefficients, self.local_navpoints


    def get_input_training_data(self):
        return self.local_actor_ids, self.local_navpoints, self.lane_coefficients
