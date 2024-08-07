import numpy as np
import collections
from Tools.TransformMatrix import rotate2D


class APF_Object:
    def __init__(self, potential_field_size, potential_field_granularity):
        self.potential_field_size = potential_field_size
        self.potential_field_granularity = potential_field_granularity
        self.safety_radius = 2/potential_field_granularity
        # self.data_log = collections.deque(maxlen=100)

        self.state = {"type": "",
                      "position": np.zeros(3),
                      "heading": 0,
                      "speed": 0,
                      "angular_velocity": np.zeros(3),
                      "acceleration": np.zeros(3),
                      "velocity": np.zeros(3)
                      }

        self.relative_state = {"type": "",
                               "position": np.zeros(3),
                               "heading": 0,
                               "speed": 0,
                               "angular_velocity": np.zeros(3),
                               "acceleration": np.zeros(3),
                               "velocity": np.zeros(3)}

        self.egocentric_state = {"type": "",
                               "position": np.zeros(3),
                               "heading": 0,
                               "speed": 0,
                               "angular_velocity": np.zeros(3),
                               "acceleration": np.zeros(3),
                               "velocity": np.zeros(3)}

        self.scaled_egocentric_state = {"type": "",
                               "position": np.zeros(3),
                               "heading": 0,
                               "speed": 0,
                               "angular_velocity": np.zeros(3),
                               "acceleration": np.zeros(3),
                               "velocity": np.zeros(3)}
    def calculate_relative_state(self, ego_vehicle_state):

        for key in self.state:
            if key == "type" or key == "speed":
                continue
            else:
                self.relative_state[key] = self.state[key] - ego_vehicle_state[key]

        self.relative_state["speed"] = np.linalg.norm(self.state["velocity"]-ego_vehicle_state["velocity"])

    def get_state(self):
        return self.state
    def set_state(self, state_data):
        self.state = state_data
    def get_relative_state(self):
        return self.relative_state

    def centered_state(self, absolute_state, center_x, center_y):

        return {"type": "",
                "position": np.array([center_x, center_y, absolute_state["position"][2]]),
                "heading": absolute_state["heading"], # todo: maybe 90 degs or 0 degs?
                "speed": absolute_state["speed"],
                "angular_velocity": absolute_state["angular_velocity"],
                "acceleration": absolute_state["acceleration"],
                "velocity": absolute_state["velocity"]}

    def update_alternate_states(self, ego_vehicle_state, center_x, center_y):

        ego_centered_state = self.centered_state(ego_vehicle_state, center_x, center_y)
        self.calculate_relative_state(ego_vehicle_state)

        # frame rotation transformation
        self.egocentric_state = {"type": "",
                                 "position": rotate2D(self.relative_state["position"] + ego_centered_state["position"], ego_vehicle_state["heading"]),
                                 "heading": self.relative_state["heading"] + ego_centered_state["heading"],
                                 # todo: maybe 90 degs or 0 deg
                                 "speed": self.relative_state["speed"] + ego_centered_state["speed"],
                                 "angular_velocity": self.relative_state["angular_velocity"] + ego_centered_state[
                                     "angular_velocity"],
                                 "acceleration": self.relative_state["acceleration"] + ego_centered_state["acceleration"],
                                 "velocity":self.relative_state["velocity"] + ego_centered_state["velocity"]}


        self.scaled_egocentric_state = {"type": "",
                                        "position": rotate2D(self.relative_state["position"],ego_vehicle_state['heading']) / self.potential_field_granularity + ego_centered_state["position"],
                                        # use relative state for position scale
                                        "heading": self.egocentric_state["heading"],  # todo: maybe 90 degs or 0 deg
                                        "speed": self.egocentric_state["speed"],
                                        "angular_velocity": self.egocentric_state["angular_velocity"],
                                        "acceleration": self.egocentric_state["acceleration"],
                                        "velocity":self.egocentric_state["velocity"]}


        return self.scaled_egocentric_state, self.egocentric_state, self.relative_state

    def get_egocentric_state(self):
        return self.egocentric_state

    def get_scaled_egocentric_state(self):
        return self.scaled_egocentric_state
    def set_relative_state(self, position = None, heading = None, speed = None, angular_vel = None, accel = None, vel = None):

        if position != None and len(position) == 3:
            self.relative_state["position"] = position
        if heading != None:
            self.relative_state["heading"] = heading
        if speed != None:
            self.relative_state["speed"] = speed
        if angular_vel != None and len(angular_vel) == 3:
            self.relative_state["angular_velocity"] = angular_vel
        if accel != None and len(accel) == 3:
            self.relative_state["acceleration"] = accel
        if vel != None and len(vel) == 3:
            self.relative_state["velocity"] = vel

    def static_APF(self, x, y):

        # todo: remember to update relative state before calling

        i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
        num = self.safety_radius - (((x-i)**2 + (y-j)**2)/self.safety_radius)
        denom = - self.safety_radius - (((x-i)**2 + (y-j)**2)/(0.01*self.safety_radius))

        return min(-100*(abs(num)+num)/denom, 255)


    def dynamic_APF(self, x, y):
        return self.static_APF(x, y)

    def dynamic_APF_SVO(self,x, y, svo):
        return self.dynamic_APF(x, y)

    def centroid_APF(self, x, y):
        position = self.scaled_egocentric_state["position"]
        if x == int(position[0]) and y == int(position[1]):
            return 255
        else:
            return 0