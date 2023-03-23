import numpy as np
import collections
from TransformMatrix import rotate2D, stretch2D

# class PedestrianAPF:
#     def __init__(self, state_data, potential_field_size, potential_field_granularity):
#         self.potential_field_size = potential_field_size
#         self.potential_field_granularity = potential_field_granularity
#         self.safety_radius = 5/potential_field_granularity
#         self.data_log = collections.deque(maxlen=100)
#         # self.state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0, "angular_velocity": np.zeros(3),
#         #               "acceleration": np.zeros(3)}
#         self.state = state_data
#
#         self.relative_state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0,
#                                "angular_velocity": np.zeros(3),
#                                "acceleration": np.zeros(3)}
#
#         self.egocentric_state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0,
#                                "angular_velocity": np.zeros(3),
#                                "acceleration": np.zeros(3)}
#
#         self.scaled_egocentric_state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0,
#                                "angular_velocity": np.zeros(3),
#                                "acceleration": np.zeros(3)}
#     def calculate_relative_state(self, ego_vehicle_state):
#
#         for key in self.state:
#             if key == "type":
#                 continue
#             else:
#                 self.relative_state[key] = self.state[key] - ego_vehicle_state[key]
#
#     def get_state(self):
#         return self.state
#
#     def get_relative_state(self):
#         return self.relative_state
#
#     def centered_state(self, absolute_state, center_x, center_y):
#         return {"type": "",
#                 "position": np.array([center_x, center_y, absolute_state["position"][2]]),
#                 "heading": absolute_state["heading"], # todo: maybe 90 degs or 0 degs?
#                 "speed": absolute_state["speed"],
#                 "angular_velocity": absolute_state["angular_velocity"],
#                 "acceleration": absolute_state["acceleration"]}
#
#     def update_alternate_states(self, ego_vehicle_state, center_x, center_y):
#         ego_centered_state = self.centered_state(ego_vehicle_state, center_x, center_y)
#         self.calculate_relative_state(ego_vehicle_state)
#
#         self.egocentric_state = {"type": "",
#                                  "position": self.relative_state["position"]+ego_centered_state["position"],
#                                  "heading": self.relative_state["heading"]+ego_centered_state["heading"], # todo: maybe 90 degs or 0 deg
#                                  "speed": self.relative_state["speed"]+ego_centered_state["speed"],
#                                  "angular_velocity": self.relative_state["angular_velocity"]+ego_centered_state["angular_velocity"],
#                                  "acceleration": self.relative_state["acceleration"]+ego_centered_state["acceleration"]}
#
#         self.scaled_egocentric_state = {"type": "",
#                                         "position": rotate2D(self.relative_state["position"], ego_vehicle_state['heading']) / self.potential_field_granularity + ego_centered_state["position"], # use relative state for position scale
#                                         "heading": self.egocentric_state["heading"],  # todo: maybe 90 degs or 0 deg
#                                         "speed": self.egocentric_state["speed"],
#                                         "angular_velocity": self.egocentric_state["angular_velocity"],
#                                         "acceleration": self.egocentric_state["acceleration"]}
#
#         return self.scaled_egocentric_state, self.egocentric_state, self.relative_state
#
#     def get_egocentric_state(self):
#         return self.egocentric_state
#
#
#     def set_relative_state(self, position = None, heading = None, speed = None, angular_vel = None, accel = None):
#
#         if position != None and len(position) == 3:
#             self.relative_state["position"] = position
#         if heading != None:
#             self.relative_state["heading"] = heading
#         if speed != None:
#             self.relative_state["speed"] = speed
#         if angular_vel != None and len(angular_vel) == 3:
#             self.relative_state["angular_velocity"] = angular_vel
#         if accel != None and len(accel) == 3:
#             self.relative_state["acceleration"] = accel
#
#     def static_APF(self, x, y):
#
#         # todo: remember to update relative state before calling
#
#         i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
#         num = self.safety_radius - (((x-i)**2 + (y-j)**2)/self.safety_radius)
#         denom = - self.safety_radius - (((x-i)**2 + (y-j)**2)/(0.01*self.safety_radius))
#
#         return min(-100*(abs(num)+num)/denom, 255)
#
#         # i, j = self.scaled_egocentric_state["position"][0], self.scaled_egocentric_state["position"][1]
#         # if x == i and y == j:
#         #     return 250
#         # else:
#         #     return 0
#
#     def dynamic_APF(self):
#         pass


from VehicleAPF import VehicleAPF
class PedestrianAPF(VehicleAPF):
    def __init__(self, potential_field_size, potential_field_granularity):
        super().__init__(potential_field_size, potential_field_granularity)
        self.safety_radius = 5 / potential_field_granularity