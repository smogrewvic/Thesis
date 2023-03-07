import numpy as np
import collections

class VehicleAPF:
    def __init__(self, state_data):
        # super().__init__()
        self.safety_radius = 2
        self.data_log = collections.deque(maxlen = 100)
        # self.state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0, "angular_velocity": np.zeros(3),
        #               "acceleration": np.zeros(3)}
        self.state = state_data

        self.relative_state = {"type": "", "position": np.zeros(3), "heading": 0, "speed": 0, "angular_velocity": np.zeros(3),
                      "acceleration": np.zeros(3)}

    def calculate_relative_state(self, ego_vehicle_state):

        for key in self.state:
            if key == "type": continue
            else:
                self.relative_state[key] = self.state[key] - ego_vehicle_state[key]


    def get_relative_state(self, ego_vehicle_state):
        self.calculate_relative_state(ego_vehicle_state)

        return self.relative_state

    def static_APF(self, x, y):


        distance = np.linalg.norm(self.relative_state["position"])
        if distance >= self.field_size: return -1

        i, j = self.relative_state["position"]
        num = self.safety_radius - (((x-i)**2 + (y-j)**2)/self.safety_radius)
        denom = - self.safety_radius - (((x-i)**2 + (y-j)**2)/(0.01*self.safety_radius))

        return (abs(num)+num)/denom

    def dynamic_APF(self):
        pass




