from simple_pid import PID
import matplotlib.pyplot as plt
import numpy as np


class Throttle_Control_PID:
    def __init__(self, ego_vehicle, potential_field, potential_field_granularity):
        self.ego_vehicle = ego_vehicle
        self.potential_field = potential_field
        self.potential_field_granularity = potential_field_granularity
        self.pid = PID(0, 0, 0, setpoint=0)
        self.pid.output_limits = (-1, 1)

        self.tracking_data = []

    def set_PID_values(self, p=0, i=0, d=0):
        self.pid.tunings = (p, i, d)

    def get_control_output(self, path):
        clear_distance = len(self.potential_field) // 2 * self.potential_field_granularity
        minima_distance = min(path[-1][0], clear_distance)
        speed_limit = 50
        minimum_speed = 0
        stop_distance = 0.5

        # map target speed to gradient descent distance (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        target_speed = (minima_distance - stop_distance) * (speed_limit - minimum_speed) / (clear_distance - stop_distance) + minimum_speed
        current_speed = np.linalg.norm(self.ego_vehicle.get_velocity())
        print("current speed", current_speed)
        self.pid.setpoint = target_speed
        control_output = self.pid(current_speed)

        print("Throttle PID output", control_output)

        return control_output

    def display_PID_tracking(self):
        x = []
        y1 = []
        y2 = []
        for i, values in enumerate(self.tracking_data):
            x.append(i)
            y1.append(values[0])
            y2.append(values[1])

        plt.cla()
        plt.plot(x, y1, label="target")
        plt.plot(x, y2, label="vehicle position")
        plt.draw()
        plt.pause(0.01)
