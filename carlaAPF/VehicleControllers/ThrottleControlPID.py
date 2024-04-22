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
        self.throttle_range = [0,1]  # max [0, 1]
        self.brake_range = [0,1]  # max [0, 1]
        self.dead_zone = {'throttle':0.05, 'brake': 0.05}
        self.tracking_data = []

    def set_PID_values(self, p=0, i=0, d=0):
        self.pid.tunings = (p, i, d)

    def get_control_output(self, path, speed_limit, kph = True):
        clear_distance = len(self.potential_field) // 2 * self.potential_field_granularity
        minima_distance = clear_distance - path[0][0] * self.potential_field_granularity
        speed_limit = speed_limit / 3.6 if kph == True else speed_limit
        minimum_speed = -0.5
        stop_distance = 0.5

        # map target speed to gradient descent distance (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        target_speed = (minima_distance - stop_distance) * (speed_limit - minimum_speed) / (clear_distance - stop_distance) + minimum_speed
        current_speed = np.linalg.norm([self.ego_vehicle.get_velocity().x, self.ego_vehicle.get_velocity().y, self.ego_vehicle.get_velocity().z])
        self.pid.setpoint = target_speed
        control_output = self.pid(current_speed)

        self.tracking_data.append([target_speed, current_speed])

        if control_output >= 0:
            brake_output = 0
            throttle_output = min(control_output, self.throttle_range[1])
        else:
            brake_output = max(control_output, self.brake_range[1])
            throttle_output = 0

        return throttle_output, brake_output

    def display_PID_tracking(self):
        x = []
        y1 = []
        y2 = []
        for i, values in enumerate(self.tracking_data):
            x.append(i)
            y1.append(values[0])
            y2.append(values[1])

        plt.cla()
        plt.title("Vehicle Speed")
        plt.plot(x, y1, label="Target m/s")
        plt.plot(x, y2, label="Current m/s")
        plt.draw()
        plt.pause(0.01)
