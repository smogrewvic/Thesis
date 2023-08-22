from simple_pid import PID
import matplotlib.pyplot as plt


class Steering_Control_PID:
    def __init__(self, ego_vehicle, potential_field_granularity, regression_precision=1, meters_look_ahead_dist=5, potential_field=None):
        self.potential_field = potential_field
        self.potential_field_granularity = potential_field_granularity
        self.regression_precision = regression_precision
        self.meters_look_ahead_dist = meters_look_ahead_dist
        self.pid_look_ahead_distance = int(meters_look_ahead_dist/regression_precision)
        self.vehicle = ego_vehicle
        self.pid = PID(0, 0, 0, setpoint=0)
        self.pid.output_limits = (-1, 1)

        self.tracking_data = []

    def set_regression_precision(self, regression_precision):
        self.regression_precision = regression_precision
        self.pid_look_ahead_distance = int(self.meters_look_ahead_dist/ regression_precision)
    def set_PID_values(self, p=0, i=0, d=0):
        self.pid.tunings = (p, i, d)

    def angular_position_pid(self, path):
        pass

    def get_control_output(self, path):
        vehicle_location_lateral = len(self.potential_field)//2 * self.potential_field_granularity
        if len(path) > self.pid_look_ahead_distance:
            target_lateral = path[self.pid_look_ahead_distance][1] * self.potential_field_granularity
        else:
            target_lateral = path[-1][1] * self.potential_field_granularity

        self.pid.setpoint = target_lateral
        control_output = self.pid(vehicle_location_lateral)

        # print("PID output", control_output)
        self.tracking_data.append([target_lateral, vehicle_location_lateral])
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
