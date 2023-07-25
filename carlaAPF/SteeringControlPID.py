from simple_pid import PID

class Steering_Control_PID:
    def __init__(self, ego_vehicle, potential_field_granularity, potential_field = None):
        self.potential_field = potential_field
        self.potential_field_granularity = potential_field_granularity
        self.pid_look_ahead_distance = 2
        self.vehicle = ego_vehicle
        self.pid = PID(0, 0, 0, setpoint=0)
        self.pid.output_limits = (-1, 1)
    def set_PID_values(self,p,i,d):
        self.pid.tunings = (p,i,d)

    def angular_position_pid(self, path):

        pass
    def get_control_output(self, path):
        vehicle_location_lateral = path[0][1]*self.potential_field_granularity
        target_lateral = path[self.pid_look_ahead_distance][1]*self.potential_field_granularity

        self.pid.setpoint = target_lateral
        control_output = self.pid(vehicle_location_lateral)

        print("PID output", control_output)
        return control_output



    def show_output(self):
        pass