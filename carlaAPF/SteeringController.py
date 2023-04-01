import numpy as np
import carla


class SteeringController:
    def __init__(self, potential_field, granularity):
        self.potential_field = potential_field
        self.granularity = granularity
        self.max_steering_angle = None

    def set_apf(self, potential_field, granularity):
        self.potential_field = potential_field
        self.granularity = granularity

    def search_lowest_potential(self, search_radius=2):

        ego_x, ego_y = len(self.potential_field) // 2, len(self.potential_field[0]) // 2
        lowest_potential = {"value": float('inf'),
                            "relative_position": [ego_x, ego_y],
                            "absolute_position": [ego_x, ego_y],
                            "angle": 0}

        # search for lowest gradient
        search_box = search_radius / self.granularity

        for y in range(int(-search_box//2), int(search_box//2)):
            for x in range(int(-search_box//2), 0):
                potential = self.potential_field[x + ego_x][y + ego_y]
                # print("checked", x + ego_x, y + ego_y)

                if potential < lowest_potential["value"]:

                    lowest_potential["value"] = potential
                    lowest_potential["relative_position"] = [-x, y]  # indexed from top left of 2d apf
                    lowest_potential["absolute_position"] = [x + ego_x, y + ego_y]


        vector1 = [0, 1]
        vector2 = lowest_potential["relative_position"]
        lowest_potential["angle"] = np.degrees(np.pi/2 - np.arccos(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))))

        # print("value",lowest_potential["value"],"position", lowest_potential["absolute_position"], "angle",lowest_potential["angle"] )
        print(lowest_potential)
        return lowest_potential

    def get_steering(self, actor):

        if self.max_steering_angle == None:
            self.max_steering_angle = actor.get_physics_control().wheels[0].max_steer_angle

        steering_angle = self.search_lowest_potential()["angle"]

        # normalize steering angle to -1 to 1
        low = -1
        high = 1
        normalized_steering_angle = (steering_angle + self.max_steering_angle) / (2 * self.max_steering_angle) * (high - low) - 1

        # actor.apply_control(carla.VehicleControl(steer=-1))
        print("applied steering",normalized_steering_angle)
        return normalized_steering_angle

