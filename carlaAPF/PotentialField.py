import numpy as np
import math
import carla

from PIL import Image
from PedestrianAPF import PedestrianAPF
from VehicleAPF import VehicleAPF
from LaneAPF import LaneAPF
from NavpointAPF import NavpointAPF
from RegressionLaneAPF import Regression_Lane_APF
import cv2
import matplotlib.pyplot as plt


class APF:
    def __init__(self, field_size=100, granularity=1):
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()

        self.field_size = field_size  # meters
        self.field_granularity = granularity  # meters
        self.potential_field = np.zeros(
            (int(self.field_size / self.field_granularity), int(self.field_size / self.field_granularity)))
        self.standard_lane_width = 3
        self.actor_ids = {}
        self.navpoint_actors = [] # to create lane apf

        self.max_steering_angle = 70  # actor.get_physics_control().wheels[0].max_steer_angle
        # self.lowest_potential = {"value": float('inf'),
        #                          "relative_position": [0, 0],
        #                          "absolute_position": [0, 0],
        #                          "angle": 0,
        #                          "normalized_angle": 0}


    def update_actor_states(self):
        carla_actors = self.world.get_actors()
        for actor in carla_actors:

            vehicle = True if actor.type_id.find("vehicle") >= 0 else False
            pedestrian = True if actor.type_id.find("pedestrian") >= 0 else False

            if not vehicle and not pedestrian:
                continue

            id = "ego_vehicle" if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle' else actor.id
            # velocity = round(actor.get_velocity().x, 4), round(actor.get_velocity().y, 4), (actor.get_velocity().z, 4)
            speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
            heading = round(actor.get_transform().rotation.yaw, 4)
            position = (round(actor.get_transform().location.x, 4),
                        round(actor.get_transform().location.y, 4),
                        round(actor.get_transform().location.z, 4))
            acceleration = (round(actor.get_acceleration().x, 4),
                            round(actor.get_acceleration().y, 4),
                            round(actor.get_acceleration().z, 4))
            angular_vel = (round(actor.get_angular_velocity().x, 4),
                           round(actor.get_angular_velocity().y, 4),
                           round(actor.get_angular_velocity().z, 4))

            velocity = (round(actor.get_velocity().x, 4),
                        round(actor.get_velocity().y, 4),
                        round(actor.get_velocity().z, 4))

            actor_state = {"position": np.array(position),
                           "heading": heading,
                           "speed": speed,
                           "angular_velocity": np.array(angular_vel),
                           "acceleration": np.array(acceleration),
                           "velocity": np.array(velocity)}

            if id not in self.actor_ids:  # create apf object
                if vehicle:
                    self.actor_ids.update({id: VehicleAPF(len(self.potential_field), self.field_granularity)})

                elif pedestrian:
                    self.actor_ids.update({id: PedestrianAPF(len(self.potential_field), self.field_granularity)})

            self.actor_ids[id].set_state(actor_state)

    def update_lane_states(self):

        # find ego vehicle asset in carla world, can't use self.actor_ids["ego_vehicle"]
        carla_actors = self.world.get_actors()
        ego_vehicle = None
        for actor in carla_actors:
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
                ego_vehicle = actor
                break
        if ego_vehicle == None: return  # ego_vehicle not found

        lane_center = self.world.get_map().get_waypoint(ego_vehicle.get_location(), project_to_road=True, lane_type=(
                carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))

        speed = 0
        heading = round(lane_center.transform.rotation.yaw, 4)

        position_left = (round(lane_center.transform.location.x, 4) + np.cos(np.radians(heading)-np.pi/2) * self.standard_lane_width / 2,
                         round(lane_center.transform.location.y, 4) + np.sin(np.radians(heading)-np.pi/2) * self.standard_lane_width / 2,
                         0)
        position_right = (round(lane_center.transform.location.x, 4) + np.cos(np.radians(heading)+np.pi/2) * self.standard_lane_width / 2,
                          round(lane_center.transform.location.y, 4) + np.sin(np.radians(heading)+np.pi/2) * self.standard_lane_width / 2,
                          0)
        acceleration = (0, 0, 0)
        angular_vel = (0, 0, 0)
        velocity = (0,0,0)

        left_lane_state = {"position": np.array(position_left),
                           "heading": heading,
                           "speed": speed,
                           "angular_velocity": np.array(angular_vel),
                           "acceleration": np.array(acceleration),
                           "velocity":velocity}

        right_lane_state = {"position": np.array(position_right),
                            "heading": heading,
                            "speed": speed,
                            "angular_velocity": np.array(angular_vel),
                            "acceleration": np.array(acceleration),
                            "velocity":velocity}

        if "left_lane" not in self.actor_ids:
            self.actor_ids.update({"left_lane": LaneAPF(len(self.potential_field), self.field_granularity)})

        if "right_lane" not in self.actor_ids:
            self.actor_ids.update({"right_lane": LaneAPF(len(self.potential_field), self.field_granularity)})

        self.actor_ids["left_lane"].set_state(left_lane_state)
        self.actor_ids["right_lane"].set_state(right_lane_state)

    def get_actor_states(self):
        for id in self.actor_ids:
            print(self.actor_ids[id].get_state())

        return self.actor_ids

    def generate_APF(self):

        self.potential_field.fill(0)  # erase potential field to not sum between calls
        self.update_actor_states()
        self.update_lane_states()

        for id in self.actor_ids:
            if id == "ego_vehicle" : continue  # ignore ego_vehicle APF

            # update egocentric actor state to center in APF relative to ego vehicle
            self.actor_ids[id].update_alternate_states(self.actor_ids["ego_vehicle"].get_state(),
                                                       len(self.potential_field) // 2, len(self.potential_field) // 2)

            distance = np.linalg.norm(self.actor_ids[id].get_relative_state()["position"])
            if abs(distance) >= self.field_size: #skip actors out of field
                continue

            if type(self.actor_ids[id]) == NavpointAPF: continue  # skip navpoints after this point

            for y in range(len(self.potential_field)):
                for x in range(len(self.potential_field[0])):

                    # indexed from top left
                    self.potential_field[-x - 1][y] = min(
                        self.potential_field[-x - 1][y] + self.actor_ids[id].dynamic_APF(x, y), 255)

        self.set_lane_APF()

    def set_navpoints(self, navpoint_transforms):
        for i, navpoint_transform in enumerate(navpoint_transforms):

            id = "navpoint_" + str(i)
            speed = 0
            heading = round(navpoint_transform.rotation.yaw, 4)
            position = (round(navpoint_transform.location.x, 4), round(navpoint_transform.location.y, 4), 0)
            acceleration = (0, 0, 0)
            angular_vel = (0, 0, 0)
            velocity = (0, 0, 0)

            navpoint_state = {"position": np.array(position),
                               "heading": heading,
                               "speed": speed,
                               "angular_velocity": np.array(angular_vel),
                               "acceleration": np.array(acceleration),
                               "velocity":np.array(velocity)}

            self.actor_ids.update({id: NavpointAPF(len(self.potential_field), self.field_granularity)})
            self.actor_ids[id].set_state(navpoint_state)

            #create the laneAPF from navpoints
            self.navpoint_actors.append(self.actor_ids[id])

    def set_lane_APF(self):
        #get all navpoint actors and send to laneAPF
        # todo: check that navpoints are stored in actor_ids in order
        lane = Regression_Lane_APF(self.field_size, self.field_granularity, self.actor_ids["ego_vehicle"].get_state())
        lane.set_navpoints(self.navpoint_actors)
        lane.update_lane()
        for y in range(len(self.potential_field)):
            for x in range(len(self.potential_field[0])):
                ## indexed from top left
                self.potential_field[-x - 1][y] = min(
                    self.potential_field[-x - 1][y] + lane.static_APF(x, y), 255)

    def save_image_APF(self):

        grayscale = np.array(self.potential_field, dtype=np.uint8)

        apf_image = Image.fromarray(grayscale, mode="L")
        apf_image.save("APF_Image.bmp")

    def show_APF(self):
        img = cv2.imread("APF_Image.bmp")
        resized = cv2.resize(img, (500, 500), interpolation=cv2.INTER_AREA)
        normalized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("image", normalized)
        cv2.waitKey(1)

    def plot_actor_positions(self):
        x_actors = []
        y_actors = []
        x_ego = [1]
        y_ego = [1]
        x_navpoints = []
        y_navpoints = []

        for id in self.actor_ids:
            if id == "ego_vehicle":
                x_ego[0] = self.actor_ids[id].get_state()["position"][0]
                y_ego[0] = self.actor_ids[id].get_state()["position"][1]
            elif type(self.actor_ids[id]) is NavpointAPF:
                x_navpoints.append(self.actor_ids[id].get_state()["position"][0])
                y_navpoints.append(self.actor_ids[id].get_state()["position"][1])
            else:
                x_actors.append(self.actor_ids[id].get_state()["position"][0])
                y_actors.append(self.actor_ids[id].get_state()["position"][1])

        plt.cla()
        plt.scatter(x_actors, y_actors, c="blue")
        plt.scatter(x_navpoints, y_navpoints, c="green")
        plt.scatter(x_ego, y_ego, c="red")
        plt.xlim(150, -150)
        plt.ylim(-150, 150)
        plt.draw()
        plt.pause(0.01)

    def get_potential_field(self):
        return self.potential_field

    def get_granularity(self):
        return self.field_granularity

    # def search_box_lowest_potential(self, box_size=2):
    #
    #     ego_x, ego_y = len(self.potential_field) // 2, len(self.potential_field[0]) // 2
    #     self.lowest_potential = {"value": float('inf'),
    #                              "relative_position": [ego_x, ego_y],
    #                              "absolute_position": [ego_x, ego_y],
    #                              "angle": 0,
    #                              "normalized_angle": 0}
    #
    #     # search for lowest gradient
    #     search_box = box_size / self.field_granularity
    #
    #     for y in range(int(-search_box // 2), int(search_box // 2)):
    #         for x in range(int(-search_box // 2), 0):
    #             potential = self.potential_field[x + ego_x][y + ego_y]
    #             # print("checked", x + ego_x, y + ego_y)
    #
    #             if potential < self.lowest_potential["value"]:
    #                 self.lowest_potential["value"] = potential
    #                 self.lowest_potential["relative_position"] = [-x, y]  # indexed from top left of 2d apf
    #                 self.lowest_potential["absolute_position"] = [x + ego_x, y + ego_y]
    #
    #     vector1 = [0, 1]
    #     vector2 = self.lowest_potential["relative_position"]
    #     self.lowest_potential["angle"] = np.degrees(
    #         np.pi / 2 - np.arccos(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))))
    #
    #     steering_angle = self.lowest_potential["angle"]
    #
    #     # normalize steering angle to -1 to 1   y=(x-a)/(b-a)*(d-c)+c
    #     low = -1
    #     high = 1
    #     self.lowest_potential["normalized_angle"] = (steering_angle + self.max_steering_angle) / (2 * self.max_steering_angle) * (high - low) + low
    #
    #     return self.lowest_potential
    #
    # def search_radius_lowest_potential(self, radius = 3):
    #     radius = min(radius, self.field_size//2)  # do not exceed pf
    #
    #     ego_x, ego_y = len(self.potential_field) // 2, len(self.potential_field[0]) // 2
    #     self.lowest_potential = {"value": float('inf'),
    #                              "relative_position": [ego_x, ego_y],
    #                              "absolute_position": [ego_x, ego_y],
    #                              "angle": 0,
    #                              "normalized_angle": 0}
    #
    #     # search for lowest gradient
    #     search_radius = radius / self.field_granularity
    #
    #     self.debug_radius = []
    #     for y in range(int(-search_radius), int(search_radius)):
    #         x = -np.sqrt(search_radius**2 - y**2)
    #         x1 = math.ceil(x)
    #         x2 = round(x)
    #         self.debug_radius.append([x1+ego_x, y + ego_y])
    #         self.debug_radius.append([x2+ego_x, y+ego_y])
    #         # check potential at both values
    #         potential = self.potential_field[x1 + ego_x][y + ego_y]
    #         if potential < self.lowest_potential["value"]:
    #             self.lowest_potential["value"] = potential
    #             self.lowest_potential["relative_position"] = [-x1, y]  # indexed from top left of 2d apf
    #             self.lowest_potential["absolute_position"] = [x1 + ego_x, y + ego_y]
    #
    #         potential = self.potential_field[x2 + ego_x][y + ego_y]
    #         if potential < self.lowest_potential["value"]:
    #             self.lowest_potential["value"] = potential
    #             self.lowest_potential["relative_position"] = [-x2, y]  # indexed from top left of 2d apf
    #             self.lowest_potential["absolute_position"] = [x2 + ego_x, y + ego_y]
    #
    #     for x in range(int(-search_radius), 0):
    #         y = np.sqrt(search_radius**2 - x**2)
    #         signs = [-1,1] # check for positive and negative values of sqrt
    #         for sign in signs:
    #             y1 = sign*math.ceil(y)
    #             y2 = sign*round(y)
    #             self.debug_radius.append([x+ego_x, y1 + ego_y])
    #             self.debug_radius.append([x+ego_x, y2+ego_y])
    #             # check potential at both values
    #             potential = self.potential_field[x + ego_x][y1 + ego_y]
    #             if potential < self.lowest_potential["value"]:
    #                 self.lowest_potential["value"] = potential
    #                 self.lowest_potential["relative_position"] = [-x, y1]  # indexed from top left of 2d apf
    #                 self.lowest_potential["absolute_position"] = [x + ego_x, y1 + ego_y]
    #
    #             potential = self.potential_field[x + ego_x][y2 + ego_y]
    #             if potential < self.lowest_potential["value"]:
    #                 self.lowest_potential["value"] = potential
    #                 self.lowest_potential["relative_position"] = [-x, y2]  # indexed from top left of 2d apf
    #                 self.lowest_potential["absolute_position"] = [x + ego_x, y2 + ego_y]
    #
    #
    #     vector1 = [0, 1]
    #     vector2 = self.lowest_potential["relative_position"]
    #     self.lowest_potential["angle"] = np.degrees(np.pi / 2 - np.arccos(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))))
    #
    #     steering_angle = self.lowest_potential["angle"]
    #
    #     # normalize steering angle to -1 to 1   y=(x-a)/(b-a)*(d-c)+c
    #     low = -1
    #     high = 1
    #     self.lowest_potential["normalized_angle"] = (steering_angle + self.max_steering_angle) / (2 * self.max_steering_angle) * (high - low) + low
    #
    #     return self.lowest_potential
    # def draw_lowest_point(self):
    #     grayscale = np.array(self.potential_field, dtype=np.uint8)
    #
    #     steering_point = self.lowest_potential["absolute_position"]
    #
    #     grayscale[steering_point[0], steering_point[1]] = 255
    #
    #     apf_image = Image.fromarray(grayscale, mode="L")
    #     apf_image.save("APF_Image.bmp")
    #
    #
    # def draw_APF(self):
    #     grayscale = np.array(self.potential_field, dtype=np.uint8)
    #
    #     apf_image = Image.fromarray(grayscale, mode="L")
    #     apf_image.save("APF_Image.bmp")