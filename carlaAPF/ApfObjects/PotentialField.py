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


class APF:
    def __init__(self, field_size=20, granularity=0.6):
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

    def get_actor_states(self):
        for id in self.actor_ids:
            print(self.actor_ids[id].get_state())

        return self.actor_ids

    def set_actor_APF(self):
        self.update_actor_states()

        for id in self.actor_ids:
            if id == "ego_vehicle": continue  # ignore ego_vehicle APF

            # update egocentric actor state to center in APF relative to ego vehicle
            self.actor_ids[id].update_alternate_states(self.actor_ids["ego_vehicle"].get_state(),
                                                       len(self.potential_field) // 2, len(self.potential_field) // 2)

            distance = np.linalg.norm(self.actor_ids[id].get_relative_state()["position"])
            if abs(distance) >= self.field_size: #skip actors out of field
                continue

            if type(self.actor_ids[id]) == NavpointAPF: continue  # skip navpoints after this point

            ignore_traffic_light = type(self.actor_ids[id]) == TrafficLightAPF and self.actor_ids[id].get_light_state() != carla.TrafficLightState.Red
            if ignore_traffic_light: continue

            for y in range(len(self.potential_field)):
                for x in range(len(self.potential_field[0])):

                    # indexed from top left
                    self.potential_field[-x - 1][y] = min(
                        self.potential_field[-x - 1][y] + self.actor_ids[id].dynamic_APF(x, y), 255)
    def generate_APF(self):

        self.potential_field.fill(0)  # erase potential field to not sum between calls

        self.set_actor_APF()
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


    def set_traffic_lights(self):

        client = carla.Client('localhost', 2000)
        world = client.get_world()

        for i, tl_actor in enumerate(world.get_actors().filter('traffic.traffic_light*')):
            #convert actor position to traffic light effect position
            conversion_key = ( round(tl_actor.get_location().x, 4), round(tl_actor.get_location().y, 4), round(tl_actor.get_location().z, 4) )
            converted_data = Traffic_Light_Info.convert_coordinates[conversion_key]

            id = "traffic_light_" + str(i)
            speed = 0
            heading = converted_data[5]  # yaw angle
            position = (converted_data[0], converted_data[1], converted_data[2] )
            acceleration = (0, 0, 0)
            angular_vel = (0, 0, 0)
            velocity = (0, 0, 0)

            traffic_light_state = {"position": np.array(position),
                               "heading": heading,
                               "speed": speed,
                               "angular_velocity": np.array(angular_vel),
                               "acceleration": np.array(acceleration),
                               "velocity":np.array(velocity)}

            self.actor_ids.update({id: TrafficLightAPF(len(self.potential_field), self.field_granularity, tl_actor)})
            self.actor_ids[id].set_state(traffic_light_state)

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

        x_traffic_lights = []
        y_traffic_lights = []
        x_crosswalks = []
        y_crosswalks = []

        for id in self.actor_ids:
            if id == "ego_vehicle":
                x_ego[0] = self.actor_ids[id].get_state()["position"][0]
                y_ego[0] = self.actor_ids[id].get_state()["position"][1]
            elif type(self.actor_ids[id]) is NavpointAPF:
                x_navpoints.append(self.actor_ids[id].get_state()["position"][0])
                y_navpoints.append(self.actor_ids[id].get_state()["position"][1])
            elif type(self.actor_ids[id]) is TrafficLightAPF:
                x_traffic_lights.append(self.actor_ids[id].get_state()["position"][0])
                y_traffic_lights.append(self.actor_ids[id].get_state()["position"][1])
            else:
                x_actors.append(self.actor_ids[id].get_state()["position"][0])
                y_actors.append(self.actor_ids[id].get_state()["position"][1])

        for coordinate in Crosswalk_Info.crosswalk_points:
            x_crosswalks.append(coordinate[0])
            y_crosswalks.append(coordinate[1])


        plt.cla()
        plt.scatter(x_navpoints, y_navpoints, c="green")
        plt.scatter(x_actors, y_actors, c="blue")
        plt.scatter(x_traffic_lights, y_traffic_lights, c = "orange")
        plt.scatter(x_crosswalks, y_crosswalks, c = "purple")
        plt.scatter(x_ego, y_ego, c="red")

        plt.xlim(150, -150)
        plt.ylim(-150, 150)
        plt.draw()
        plt.pause(0.01)

    def get_potential_field(self):
        return self.potential_field

    def get_granularity(self):
        return self.field_granularity

