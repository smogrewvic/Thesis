import numpy as np
import carla

from PIL import Image
from PedestrianAPF import PedestrianAPF
from VehicleAPF import VehicleAPF
import cv2
import matplotlib.pyplot as plt


class APF:
    def __init__(self, field_size = 20, granularity = 0.5):
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()

        self.field_size = field_size  # meters
        self.field_granularity = granularity   # meters
        self.potential_field = np.zeros((int(self.field_size/self.field_granularity), int(self.field_size/self.field_granularity)))

        self.actor_ids = {}

    def update_actor_states(self):
        #todo: stop creating a new vehicleAPF each call
        carla_actors = self.world.get_actors()

        for actor in carla_actors:

            vehicle = True if actor.type_id.find("vehicle") >= 0 else False
            pedestrian = True if actor.type_id.find("pedestrian") >= 0 else False

            if not vehicle and not pedestrian:
                continue

            id = actor.id
            # velocity = round(actor.get_velocity().x, 4), round(actor.get_velocity().y, 4), (actor.get_velocity().z, 4)
            speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]),4)
            heading = round(actor.get_transform().rotation.yaw, 4)
            position = round(actor.get_transform().location.x, 4), round(actor.get_transform().location.y, 4), round(actor.get_transform().location.z, 4)
            acceleration = round(actor.get_acceleration().x, 4), round(actor.get_acceleration().y, 4), round(actor.get_acceleration().z, 4)
            angular_vel = round(actor.get_angular_velocity().x, 4), round(actor.get_angular_velocity().y, 4), round(actor.get_angular_velocity().z, 4)

            actor_state = {"position":np.array(position),
                           "heading": heading,
                           "speed": speed,
                           # "velocity": np.array(velocity),
                           "angular_velocity": np.array(angular_vel),
                           "acceleration": np.array(acceleration)}

            #check if ego_vehicle else make generic vehicle or pedestrian
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
                self.actor_ids.update({"ego_vehicle": VehicleAPF(actor_state, len(self.potential_field), self.field_granularity)})

            elif vehicle:
                self.actor_ids.update({id: VehicleAPF(actor_state, len(self.potential_field), self.field_granularity)})

            elif pedestrian:
                self.actor_ids.update({id : PedestrianAPF(actor_state, len(self.potential_field), self.field_granularity)})


    def get_actor_states(self):
        for id in self.actor_ids:
            print(self.actor_ids[id].get_state())

        return self.actor_ids



    def generate_APF(self):

        self.potential_field.fill(0)  # erase potential field to not sum between calls
        self.update_actor_states()

        for id in self.actor_ids:
            # if id == "ego_vehicle" : continue  # ignore ego_vehicle

            #update egocentric actor state to center in APF relative to ego vehicle
            self.actor_ids[id].update_alternate_states(self.actor_ids["ego_vehicle"].get_state(), len(self.potential_field)//2, len(self.potential_field)//2)

            distance = np.linalg.norm(self.actor_ids[id].get_relative_state()["position"])
            if abs(distance) >= self.field_size:
                continue

            if type(self.actor_ids[id]) is VehicleAPF:  # might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field)):
                    for x in range(len(self.potential_field[0])):
                        # self.potential_field[x][y] = min(self.potential_field[x][y] + self.actor_ids[id].static_APF(x, y), 255)
                        #negative indexes to draw right side up
                        self.potential_field[-x-1][y] = min(self.potential_field[-x-1][y]+self.actor_ids[id].static_APF(x,y), 255)
                        # negative indexes for double flipped
                        # self.potential_field[-x - 1][-y - 1] = min(self.potential_field[-x - 1][-y - 1] + self.actor_ids[id].static_APF(x, y), 255)

            elif type(self.actor_ids[id]) is PedestrianAPF:  # might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field)):
                    for x in range(len(self.potential_field[0])):
                        # self.potential_field[x][y] = min(self.potential_field[x][y] + self.actor_ids[id].static_APF(x, y), 255)
                        # negative indexes to draw right side up
                        self.potential_field[-x-1][y] = min(self.potential_field[-x-1][y]+self.actor_ids[id].static_APF(x,y), 255)
                        # negative indexes for double flipped
                        # self.potential_field[-x - 1][-y-1] = min(self.potential_field[-x - 1][-y-1] + self.actor_ids[id].static_APF(x, y), 255)


    def save_image_APF(self):

        grayscale = np.array(self.potential_field, dtype = np.uint8)

        apf_image = Image.fromarray(grayscale, mode = "L")
        apf_image.save("APF_Image.bmp")


    def show_APF(self):
        img =cv2.imread("APF_Image.bmp")
        resized = cv2.resize(img, (500,500), interpolation=cv2.INTER_AREA)
        cv2.imshow("image", resized)
        cv2.waitKey(1)

    def plot_actor_positions(self):
        x_actors = []
        y_actors = []
        x_ego = [1]
        y_ego = [1]
        for id in self.actor_ids:
            if id == "ego_vehicle":
                x_ego[0] = self.actor_ids[id].get_state()["position"][0]
                y_ego[0] = self.actor_ids[id].get_state()["position"][1]
            else:
                x_actors.append(self.actor_ids[id].get_state()["position"][0])
                y_actors.append(self.actor_ids[id].get_state()["position"][1])

        plt.cla()
        plt.scatter(x_actors, y_actors, c="blue")
        plt.scatter(x_ego, y_ego, c = "red")
        plt.xlim(-150,150)
        plt.ylim(-150,150)
        plt.draw()
        plt.pause(0.01)