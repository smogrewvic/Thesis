import numpy as np
import collections
import threading
from multiprocessing import Process, Pool

from PIL import Image
from PedestrianAPF import PedestrianAPF
from VehicleAPF import VehicleAPF


class APF:
    def __init__(self, field_size = 10, granularity = 0.1, actor_data_filepath = "actor_data.txt"):
        self.field_size = field_size  # meters
        self.field_granularity = granularity   # meters
        self.potential_field = np.zeros((int(self.field_size/self.field_granularity), int(self.field_size/self.field_granularity)))
        self.actor_data_filepath = actor_data_filepath
        self.actor_ids = {}
        # self.ego_vehicle = ego_vehicle

    def compute_total_APF(self):
        pass

    @staticmethod
    def write_actor_data(actorList):
        with open('actor_data.txt', 'w') as file:
            for actor in actorList:

                id = actor.id
                velocity = round(np.linalg.norm([actor.get_velocity().x,actor.get_velocity().y,actor.get_velocity().z]), 4)
                heading = round(actor.get_transform().rotation.yaw, 4)
                position = round(actor.get_transform().location.x, 4), round(actor.get_transform().location.y, 4), round(actor.get_transform().location.z, 4)
                acceleration = round(actor.get_acceleration().x, 4), round(actor.get_acceleration().y, 4), round(actor.get_acceleration().z, 4)
                angular_vel = round(actor.get_angular_velocity().x, 4), round(actor.get_angular_velocity().y, 4), round(actor.get_angular_velocity().z, 4)

                vehicle_type = actor.type_id
                if vehicle_type.find("vehicle") >= 0:
                    vehicle_type = "vehicle"
                elif vehicle_type.find("pedestrian") >= 0:
                    vehicle_type = "pedestrian"
                else:
                    continue  # does not send other actor types

                file.write("ID = " + str(id)
                           + "\ttype = " + str(vehicle_type)
                           + "\tposition = " + str(position)
                           + "\theading = " + str(heading)
                           + "\tspeed = " + str(velocity)
                           + "\tangular_velocity = " + str(angular_vel)
                           + "\tacceleration = " + str(acceleration)
                           + '\n')

            file.write("DONE")
            file.close()

    def read_actor_data(self):
        file = open(self.actor_data_filepath, "r")

        actor_state = {"type": "", "position":[0,0,0], "heading": 0,  "speed": 0, "angular_velocity": [0,0,0], "acceleration": [0,0,0]}


        for data in file:
            if data[0:2] != "ID": break  # file has finished

            #read id
            start = data.find("ID") + len("ID") + 3
            end = data.find("\t")
            id = float(data[start:end])

            #read type
            start = data.find("type") + len("type") + 3
            end = data.find("\t", start)
            type = data[start:end]

            #read position x,y,z
            start = data.find("position") + len("position") + 4
            end = data.find(",", start)
            x = float(data[start:end])

            start = end+2
            end = data.find(",", start)
            y = float(data[start:end])

            start = end + 2
            end = data.find(")", start)
            z = float(data[start:end])

            position = [x,y,z]

            #read heading
            start = data.find("heading")  + len("heading") + 3
            end = data.find("\t", start)
            heading = float(data[start:end])

            #read speed
            start = data.find("speed") + len("speed") + 3
            end = data.find("\t", start)
            speed = float(data[start:end])

            #read angular velocity x,y,z
            start = data.find("angular_velocity") + len("angular_velocity") + 4
            end = data.find(",", start)
            x = float(data[start:end])

            start = end + 2
            end = data.find(",", start)
            y = float(data[start:end])

            start = end + 2
            end = data.find(")", start)
            z = float(data[start:end])

            angular_velocity = [x, y, z]

            # read acceleration x,y,z
            start = data.find("acceleration") + len("acceleration") + 4
            end = data.find(",", start)
            x = float(data[start:end])

            start = end + 2
            end = data.find(",", start)
            y = float(data[start:end])

            start = end + 2
            end = data.find(")", start)
            z = float(data[start:end])

            acceleration = [x, y, z]

            actor_state = {"type": type,
                          "position": position,
                          "heading": heading,
                          "speed": speed,
                          "angular_velocity": angular_velocity,
                          "acceleration": acceleration}

            # self.actor_ids.update({id : actor_state})

            self.actor_ids.update({id : VehicleAPF(actor_state)})


        file.close()

    def lane_data(self, mapObj):
        print(mapObj)

    def add_APF(self, id):
        # print("thread#", threading.get_ident())
        if type(self.actor_ids[id]) is VehicleAPF:  # might have to change to PotentialField.VehicleAPF
            for y in range(len(self.potential_field[0])):
                for x in range(len(self.potential_field[0])):
                    self.potential_field[x][y] = 255

        elif type(self.actor_ids[id]) is PedestrianAPF:  # might have to change to PotentialField.VehicleAPF
            for y in range(len(self.potential_field[0])):
                for x in range(len(self.potential_field[0])):
                    self.potential_field[x][y] = 255

    def generate_APF_multicored(self):
        self.read_actor_data()
        processes = []
        for id in self.actor_ids:
            processes.append(Process(target=self.add_APF, args=(id,)))
            processes[-1].start()

        for process in processes:
            process.join()

    def generate_APF_threaded(self):
        # print("generating threaded")
        self.read_actor_data()
        threads = []
        for id in self.actor_ids:
            threads.append(threading.Thread(target = self.add_APF, args = (id,)))
            threads[-1].start()

        for thread in threads:
            thread.join()

    def set_ego_vehicle_state(self):
        pass

    def generate_APF(self):
        print("generating")
        self.read_actor_data()
        for id in self.actor_ids:
            # distance = np.linalg.norm(self.actor_ids[id].get_relative_state(ego_vehicle_state)["position"])
            # if distance >= self.field_size:
            #     continue

            if type(self.actor_ids[id]) is VehicleAPF:  # might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field[0])):
                    for x in range(len(self.potential_field[0])):
                        self.potential_field[x][y] = 35


            elif type(self.actor_ids[id]) is PedestrianAPF:  # might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field[0])):
                    for x in range(len(self.potential_field[0])):
                        self.potential_field[x][y] = 35



    def save_image_APF(self):
        # test_img = np.zeros((255,255,3))
        # val = 0
        #
        # for x in range(len(test_img)):
        #     for y in range(len(test_img)):
        #
        #         test_img[x][y][0] = val
        #         test_img[x][y][1] = val
        #         test_img[x][y][2] = val
        #         val+=1
        # apf_image = Image.fromarray(test_img, mode="L")
        # apf_image.save("APF_Image.bmp")

        grayscale = np.array(self.potential_field, dtype = np.uint8)

        apf_image = Image.fromarray(grayscale, mode = "L")
        apf_image.save("APF_Image.bmp")



