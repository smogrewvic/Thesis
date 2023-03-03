import numpy as np
import collections

class APF:
    def __init__(self, field_size = 100, granularity = 0.1, actor_data_filepath = "actor_data.txt"):
        self.field_size = field_size  # meters
        self.field_granularity = granularity   # meters
        self.potential_field = np.zeros((int(self.field_size/self.field_granularity), int(self.field_size/self.field_granularity)))
        self.actor_data_filepath = actor_data_filepath
        self.actor_ids = {}

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


    def generate_APF(self, ego_vehicle = None):

        self.read_actor_data()

        for id in self.actor_ids:

            if type(self.actor_ids[id]) is VehicleAPF: #might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field)):
                    for x in range(len(self.potential_field)):
                        self.potential_field = 0

            elif type(self.actor_ids[id]) is PedestrianAPF: #might have to change to PotentialField.VehicleAPF
                for y in range(len(self.potential_field)):
                    for x in range(len(self.potential_field)):
                        self.potential_field = 0


class PedestrianAPF:
    def __init__(self, state_data):
        # super().__init__()
        self.safety_radius = 5
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




