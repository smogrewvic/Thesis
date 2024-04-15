from SVO.FuzzyEstimators.Vehicle_SVO_Fuzzy import Vehicle_SVO_Fuzzy
import numpy as np
import time
import collections
import carla
from SVO.ActorBehaviorProfiles.Vehicle_Behavior_Types import Vehicle_Behavior_Types
from SVO.ActorBehaviorProfiles.Reference_SVO import Reference_SVO

class Vehicle_Behavior_Analyser:
    def __init__(self, world):
        self.world = world
        self.map = world.get_map()
        self.type1_fuzzy = Vehicle_SVO_Fuzzy()
        self.social_values = {}
        self.vehicle_actors = self.filter_actors('vehicle')

        tracking_length = 100
        self.smoothness_tracker = {}
        self.speed_limit_tracker = {}
        self.lane_change_tracker = {}
        self.lane_centering_tracker = {}

        self.vehicle_behaviors = {}
        for actor in self.vehicle_actors:
            id = actor.id

            self.smoothness_tracker[id] = collections.deque(maxlen=tracking_length)
            self.speed_limit_tracker[id] = collections.deque(maxlen=tracking_length)
            self.lane_centering_tracker[id] = collections.deque(maxlen=tracking_length)
            self.lane_change_tracker[id] = collections.deque(maxlen=tracking_length)

            self.vehicle_behaviors[id] = {'follow_time': 0,
                                          'lane_changes': 0,
                                          'lane_centering': 0,
                                          'speed_limit_percent': 0.0,
                                          'smoothness': 0.0}
            self.social_values[id] = 0

    def filter_actors(self, actor_type):

        if actor_type == 'pedestrian':
            filter_key = 'walker*'
        elif actor_type == 'pedestrian_controller':
            filter_key = 'controller.ai.walker'
        elif actor_type == 'vehicle':
            filter_key = 'vehicle.*.*'
        else:
            print('invalid filter key')
            return
        filtered_actors = list(self.world.get_actors().filter(filter_key))
        # filtered_actors.sort(key=lambda actor: actor.id)

        filtered_ids = []
        for i in range(len(filtered_actors)):
            filtered_ids.append(filtered_actors[i].id)

        return self.world.get_actors(filtered_ids)

    def average_value(self, data):
        average = sum(data) / len(data)

        return average

    def count_unique_values(self, data):
        unique_values = set()
        for item in data:
            unique_values.add(item)

        return len(unique_values)

    def count_value_changes(self, data):
        counter = 0
        for i in range(len(data)-1):
            if data[i - 1] != data[i]:
                counter += 1

        return counter

    def get_actor_behavior_type(self, actor):
        if actor.attributes['role_name'] == 'sadistic':
            behavior = Vehicle_Behavior_Types.SADISTIC.value

        elif actor.attributes['role_name'] == 'competitive':
            behavior = Vehicle_Behavior_Types.COMPETITIVE.value

        elif actor.attributes['role_name'] == 'individualistic':
            behavior = Vehicle_Behavior_Types.INDIVIDUALISTIC.value

        elif actor.attributes['role_name'] == 'cooperative':
            behavior = Vehicle_Behavior_Types.COOPERATIVE.value

        elif actor.attributes['role_name'] == 'altruistic':
            behavior = Vehicle_Behavior_Types.ALTRUISTIC.value
        else:
            behavior = Vehicle_Behavior_Types.COOPERATIVE.value  # default value/ego-vehicle

        return behavior



    def calculate_smoothness(self, quantification_method='average'):

        for actor in self.vehicle_actors:
            id = actor.id
            acceleration = np.linalg.norm(np.array([actor.get_acceleration().x, actor.get_acceleration().y, actor.get_acceleration().z]))
            self.smoothness_tracker[id].append(acceleration)

            if quantification_method == 'average':
                self.vehicle_behaviors[id]['smoothness'] = self.average_value(self.smoothness_tracker[id])
            # todo: implement more quantification methods

    def calculate_lane_centering(self, quantification_method='average'):
        for actor in self.vehicle_actors:
            id = actor.id
            waypoint = self.map.get_waypoint(actor.get_location(), project_to_road=True)
            lane_center = waypoint.transform.location.y
            vehicle_y = actor.get_location().y
            offset = abs(vehicle_y-lane_center)
            self.lane_centering_tracker[id].append(offset)

            if quantification_method == 'average':
                self.vehicle_behaviors[id]['lane_centering'] = self.average_value( self.lane_centering_tracker[id])
                # todo: implement more quantification methods -- high pass filter??

    def calculate_speed_limit(self, quantification_method='average'):
        # todo: Observe and quantify speed limit of vehicle
        # for actor in self.vehicle_actors:
        #     id = actor.id
        #     vehicle_speed = np.linalg.norm(np.array([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]))
        #     speed_limit = actor.get_speed_limit()
        #     self.speed_limit_tracker[id].append(speed_limit)
        #     if quantification_method == 'average':
        #         self.vehicle_behaviors[id]['speed_limit'] = self.average_value(self.speed_limit_tracker[id])

        #Direct actor behavior label
        for actor in self.vehicle_actors:
            id = actor.id
            behavior = self.get_actor_behavior_type(actor)
            self.vehicle_behaviors[id]['speed_limit_percent'] = behavior['speed_limit_percent']

    def calculate_lane_changes(self, quantification_method='quantity'):
        for actor in self.vehicle_actors:
            id = actor.id
            waypoint = self.map.get_waypoint(actor.get_location(), project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
            current_lane = waypoint.lane_id
            self.lane_change_tracker[id].append(current_lane)

            if quantification_method == 'quantity':
                self.vehicle_behaviors[id]['lane_changes'] = self.count_value_changes(self.lane_change_tracker[id])
                # todo: implement more quantification methods -- high pass filter??


    def calculate_follow_time(self, quantification_method='average'):
        # todo: Observe and quantify following time
        # for actor in self.vehicle_actors:
        #     id = actor.id
        #     do something

        # Direct actor behavior label
        for actor in self.vehicle_actors:
            id = actor.id
            behavior = self.get_actor_behavior_type(actor)
            self.vehicle_behaviors[id]['follow_time'] = behavior['follow_time']

    def calculate_svo(self, estimation_type = 'generic'):
        if estimation_type == 'generic':
            self.generic_svo()
            return self.social_values
        elif estimation_type == 'none':
            self.no_svo()

        # self.calculate_smoothness('average')
        self.calculate_speed_limit('average')
        self.calculate_lane_centering('average')
        self.calculate_lane_changes('quantity')
        self.calculate_follow_time('average')

        if estimation_type == 'type_1':
            for actor in self.vehicle_actors:
                id = actor.id
                self.social_values[id] = self.type1_fuzzy.calculate_output(self.vehicle_behaviors[id])
        elif estimation_type == 'type_2':
            for actor in self.vehicle_actors:
                id = actor.id
                self.social_values[id] = self.type1_fuzzy.calculate_output(self.vehicle_behaviors[id])

        return self.social_values


    def generic_svo(self):
        for actor in self.vehicle_actors:
            id = actor.id
            if actor.attributes['role_name'] == 'sadistic':
                self.social_values[id] = Reference_SVO.SADISTIC.value
            elif actor.attributes['role_name'] == 'competitive':
                self.social_values[id] = Reference_SVO.COMPETITIVE.value
            elif actor.attributes['role_name'] == 'individualistic':
                self.social_values[id] = Reference_SVO.INDIVIDUALISTIC.value
            elif actor.attributes['role_name'] == 'cooperative':
                self.social_values[id] = Reference_SVO.COOPERATIVE.value
            elif actor.attributes['role_name'] == 'altruistic':
                self.social_values[id] = Reference_SVO.ALTRUISTIC.value
            else:
                self.social_values[id] = Reference_SVO.INDIVIDUALISTIC.value


    def no_svo(self):
        for actor in self.vehicle_actors:
            id = actor.id
            self.social_values[id] = 0