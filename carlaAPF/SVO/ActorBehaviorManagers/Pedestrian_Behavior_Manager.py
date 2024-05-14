import numpy as np
from SVO.ActorBehaviorProfiles.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types
from Tools.Crosswalk_Info import Crosswalk_Info
import carla
import time


class Pedestrian_Behavior_Manager():
    def __init__(self, controller_pedestrian_list):
        """
        controller_pedestrian_list: [controller_ai1, actor1, controller_ai2, actor2, ... ]
        """
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()
        self.controller_pedestrian_list = controller_pedestrian_list
        # self.all_vehicle_actors = self.world.get_actors().filter('vehicle.*')

        self.ego_vehicle = None  # ego_vehicle assigned later to have time to load in world

        self.crossing_state = {}
        self.svo_attributes = {}
        self.crosswalk_trigger_distance = Crosswalk_Info.trigger_distance

        for i in range(1, len(self.controller_pedestrian_list), 2):
            actor = self.controller_pedestrian_list[i]
            id = actor.id
            self.crossing_state[id] = {'distance_to_crosswalk': float('inf'),
                                       'wait_start_time': 0.0,
                                       'currently_waiting': False,
                                       'crosswalk_coordinates': (float('inf'), float('inf'), float('inf')),
                                       'currently_looking': False
                                       }

            self.svo_attributes[id] = {'behavior_type': actor.attributes['role_name'],
                                       'time_looking': 0,
                                       'time_waiting': 0,
                                       'distance_to_crosswalk': float('inf'),
                                       'currently_crossing': False
                                       }

    def _crosswalk_behavior(self):

        for i in range(0, len(self.controller_pedestrian_list), 2):
            controller_ai = self.controller_pedestrian_list[i]
            actor = self.controller_pedestrian_list[i + 1]
            # print(actor.attributes)
            id = actor.id
            position = np.array([actor.get_location().x, actor.get_location().y, actor.get_location().z])
            distance_to_crosswalk = float('inf')
            crosswalk_coordinates = (0, 0, 0)
            crosswalk_end_point = Crosswalk_Info.crosswalk_pairs[self.crossing_state[id]['crosswalk_coordinates']]

            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                if (crosswalk == crosswalk_end_point).all(): continue  # bool all elements

                if distance_to_crosswalk > np.linalg.norm(position - crosswalk):
                    distance_to_crosswalk = np.linalg.norm(position - crosswalk)
                    crosswalk_coordinates = tuple(crosswalk)  # convert to tuple for immutable dict key

            self.svo_attributes[id]['distance_to_crosswalk'] = distance_to_crosswalk
            if not self.crossing_state[id]['currently_waiting']: self.svo_attributes[id]['time_waiting'] = 0

            valid_crosswalk = distance_to_crosswalk <= self.crosswalk_trigger_distance or self.crossing_state[id]['currently_waiting']
            if valid_crosswalk:
                self.crossing_state[id]['crosswalk_coordinates'] = crosswalk_coordinates
                self._wait_at_crosswalk(actor, controller_ai)

    def _wait_at_crosswalk(self, actor, controller_ai):
        current_time = time.time()
        id = actor.id

        if self.svo_attributes[id]['behavior_type'] == 'sadistic':
            behavior = Pedestrian_Behavior_Types.SADISTIC.value
        elif self.svo_attributes[id]['behavior_type'] == 'competitive':
            behavior = Pedestrian_Behavior_Types.COMPETITIVE.value
        elif self.svo_attributes[id]['behavior_type'] == 'individualistic':
            behavior = Pedestrian_Behavior_Types.INDIVIDUALISTIC.value
        elif self.svo_attributes[id]['behavior_type'] == 'cooperative':
            behavior = Pedestrian_Behavior_Types.COOPERATIVE.value
        elif self.svo_attributes[id]['behavior_type'] == 'altruistic':
            behavior = Pedestrian_Behavior_Types.ALTRUISTIC.value
        else:
            print("No behavior_type found actor:", actor.id)
            return

        speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
        elapsed_time_waiting = current_time - self.crossing_state[id]['wait_start_time']
        self.svo_attributes[id]['time_waiting'] = elapsed_time_waiting

        wait_to_cross = current_time - self.crossing_state[id]['wait_start_time'] > 10.0 and not self.crossing_state[id]['currently_waiting']
        continue_waiting = elapsed_time_waiting < behavior['wait_time_to_cross'] and not self._safe_to_cross(actor)
        start_crossing = speed == 0.0 and elapsed_time_waiting >= behavior['wait_time_to_cross'] and self._safe_to_cross(actor)
        if wait_to_cross:
            controller_ai.set_max_speed(0)
            self.crossing_state[id]['wait_start_time'] = current_time
            self.crossing_state[id]['currently_waiting'] = True

        elif continue_waiting:
            return

        elif start_crossing:
            self.crossing_state[id]['currently_waiting'] = False
            controller_ai.set_max_speed(1.4)

    def _find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
                return actor

        print('Pedestrian_Behavior_Manager -- ego_vehicle not found')
        return 0

    def _calculate_look_angle(self, actor):
        look_point = [self.ego_vehicle.get_location().x, self.ego_vehicle.get_location().y]  # look at ego vehicle
        actor_point = [actor.get_location().x, actor.get_location().y]

        angle = np.arctan2(look_point[1], look_point[0]) - np.arctan2(actor_point[1], actor_point[0])  # angle in global coordinates

        return angle

    def _safe_to_cross(self, actor):

        id = actor.id

        if self.svo_attributes[id]['behavior_type'] == 'sadistic':
            behavior = Pedestrian_Behavior_Types.SADISTIC.value
        elif self.svo_attributes[id]['behavior_type'] == 'competitive':
            behavior = Pedestrian_Behavior_Types.COMPETITIVE.value
        elif self.svo_attributes[id]['behavior_type'] == 'individualistic':
            behavior = Pedestrian_Behavior_Types.INDIVIDUALISTIC.value
        elif self.svo_attributes[id]['behavior_type'] == 'cooperative':
            behavior = Pedestrian_Behavior_Types.COOPERATIVE.value
        elif self.svo_attributes[id]['behavior_type'] == 'altruistic':
            behavior = Pedestrian_Behavior_Types.ALTRUISTIC.value

        actor_position = np.array([round(actor.get_transform().location.x, 4),
                                   round(actor.get_transform().location.y, 4),
                                   round(actor.get_transform().location.z, 4)])
        min_distance = float('inf')

        all_vehicles = self.world.get_actors().filter('vehicle.*')
        for vehicle in all_vehicles:
            ego_position = np.array([round(vehicle.get_transform().location.x, 4),
                                     round(vehicle.get_transform().location.y, 4),
                                     round(vehicle.get_transform().location.z, 4)])

            min_distance = min(min_distance, np.linalg.norm(actor_position - ego_position))
        if min_distance > behavior['safe_to_cross_distance']:
            return True
        else:
            return False

    def _set_pedestrian_angle(self, look_angle, actor):
        actor_transform = actor.get_transform()
        actor_transform.rotation.yaw = look_angle  # modify transform to look angle
        actor_transform.location.x = 0
        actor_transform.location.y = 0
        actor_transform.location.z = 0
        # print("TRANSFORM", actor_transform)
        actor.set_transform(actor_transform)

    def _look_at_crosswalk_behavior(self):
        while not self.ego_vehicle:
            self.ego_vehicle = self._find_ego_vehicle()  # keep checking until ego_vehicle loads

        for i in range(0, len(self.controller_pedestrian_list), 2):
            controller_ai = self.controller_pedestrian_list[i]
            actor = self.controller_pedestrian_list[i + 1]
            id = actor.id

            if self.svo_attributes[id]['behavior_type'] == 'sadistic':
                behavior = Pedestrian_Behavior_Types.SADISTIC.value
            elif self.svo_attributes[id]['behavior_type'] == 'competitive':
                behavior = Pedestrian_Behavior_Types.COMPETITIVE.value
            elif self.svo_attributes[id]['behavior_type'] == 'individualistic':
                behavior = Pedestrian_Behavior_Types.INDIVIDUALISTIC.value
            elif self.svo_attributes[id]['behavior_type'] == 'cooperative':
                behavior = Pedestrian_Behavior_Types.COOPERATIVE.value
            elif self.svo_attributes[id]['behavior_type'] == 'altruistic':
                behavior = Pedestrian_Behavior_Types.ALTRUISTIC.value
            else:
                print("No behavior_type found actor:", actor.id)
                return

            # if self.crossing_state[id]['currently_waiting']:
            #
            #     remaining_time = behavior['wait_time_to_cross'] - self.svo_attributes[id]['time_waiting']
            #     look_at_traffic = 0 <= remaining_time <= behavior['look_at_traffic_time']  # bool
            #
            #     if look_at_traffic:
            #         print("looking")
            #         # self.crossing_state[id]['currently_looking'] = True
            #         look_angle = self._calculate_look_angle(actor)
            #         self._set_pedestrian_angle(look_angle, actor)

            if self.crossing_state[id]['currently_waiting']:
                look_angle = self._calculate_look_angle(actor)
                controller_ai.stop()
                self._set_pedestrian_angle(180, actor)
                controller_ai.start()

            print("transform", actor.get_transform())
        print('\n')

    def get_actor_svo_attributes(self):
        return self.svo_attributes

    def update_behaviors(self):
        self._crosswalk_behavior()
        # self._look_at_crosswalk_behavior() #TODO: NOT IMPLEMENTED
