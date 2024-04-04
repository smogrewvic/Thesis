from Tools.Crosswalk_Info import Crosswalk_Info
from SVO.FuzzyEstimators.Pedestrian_SVO_Fuzzy import Pedestrian_SVO_Fuzzy
from SVO.ActorBehaviorProfiles.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types
import numpy as np
import time

from SVO.ActorBehaviorProfiles.Reference_SVO import Reference_SVO

class Pedestrian_Behavior_Analyser:
    def __init__(self, world):
        self.world = world
        self.crosswalk_trigger_distance = Crosswalk_Info.trigger_distance + Crosswalk_Info.trigger_overshoot
        self.fuzzy = Pedestrian_SVO_Fuzzy()

        self.pedestrian_actors = self.filter_actors('pedestrian')
        self.social_values = {}
        self.pedestrian_behaviors = {}
        self.pedestrian_time_tracker = {}
        for actor in self.pedestrian_actors:
            id = actor.id
            self.pedestrian_behaviors[id] = {'distance_to_crosswalk': float('inf'),
                                             'time_waiting': 0.0,
                                             'time_looking': 0.0}

            self.pedestrian_time_tracker[id] = {'wait_start_time': 0.0,
                                                'look_start_time': 0.0,
                                                'last_speed': 1,
                                                'last_looking': float('inf')}

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

    def _update_distances_to_crosswalks(self):
        for actor in self.pedestrian_actors:
            id = actor.id
            position = np.array([actor.get_location().x, actor.get_location().y, actor.get_location().z])
            distance_to_crosswalk = float('inf')

            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                distance_to_crosswalk = min(distance_to_crosswalk, np.linalg.norm(position - crosswalk))

            self.pedestrian_behaviors[id]['distance_to_crosswalk'] = distance_to_crosswalk

    def _calculate_head_pose(self):
        pass

    def _update_time_waiting(self):
        current_time = time.time()

        for actor in self.pedestrian_actors:
            id = actor.id
            speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)

            is_at_crosswalk = self.pedestrian_behaviors[id]['distance_to_crosswalk'] <= self.crosswalk_trigger_distance
            started_waiting = is_at_crosswalk and speed == 0 and self.pedestrian_time_tracker[id]['last_speed'] > 0
            waiting = is_at_crosswalk and speed == 0 and self.pedestrian_time_tracker[id]['last_speed'] == 0
            finished_waiting = speed > 0 and self.pedestrian_time_tracker[id]['last_speed'] == 0

            if started_waiting:
                self.pedestrian_time_tracker[id]['wait_start_time'] = current_time
            elif waiting:
                elapsed_time = current_time - self.pedestrian_time_tracker[id]['wait_start_time']
                self.pedestrian_behaviors[id]['time_waiting'] = elapsed_time
            elif finished_waiting:
                self.pedestrian_behaviors[id]['time_waiting'] = 0.0

            self.pedestrian_time_tracker[id]['last_speed'] = speed

    def _update_time_looking(self):
        #TODO: Implement true looking behavior analysis, not artificial chronometer
        for actor in self.pedestrian_actors:
            id = actor.id

            if actor.attributes['role_name'] == 'sadistic':
                behavior = Pedestrian_Behavior_Types.SADISTIC.value
            elif actor.attributes['role_name'] == 'competitive':
                behavior = Pedestrian_Behavior_Types.COMPETITIVE.value
            elif actor.attributes['role_name'] == 'individualistic':
                behavior = Pedestrian_Behavior_Types.INDIVIDUALISTIC.value
            elif actor.attributes['role_name'] == 'cooperative':
                behavior = Pedestrian_Behavior_Types.COOPERATIVE.value
            elif actor.attributes['role_name'] == 'altruistic':
                behavior = Pedestrian_Behavior_Types.ALTRUISTIC.value
            else:
                print("No behavior_type found actor:", actor.id)
                return

            if self.pedestrian_behaviors[id]['time_waiting'] > 0:
                remaining_time = behavior['wait_time_to_cross'] - self.pedestrian_behaviors[id]['time_waiting']
                looking_at_traffic = 0 <= remaining_time <= behavior['look_at_traffic_time']  # bool

                if looking_at_traffic:
                    self.pedestrian_behaviors[id]['time_looking'] = behavior['look_at_traffic_time'] - remaining_time
            else:
                self.pedestrian_behaviors[id]['time_looking'] = 0

    def calculate_svo(self, override_svo = False):
        if override_svo == True:
            self.generic_svo()
            return self.social_values

        self._update_distances_to_crosswalks()
        self._update_time_waiting()
        self._update_time_looking()

        for actor in self.pedestrian_actors:
            id = actor.id
            self.social_values[id] = self.fuzzy.calculate_output(self.pedestrian_behaviors[id])

        return self.social_values


    def generic_svo(self):
        for actor in self.pedestrian_actors:
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