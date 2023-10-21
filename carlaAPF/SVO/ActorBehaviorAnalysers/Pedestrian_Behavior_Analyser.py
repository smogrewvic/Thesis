from Tools.Crosswalk_Info import Crosswalk_Info
from SVO.FuzzyControllers.Pedestrian_SVO_Fuzzy import Pedestrian_SVO_Fuzzy
import numpy as np
import time


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
    def update_time_looking(self):
        pass
    def calculate_svo(self):
        self._update_distances_to_crosswalks()
        self._update_time_waiting()

        for actor in self.pedestrian_actors:
            id = actor.id
            self.social_values[id] = self.fuzzy.calculate_output(self.pedestrian_behaviors[id])

        return self.social_values
