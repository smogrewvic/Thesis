
from SVO.FuzzyControllers.Vehicle_SVO_Fuzzy import Vehicle_SVO_Fuzzy
import numpy as np
import time


class Vehicle_Behavior_Analyser:
    def __init__(self, world):
        self.world = world
        self.fuzzy = Vehicle_SVO_Fuzzy()
        self.social_values = {}
        self.vehicle_actors = self.filter_actors('vehicle')

        for actor in self.vehicle_actors:
            self.vehicle_behaviors[id] = {'following_distance': 0,
                                          'lane_changes': 0,
                                          'lane_centering': 0,
                                          'speed_limit': 0.0,
                                          'smoothness': 0.0}
            id = actor.id
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

    def calculate_svo(self):

        #Todo: PLACE HOLDER
        for actor in self.vehicle_actors:
            id = actor.id
            # self.social_values[id] = self.fuzzy.calculate_output(self.vehicle_behaviors[id])
            self.social_values[id] = 0

        return self.social_values