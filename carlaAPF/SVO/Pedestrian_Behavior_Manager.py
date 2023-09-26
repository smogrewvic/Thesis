import carla
import numpy as np
from SVO.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types
from Tools.Crosswalk_Info import Crosswalk_Info
import time


class Pedestrian_Behavior_Manager():
    def __init__(self, pedestrian_list):
        self.pedestrian_list = pedestrian_list
        self.pedestrian_attribute_map = {}
        self.crosswalk_trigger_distance = 0.5

        for actor in self.pedestrian_list:
            id = actor.get_id()
            attributes = {'distance_to_crosswalk': float('inf'),
                          'start_time_waiting_to_cross': 0.0,
                          'last_wait_to_cross': float('inf')
                          }
            self.pedestrian_attribute_map[id] = attributes

    def calculate_distance_to_crosswalk(self):

        for actor in self.pedestrian_list:
            id = actor.get_id()
            position = np.array([actor.get_position().x, actor.get_position().y, actor.get_position().z])
            distance_to_crosswalk = float('inf')
            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                distance_to_crosswalk = min(distance_to_crosswalk,
                                            np.linalg.norm(position - crosswalk))

            self.pedestrian_attribute_map[id]['distance_to_crosswalk'] = distance_to_crosswalk
            if distance_to_crosswalk <= self.crosswalk_trigger_distance:
                self.wait_at_crosswalk(actor)

    def wait_at_crosswalk(self,actor):
        current_time = time.time()
        id = actor.get_id()

        if actor.attributes['role_name'] == 'sadistic':
            behavior = Pedestrian_Behavior_Types.sadistic()
        elif actor.attributes['role_name'] == 'competitive':
            behavior = Pedestrian_Behavior_Types.competitive()
        elif actor.attributes['role_name'] == 'individualistic':
            behavior = Pedestrian_Behavior_Types.individualistic()
        elif actor.attributes['role_name'] == 'cooperative':
            behavior = Pedestrian_Behavior_Types.cooperative()
        elif actor.attributes['role_name'] == 'altruistic':
            behavior = Pedestrian_Behavior_Types.altruistic()
        else:
            print("No behavior_type found actor:", actor.get_id())
            return

        elapsed_time_waiting = current_time - self.pedestrian_attribute_map[id]['start_time_waiting_to_cross']

        wait_to_cross = actor.get_speed() > 0.0 and current_time - self.pedestrian_attribute_map[id]['last_wait_to_cross'] > 5.0
        continue_waiting = actor.get_speed() == 0.0 and elapsed_time_waiting < behavior['wait_time_before_crossing']
        start_crossing = actor.get_speed() == 0.0 and elapsed_time_waiting >= behavior['wait_time_before_crossing']

        if wait_to_cross:
            actor.set_speed(0.0)
            self.pedestrian_attribute_map[id]['start_time_waiting_to_cross'] = current_time
        elif continue_waiting:
            return
        elif start_crossing:
            actor.set_speed(behavior['walking_speed'])
            self.pedestrian_attribute_map[id]['last_wait_to_cross'] = current_time





    def change_bones(self):
       pass

    def update_follow_time(self):
        for actor in self.pedestrian_list:
            speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
            if actor.attributes['role_name'] == 'sadistic':
                behavior = Pedestrian_Behavior_Types.sadistic()
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'competitive':
                behavior = Pedestrian_Behavior_Types.competitive()
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'individualistic':
                behavior = Pedestrian_Behavior_Types.individualistic()
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'cooperative':
                behavior = Pedestrian_Behavior_Types.cooperative()
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'altruistic':
                behavior = Pedestrian_Behavior_Types.altruistic()
                distance = speed * behavior['follow_time']
            else:
                return  # retain default

            self.traffic_manager.distance_to_leading_vehicle(actor, distance)

    def set_behaviors(self):
        for actor in self.pedestrian_list:
            if actor.attributes['role_name'] == 'sadistic':
                behavior = Pedestrian_Behavior_Types.sadistic()

            elif actor.attributes['role_name'] == 'competitive':
                behavior = Pedestrian_Behavior_Types.competitive()

            elif actor.attributes['role_name'] == 'individualistic':
                behavior = Pedestrian_Behavior_Types.individualistic()

            elif actor.attributes['role_name'] == 'cooperative':
                behavior = Pedestrian_Behavior_Types.cooperative()

            elif actor.attributes['role_name'] == 'altruistic':
                behavior = Pedestrian_Behavior_Types.altruistic()

            else:
                return  # retain default
