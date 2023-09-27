import carla
import numpy as np
from SVO.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types
from Tools.Crosswalk_Info import Crosswalk_Info
import time


class Pedestrian_Behavior_Manager():
    def __init__(self, pedestrian_controller_list):
        """
        pedestrian_controller_list: controller_ai1, actor1, controller_ai2, actor2...
        """
        print("initialized")
        self.pedestrian_controller_list = pedestrian_controller_list
        self.pedestrian_attribute_map = {}
        self.crosswalk_trigger_distance = 2

        for i in range(1, len(self.pedestrian_controller_list), 2):
            actor = self.pedestrian_controller_list[i]
            id = actor.get_id()
            self.pedestrian_attribute_map[id] = {'distance_to_crosswalk': float('inf'),
                                                 'start_time_waiting_to_cross': 0.0,
                                                 'last_wait_to_cross': float('inf')
                                                 }

    def crosswalk_behavior(self):

        for i in range(0, len(self.pedestrian_controller_list), 2):
            controller_ai = self.pedestrian_controller_list[i]
            actor = self.pedestrian_controller_list[i + 1]

            id = actor.get_id()
            print("crosswalk behavior",id)
            position = np.array([actor.get_position().x, actor.get_position().y, actor.get_position().z])
            distance_to_crosswalk = float('inf')
            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                distance_to_crosswalk = min(distance_to_crosswalk,
                                            np.linalg.norm(position - crosswalk))

            self.pedestrian_attribute_map[id]['distance_to_crosswalk'] = distance_to_crosswalk
            if distance_to_crosswalk <= self.crosswalk_trigger_distance:
                self._wait_at_crosswalk(actor, controller_ai)

    def _wait_at_crosswalk(self, actor, controller_ai):
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
        print("PEDESTRIAN SPEED:", actor, actor.get_speed())
        wait_to_cross = actor.get_speed() > 0.0 and current_time - self.pedestrian_attribute_map[id]['last_wait_to_cross'] > 5.0
        continue_waiting = actor.get_speed() == 0.0 and elapsed_time_waiting < behavior['wait_time_before_crossing']
        start_crossing = actor.get_speed() == 0.0 and elapsed_time_waiting >= behavior['wait_time_before_crossing']

        if wait_to_cross:
            controller_ai.stop()
            self.pedestrian_attribute_map[id]['start_time_waiting_to_cross'] = current_time
        elif continue_waiting:
            return
        elif start_crossing:
            controller_ai.start()
            self.pedestrian_attribute_map[id]['last_wait_to_cross'] = current_time

    def change_bones(self):
        pass

    def set_behaviors(self):
        self.crosswalk_behavior()
