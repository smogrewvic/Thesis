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
        self.pedestrian_controller_list = pedestrian_controller_list
        self.pedestrian_attribute_map = {}
        self.crosswalk_trigger_distance = 2

        for i in range(1, len(self.pedestrian_controller_list), 2):
            actor = self.pedestrian_controller_list[i]
            id = actor.id
            self.pedestrian_attribute_map[id] = {'distance_to_crosswalk': float('inf'),
                                                 'start_time_waiting_to_cross': 0.0,
                                                 'last_wait_to_cross': 0.0,
                                                 'currently_waiting':False
                                                 }

    def crosswalk_behavior(self):

        for i in range(0, len(self.pedestrian_controller_list), 2):
            controller_ai = self.pedestrian_controller_list[i]
            actor = self.pedestrian_controller_list[i + 1]

            id = actor.id
            position = np.array([actor.get_location().x, actor.get_location().y, actor.get_location().z])
            distance_to_crosswalk = float('inf')
            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                distance_to_crosswalk = min(distance_to_crosswalk,
                                            np.linalg.norm(position - crosswalk))

            self.pedestrian_attribute_map[id]['distance_to_crosswalk'] = distance_to_crosswalk
            # print("DISTANCE TO CROSSWALK", distance_to_crosswalk)
            if distance_to_crosswalk <= self.crosswalk_trigger_distance or self.pedestrian_attribute_map[id]['currently_waiting']:
                self._wait_at_crosswalk(actor, controller_ai)

    def _wait_at_crosswalk(self, actor, controller_ai):
        current_time = time.time()
        id = actor.id

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
            print("No behavior_type found actor:", actor.id)
            return

        speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
        elapsed_time_waiting = current_time - self.pedestrian_attribute_map[id]['start_time_waiting_to_cross']



        wait_to_cross = current_time - self.pedestrian_attribute_map[id]['last_wait_to_cross'] > 5.0 and not self.pedestrian_attribute_map[id]['currently_waiting']
        continue_waiting = elapsed_time_waiting < behavior['wait_time_to_cross']
        start_crossing = speed == 0.0 and elapsed_time_waiting >= behavior['wait_time_to_cross']


        print("\nPEDESTRIAN:", actor.id, "speed", speed, "elapsed_time_waiting", elapsed_time_waiting)
        print("wait_to_cross",wait_to_cross, "continue_waiting",continue_waiting, "start_crossing",start_crossing)


        if wait_to_cross:
            controller_ai.set_max_speed(0)
            self.pedestrian_attribute_map[id]['start_time_waiting_to_cross'] = current_time
            self.pedestrian_attribute_map[id]['currently_waiting'] = True
            
        elif continue_waiting:
            return

        elif start_crossing:
            controller_ai.set_max_speed(1.4)
            self.pedestrian_attribute_map[id]['last_wait_to_cross'] = current_time
            self.pedestrian_attribute_map[id]['currently_waiting'] = False

    def change_bones(self):
        pass

    def set_behaviors(self):
        self.crosswalk_behavior()
