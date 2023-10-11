import carla
import numpy as np
from SVO.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types
from Tools.Crosswalk_Info import Crosswalk_Info

import time



class Pedestrian_Behavior_Manager():
    def __init__(self,controller_pedestrian_list):
        """
        controller_pedestrian_list: [controller_ai1, actor1, controller_ai2, actor2, ... ]
        """
        self.crossing_state = {}
        self.svo_attributes = {}
        self.crosswalk_trigger_distance = 2.5

        self.controller_pedestrian_list = controller_pedestrian_list


        for i in range(1, len(self.controller_pedestrian_list), 2):
            actor = self.controller_pedestrian_list[i]
            id = actor.id
            self.crossing_state[id] = {'distance_to_crosswalk': float('inf'),
                                       'wait_start_time': 0.0,
                                       'currently_waiting': False,
                                       'crosswalk_coordinates': (float('inf'), float('inf'), float('inf'))
                                       }

            self.svo_attributes[id] = {'behavior_type': actor.attributes['role_name'],
                                       'time_looking': 0,
                                       'time_waiting': 0,
                                       'distance_to_crosswalk': float('inf'),
                                       'currently_crossing': False
                                       }

        print("TYPE BEHAVIOR", type(self.controller_pedestrian_list))
    def _crosswalk_behavior(self):

        for i in range(0, len(self.controller_pedestrian_list), 2):
            controller_ai = self.controller_pedestrian_list[i]
            actor = self.controller_pedestrian_list[i + 1]
            # print(actor.attributes)
            id = actor.id
            position = np.array([actor.get_location().x, actor.get_location().y, actor.get_location().z])
            distance_to_crosswalk = float('inf')
            crosswalk_coordinates = (0, 0, 0)
            ignore_crosswalk = Crosswalk_Info.crosswalk_pairs[self.crossing_state[id]['crosswalk_coordinates']]

            for crosswalk in Crosswalk_Info.numpy_crosswalk_points:
                if (crosswalk == ignore_crosswalk).all(): continue  # bool all elements

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
            behavior = Pedestrian_Behavior_Types.sadistic()
        elif self.svo_attributes[id]['behavior_type'] == 'competitive':
            behavior = Pedestrian_Behavior_Types.competitive()
        elif self.svo_attributes[id]['behavior_type'] == 'individualistic':
            behavior = Pedestrian_Behavior_Types.individualistic()
        elif self.svo_attributes[id]['behavior_type'] == 'cooperative':
            behavior = Pedestrian_Behavior_Types.cooperative()
        elif self.svo_attributes[id]['behavior_type'] == 'altruistic':
            behavior = Pedestrian_Behavior_Types.altruistic()
        else:
            print("No behavior_type found actor:", actor.id)
            return

        speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
        elapsed_time_waiting = current_time - self.crossing_state[id]['wait_start_time']
        self.svo_attributes[id]['time_waiting'] = elapsed_time_waiting

        wait_to_cross = current_time - self.crossing_state[id]['wait_start_time'] > 10.0 and not self.crossing_state[id]['currently_waiting']
        continue_waiting = elapsed_time_waiting < behavior['wait_time_to_cross']
        start_crossing = speed == 0.0 and elapsed_time_waiting >= behavior['wait_time_to_cross']

        if wait_to_cross:
            controller_ai.set_max_speed(0)
            self.crossing_state[id]['wait_start_time'] = current_time
            self.crossing_state[id]['currently_waiting'] = True

        elif continue_waiting:
            return

        elif start_crossing:
            self.crossing_state[id]['currently_waiting'] = False
            controller_ai.set_max_speed(1.4)

    def _change_bones(self):
        pass

    def set_pedestrians(self, world):
        # client = carla.Client('localhost', 2000)
        # world = client.get_world()

        pedestrians = list(world.get_actors().filter('walker*'))
        pedestrians.sort(key=lambda actor: actor.id)

        controllers = list(world.get_actors().filter('controller.ai.walker'))
        controllers.sort(key=lambda actor: actor.id)

        all_ids = []
        for i in range(len(pedestrians)):
            all_ids.append(controllers[i].id)
            all_ids.append(pedestrians[i].id)
        all_actors = world.get_actors(all_ids)

        print("\nIDS", all_ids)
        print("ACTORS", all_actors)

        self.controller_pedestrian_list = world.get_actors(all_ids)

        for i in range(1, len(self.controller_pedestrian_list), 2):
            actor = self.controller_pedestrian_list[i]
            id = actor.id
            self.crossing_state[id] = {'distance_to_crosswalk': float('inf'),
                                       'wait_start_time': 0.0,
                                       'currently_waiting': False,
                                       'crosswalk_coordinates': (float('inf'), float('inf'), float('inf'))
                                       }

            self.svo_attributes[id] = {'behavior_type': actor.attributes['role_name'],
                                       'time_looking': 0,
                                       'time_waiting': 0,
                                       'distance_to_crosswalk': float('inf'),
                                       'currently_crossing': False
                                       }

    def get_actor_svo_attributes(self):
        return self.svo_attributes
    def update_behaviors(self):
        self._crosswalk_behavior()


