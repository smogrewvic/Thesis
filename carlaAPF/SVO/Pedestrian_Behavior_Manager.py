import carla
import numpy as np
from SVO.Pedestrian_Behavior_Types import Pedestrian_Behavior_Types


class Pedestrian_Behavior_Manager():
    def __init__(self, vehicles_list, traffic_manager):
        self.vehicles_list = vehicles_list
        self.traffic_manager = traffic_manager

    def update_follow_time(self):
        for actor in self.vehicles_list:
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
        for actor in self.vehicles_list:
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

            self.traffic_manager.distance_to_leading_vehicle(actor, behavior['distance_to_leading_vehicle'])
            self.traffic_manager.keep_right_rule_percentage(actor, behavior['keep_right_rule_percentage'])
            self.traffic_manager.random_left_lanechange_percentage(actor, behavior['random_left_lanechange_percentage'])
            self.traffic_manager.random_right_lanechange_percentage(actor, behavior['random_right_lanechange_percentage'])
            self.traffic_manager.vehicle_lane_offset(actor, behavior['vehicle_lane_offset'])
            self.traffic_manager.vehicle_percentage_speed_difference(actor, behavior['vehicle_percentage_speed_difference'])
