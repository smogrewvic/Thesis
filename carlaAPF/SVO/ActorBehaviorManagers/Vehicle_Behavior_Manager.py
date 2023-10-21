import numpy as np
from SVO.ActorBehaviorProfiles.Vehicle_Behavior_Types import Vehicle_Behavior_Types

class Vehicle_Behavior_Manager():
    def __init__(self, vehicles_list, traffic_manager):
        self.vehicles_list = vehicles_list
        self.traffic_manager = traffic_manager

    def update_follow_time(self):
        for actor in self.vehicles_list:
            speed = round(np.linalg.norm([actor.get_velocity().x, actor.get_velocity().y, actor.get_velocity().z]), 4)
            if actor.attributes['role_name'] == 'sadistic':
                behavior = Vehicle_Behavior_Types.SADISTIC.value
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'competitive':
                behavior = Vehicle_Behavior_Types.COMPETITIVE.value
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'individualistic':
                behavior = Vehicle_Behavior_Types.INDIVIDUALISTIC.value
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'cooperative':
                behavior = Vehicle_Behavior_Types.COOPERATIVE.value
                distance = speed * behavior['follow_time']

            elif actor.attributes['role_name'] == 'altruistic':
                behavior = Vehicle_Behavior_Types.ALTRUISTIC.value
                distance = speed * behavior['follow_time']
            else:
                return  # retain default

            self.traffic_manager.distance_to_leading_vehicle(actor, distance)

    def update_behaviors(self):
        for actor in self.vehicles_list:
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
                return  # retain default

            self.traffic_manager.distance_to_leading_vehicle(actor, behavior['distance_to_leading_vehicle'])
            self.traffic_manager.keep_right_rule_percentage(actor, behavior['keep_right_rule_percentage'])
            self.traffic_manager.random_left_lanechange_percentage(actor, behavior['random_left_lanechange_percentage'])
            self.traffic_manager.random_right_lanechange_percentage(actor, behavior['random_right_lanechange_percentage'])
            self.traffic_manager.vehicle_lane_offset(actor, behavior['vehicle_lane_offset'])
            self.traffic_manager.vehicle_percentage_speed_difference(actor, behavior['vehicle_percentage_speed_difference'])

    def get_actor_svo_attributes(self):
        return self.svo_attributes