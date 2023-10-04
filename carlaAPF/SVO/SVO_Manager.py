
from Vehicle_Behavior_Manager import Vehicle_Behavior_Manager
from Pedestrian_Behavior_Manager import Pedestrian_Behavior_Manager
from Vehicle_SVO_Fuzzy import Vehicle_SVO_Fuzzy
from Pedestrian_SVO_Fuzzy import Pedestrian_SVO_Fuzzy
class SVO_Manager():

    def __init__(self):

        self.actor_social_value_sigmas = {}
        self.pedestrian_behavior_manager = Pedestrian_Behavior_Manager()
        self.vehicle_behavior_manager = Vehicle_Behavior_Manager()

        self.pedestrian_fuzzy = Pedestrian_SVO_Fuzzy()
        self.vehicle_fuzzy = Vehicle_SVO_Fuzzy()
    def update_actor_behaviors(self):
        self.pedestrian_behavior_manager.update_behaviors()
        self.vehicle_behavior_manager.update_behaviors()

    def calculate_SVO(self):
        pedestrian_svo_attributes = self.pedestrian_behavior_manager.get_actor_svo_attributes()
        vehicle_svo_attributes = self.vehicle_behavior_manager.get_actor_svo_attributes()


    def get_all_svo(self):
        return self.actor_social_value_sigmas