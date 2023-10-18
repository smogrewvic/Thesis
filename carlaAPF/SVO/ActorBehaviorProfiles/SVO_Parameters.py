import random

class Pedestrian_Queues:
    def __init__(self):
        self.time_looking_at_traffic = None
        self.distance_to_crosswalk = None
        # self.distraction_score = None

    def update_parameters(self, actor):
        pass
        # self.time_looking_at_traffic = actor.get_time_at_crosswalk()
        # self.distance_to_crosswalk = actor.get_distance_to_crosswalk()
        

class Vehicle_Queues:
    def __init__(self):
        self.following_distance = None
        self.merge_speed_change = None
        self.lane_changes = None
        self.speed_limit = None
        self.smoothness = None

    def random(self):
        self.following_distance = random.uniform(0, 5)
        self.merge_speed_change = random.uniform(-100, 100)
        self.lane_changes = random.uniform(0, 10)
        self.speed_limit = random.uniform(0, 200)
        self.smoothness = random.uniform(0, 5)

    def altruistic(self):
        self.following_distance = 5
        self.merge_speed_change = -100
        self.lane_changes = 0
        self.speed_limit = 60
        self.smoothness = 0.1

    def cooperative(self):
        self.following_distance = 2
        self.merge_speed_change = -20
        self.lane_changes = 0.2
        self.speed_limit = 100
        self.smoothness = 0.5

    def individualistic(self):
        self.following_distance = 1.5
        self.merge_speed_change = 120
        self.lane_changes = 1
        self.speed_limit = 120
        self.smoothness = 1

    def competitive(self):
        self.following_distance = 1
        self.merge_speed_change = 150
        self.lane_changes = 2
        self.speed_limit = 140
        self.smoothness = 1.5

    def sadistic(self):
        self.following_distance = 0.25
        self.merge_speed_change = 200
        self.lane_changes = 5
        self.speed_limit = 160
        self.smoothness = 3
