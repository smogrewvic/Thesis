
class Pedestrian_Behavior_Types:

    @staticmethod
    def sadistic():
        params = {'walking_speed': 10,
                  'speed_percent_change_before_crossing': 140,
                  'wait_time_to_cross': 0,
                  'look_at_traffic_time': 0,
                  'effect_distance_to_crosswalk': 0,
                  }

        return params

    @staticmethod
    def competitive():
        params = {'walking_speed': 8,
                  'speed_percent_change_before_crossing': 120,
                  'wait_time_to_cross': 0.5,
                  'look_at_traffic_time': 0.5,
                  'effect_distance_to_crosswalk': 1,
                  }

        return params

    @staticmethod
    def individualistic():
        params = {'walking_speed': 6,
                  'speed_percent_change_before_crossing': 100,
                  'wait_time_to_cross': 1,
                  'look_at_traffic_time': 1,
                  'effect_distance_to_crosswalk': 50,
                  }


        return params

    @staticmethod
    def cooperative():
        params = {'walking_speed': 4,
                  'speed_percent_change_before_crossing': 80,
                  'wait_time_to_cross': 2,
                  'look_at_traffic_time': 2,
                  'effect_distance_to_crosswalk': 50,
                  }


        return params

    @staticmethod
    def altruistic():
        params = {'walking_speed': 2,
                  'speed_percent_change_before_crossing': 60,
                  'wait_time_to_cross': 4,
                  'look_at_traffic_time': 4,
                  'effect_distance_to_crosswalk': 50,
                  }


        return params
