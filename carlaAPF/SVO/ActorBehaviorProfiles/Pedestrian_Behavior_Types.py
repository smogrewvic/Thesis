
import enum

class Pedestrian_Behavior_Types(enum.Enum):
    SADISTIC = {'walking_speed': 10,
                'speed_percent_change_before_crossing': 140,
                'wait_time_to_cross': 0,
                'look_at_traffic_time': 0,
                'effect_distance_to_crosswalk': 1,
                }

    COMPETITIVE = {'walking_speed': 8,
                   'speed_percent_change_before_crossing': 120,
                   'wait_time_to_cross': 0.5,
                   'look_at_traffic_time': 0.5,
                   'effect_distance_to_crosswalk': 2,
                   }

    INDIVIDUALISTIC = {'walking_speed': 6,
                       'speed_percent_change_before_crossing': 100,
                       'wait_time_to_cross': 1,
                       'look_at_traffic_time': 1,
                       'effect_distance_to_crosswalk': 3,
                       }

    COOPERATIVE = {'walking_speed': 4,
                   'speed_percent_change_before_crossing': 80,
                   'wait_time_to_cross': 2,
                   'look_at_traffic_time': 2,
                   'effect_distance_to_crosswalk': 4,
                   }

    ALTRUISTIC = {'walking_speed': 2,
                  'speed_percent_change_before_crossing': 60,
                  'wait_time_to_cross': 4,
                  'look_at_traffic_time': 4,
                  'effect_distance_to_crosswalk': 5,
                  }
