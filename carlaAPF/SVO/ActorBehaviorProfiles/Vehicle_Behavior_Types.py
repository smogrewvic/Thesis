import enum


class Vehicle_Behavior_Types(enum.Enum):
    SADISTIC = {'follow_time': 0.25,
                'distance_to_leading_vehicle': 1,
                'keep_right_rule_percentage': 0,
                'random_left_lanechange_percentage': 100,
                'random_right_lanechange_percentage': 100,
                'vehicle_lane_offset': 1,
                'vehicle_percentage_speed_difference': -60,  # for internal carla use (160%)
                'speed_limit_percent': 160
                }

    COMPETITIVE = {'follow_time': 0.5,
                   'distance_to_leading_vehicle': 2,
                   'keep_right_rule_percentage': 10,
                   'random_left_lanechange_percentage': 25,
                   'random_right_lanechange_percentage': 25,
                   'vehicle_lane_offset': 0.2,
                   'vehicle_percentage_speed_difference': -40,  # for internal carla use (140%)
                   'speed_limit_percent': 140
                   }

    INDIVIDUALISTIC = {'follow_time': 1,
                       'distance_to_leading_vehicle': 4,
                       'keep_right_rule_percentage': 25,
                       'random_left_lanechange_percentage': 10,
                       'random_right_lanechange_percentage': 10,
                       'vehicle_lane_offset': 0.1,
                       'vehicle_percentage_speed_difference': -20,   # for internal carla use (120%)
                       'speed_limit_percent': 120
                       }

    COOPERATIVE = {'follow_time': 2,
                   'distance_to_leading_vehicle': 8,
                   'keep_right_rule_percentage': 50,
                   'random_left_lanechange_percentage': 3,
                   'random_right_lanechange_percentage': 3,
                   'vehicle_lane_offset': 0,
                   'vehicle_percentage_speed_difference': 0,  # for internal carla use (100%)
                   'speed_limit_percent': 100
                   }

    ALTRUISTIC = {'follow_time': 4,
                  'distance_to_leading_vehicle': 16,
                  'keep_right_rule_percentage': 100,
                  'random_left_lanechange_percentage': 0,
                  'random_right_lanechange_percentage': 0,
                  'vehicle_lane_offset': 0,
                  'vehicle_percentage_speed_difference': 20,   # for internal carla use (80%)
                  'speed_limit_percent': 80
                  }

