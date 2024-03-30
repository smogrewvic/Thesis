from Tools.SpawnPoints import Spawn_Points
import carla
import random


class VehicleInfo:
    def __init__(self, spawn_point_id='random', destination_point_id='random', model_category='sedan', behavior_type='individualistic',
                 behavior_override=False):

        if model_category == 'sedan':
            model_blueprint = 'vehicle.lincoln.mkz_2020'
        elif model_category == 'truck':
            model_blueprint = 'vehicle.tesla.cybertruck'
        elif model_category == 'bus':
            model_blueprint = 'vehicle.mitsubishi.fusorosa'
        else:
            print(model_category, "model_category not recognized, setting default sedan")
            model_blueprint = 'vehicle.lincoln.mkz_2020'

        color_info_map = {'altruistic': '0,204,0',
                          'cooperative': '128,255,0',
                          'individualistic': '255,255,0',
                          'competitive': '255,128,0',
                          'sadistic': '255,0,0'}
        model_color = color_info_map[behavior_type]

        if behavior_override == True:
            svo_value_map = {'altruistic': 0.75,
                             'cooperative': 0.35,
                             'individualistic': 0,
                             'competitive': -0.35,
                             'sadistic': -0.75}
            behavior_type = svo_value_map[behavior_type]

        if spawn_point_id == 'random':
            spawn_point = random.choice(list(Spawn_Points.points.value.values()))
        else:
            spawn_point = Spawn_Points.points.value[spawn_point_id]

        if destination_point_id == 'random':
            destination_transform = random.choice(list(Spawn_Points.points.value.values()))
        else:
            destination_transform = Spawn_Points.points.value[spawn_point_id]

        self.data = {'type': 'vehicle',
                     'model_blueprint': model_blueprint,
                     'model_color': model_color,
                     'spawn_id': spawn_point_id,
                     'spawn_transform': self.convert_point_to_carla_transform(spawn_point),
                     'destination_id': destination_point_id,
                     'destination_transform': destination_transform,
                     'behavior_type': behavior_type}



    def convert_point_to_carla_transform(self, point):

        return carla.Transform(carla.Location(x=point[0], y=point[1], z=point[2]),
                               carla.Rotation(pitch=point[3], yaw=point[4], roll=point[5]))




class PedestrianInfo:
    def __init__(self, spawn_point_id='random', destination_point_id='random', age_category='adult', behavior_type='individualistic',
                 behavior_override=False):

        if age_category == 'child':
            model_blueprint = 'vehicle.lincoln.mkz_2020'
        else:
            model_blueprint = 'vehicle.lincoln.mkz_2020'

        if behavior_override == True:
            svo_value_map = {'altruistic': 0.75,
                             'cooperative': 0.35,
                             'individualistic': 0,
                             'competitive': -0.35,
                             'sadistic': -0.75}
            behavior_type = svo_value_map[behavior_type]

        if spawn_point_id == 'random':
            spawn_point = random.choice(list(Spawn_Points.points.value.values()))
        else:
            spawn_point = Spawn_Points.points.value[spawn_point_id]

        if destination_point_id == 'random':
            destination_transform = random.choice(list(Spawn_Points.points.value.values()))
        else:
            destination_transform = Spawn_Points.points.value[spawn_point_id]

        self.data = {'type': 'vehicle',
                     'model_blueprint': model_blueprint,
                     'spawn_id': spawn_point_id,
                     'spawn_transform': self.convert_point_to_carla_transform(spawn_point),
                     'destination_id': destination_point_id,
                     'destination_transform': destination_transform,
                     'behavior_type': behavior_type}

    def convert_point_to_carla_transform(self, point):

        return carla.Transform(carla.Location(x=point[0], y=point[1], z=point[2]),
                               carla.Rotation(pitch=point[3], yaw=point[4], roll=point[5]))