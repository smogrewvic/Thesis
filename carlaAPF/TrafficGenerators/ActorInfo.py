from Tools.SpawnPoints import Spawn_Points
# import carla
import random


class VehicleInfo:
    def __init__(self, spawn_point_id='random', destination_point_id='random', model_category='sedan', behavior_type='individualistic',
                 static_svo=False, autopilot_state=True):

        if model_category == 'sedan':
            model_blueprint = 'vehicle.lincoln.mkz_2020'
        elif model_category == 'truck':
            model_blueprint = 'vehicle.tesla.cybertruck'
        elif model_category == 'bus':
            model_blueprint = 'vehicle.mitsubishi.fusorosa'
        else:
            print(model_category, "model_category not recognized, setting default sedan")
            model_blueprint = 'vehicle.lincoln.mkz_2020'

        color_info_map = {'altruistic': '0,155,0',
                          'cooperative': '150,255,50',
                          'individualistic': '255,255,0',
                          'competitive': '255,128,0',
                          'sadistic': '155,0,0',
                          'immobile': '255,255,255'}
        model_color = color_info_map[behavior_type]

        if static_svo == True:
            svo_value_map = {'altruistic': 0.75,
                             'cooperative': 0.35,
                             'individualistic': 0,
                             'competitive': -0.35,
                             'sadistic': -0.75,
                             'immobile':0}
            behavior_type = svo_value_map[behavior_type]

        if spawn_point_id == 'random':
            spawn_location = random.choice(list(Spawn_Points.points.value.values()))
        else:
            spawn_location = Spawn_Points.points.value[spawn_point_id]

        if destination_point_id == 'random':
            destination_location = random.choice(list(Spawn_Points.points.value.values()))
        else:
            destination_location = Spawn_Points.points.value[destination_point_id]

        self.data = {'type': 'vehicle',
                     'model_blueprint': model_blueprint,
                     'model_color': model_color,
                     'spawn_id': spawn_point_id,
                     'spawn_location': spawn_location,
                     'destination_id': destination_point_id,
                     'destination_location': destination_location,
                     'behavior_type': behavior_type,
                     'autopilot_state': autopilot_state}


class PedestrianInfo:
    def __init__(self, spawn_point_id='random', destination_point_id='random', model_category='sedan', behavior_type='individualistic',
                 static_svo=False, autopilot_state=True):

        # pedestrian colors can not be set directly.
        if model_category == 'adult':
            if behavior_type == 'altruistic':
                model_blueprint = 'walker.pedestrian.0006'  # dark green shirt
            if behavior_type == 'cooperative':
                model_blueprint = 'walker.pedestrian.0005'  # light green shirt
            if behavior_type == 'individualistic':
                model_blueprint = 'walker.pedestrian.0004'  # yellow shirt
            if behavior_type == 'competitive':
                model_blueprint = 'walker.pedestrian.0007'  # purple shirt
            if behavior_type == 'sadistic':
                model_blueprint = 'walker.pedestrian.0008'  # bright orange shirt
            if behavior_type == 'immobile':
                model_blueprint = ' walker.pedestrian.0032'  # police officer
        elif model_category == 'child':
            model_blueprint = 'walker.pedestrian.0009'  # no relevant color options for children
        else:
            print(model_category, "model_category not recognized, setting default adult pedestrian")
            model_blueprint = 'walker.pedestrian.0004'


        if static_svo==True:
            svo_value_map = {'altruistic': 0.75,
                             'cooperative': 0.35,
                             'individualistic': 0,
                             'competitive': -0.35,
                             'sadistic': -0.75,
                             'immobile': 0}
            behavior_type = svo_value_map[behavior_type]

        if spawn_point_id == 'random':
            spawn_location = random.choice(list(Spawn_Points.pedestrian_points.value.values()))
        else:
            spawn_location = Spawn_Points.pedestrian_points.value[spawn_point_id]

        if destination_point_id == 'random':
            destination_location = random.choice(list(Spawn_Points.pedestrian_points.value.values()))
        else:
            destination_location = Spawn_Points.pedestrian_points.value[destination_point_id]

        self.data = {'type': 'vehicle',
                     'model_blueprint': model_blueprint,
                     'spawn_id': spawn_point_id,
                     'spawn_location': spawn_location,
                     'destination_id': destination_point_id,
                     'destination_location': destination_location,
                     'behavior_type': behavior_type,
                     'autopilot_state': autopilot_state}
