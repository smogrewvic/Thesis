from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
import enum


class Scenario:
    def __init__(self, scenario_name):
        self.cars = []
        self.pedestrians = []
        if scenario_name == ('empty_world'):
            pass

        elif scenario_name == 'pedestrian_crossing':
            self.cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)

            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)


        elif scenario_name == '3way_junction':
            self.cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)

            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)


    def get_pedestrians(self):
        return self.pedestrians
    def get_cars(self):
        return self.cars