from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
import enum


class Scenario:
    def __init__(self, scenario_name):
        self.cars = []
        self.pedestrians = []
        self.origin = None
        self.destination = None
        self.ego_delay = 0
        self.traffic_delay = 0

        if scenario_name == ('empty_world_1'):
            self.origin = 'id_113'
            self.destination = 'id_62'

        elif scenario_name == ('empty_world_2'):
            self.origin = 'id_117'
            self.destination = 'id_14'

        elif scenario_name == 'generic':
            self.origin = 'id_113'
            self.destination = 'id_62'

            self.cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)

            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)

        elif scenario_name == 'basic_merge':
            self.origin = 'id_66'
            self.destination = ('id_62',)
            self.ego_delay = 2.1
            self.traffic_delay = 0

            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_63', model_category='sedan', behavior_type='sadistic').data)

        elif scenario_name == 'pedestrian_crossing':
            self.origin = 'id_44'
            self.destination = ('id_63','id_152')
            # self.cars.append(
            #     VehicleInfo(spawn_point_id='id_16', destination_point_id='id_62', model_category='sedan', behavior_type='altruistic').data)
            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)

        elif scenario_name == 'blocked_road_lane_change':
            self.origin = 'id_15'
            self.destination = ( 'id_x01','id_66','id_62')
            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_62', model_category='sedan', behavior_type='immobile').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_16', destination_point_id='id_62', model_category='sedan', behavior_type='altruistic').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_x02', destination_point_id='id_62', model_category='sedan', behavior_type='individualistic').data)

            self.ego_delay = 10

        elif scenario_name == 'blocked_road_lane_change2':
            self.origin = 'id_15'
            self.destination = ( 'id_x01','id_66','id_62')
            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_62', model_category='sedan', behavior_type='immobile').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_16', destination_point_id='id_62', model_category='sedan', behavior_type='altruistic').data)
            self.ego_delay = 10

        else:
            print('No matching scenario found for: ', scenario_name)
    def get_pedestrians(self):
        return self.pedestrians

    def get_cars(self):
        return self.cars

    def get_origin(self):
        return self.origin

    def get_destination(self):
        return self.destination

    def get_traffic_delay(self):
        return self.traffic_delay

    def get_ego_delay(self):
        return self.ego_delay