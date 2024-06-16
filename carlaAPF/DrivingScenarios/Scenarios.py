from TrafficGenerators.ActorInfo import VehicleInfo, PedestrianInfo
import random
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
            self.destination = ( 'id_x01','id_66','id_62')

            self.cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
            self.cars.append(
                VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)

            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)

            self.ego_delay = 1

        elif scenario_name == 'basic_merge':
            self.origin = 'id_66'
            self.destination = ('id_62',)
            self.ego_delay = 2.1
            self.traffic_delay = 0

            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_63', model_category='sedan', behavior_type='sadistic').data)

        elif scenario_name == 'pedestrian_crossing':
            self.origin = 'id_41'
            self.destination = ('id_63','id_152')

            self.pedestrians.append(
                PedestrianInfo(spawn_point_id='id_481', destination_point_id='id_539', model_category='adult', behavior_type='competitive').data)
            self.ego_delay = 8.2

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

        elif scenario_name == 'training_short':
            self.origin = 'id_66'
            self.destination = ('id_147','id_32', 'id_29', 'id_10', 'id_51', 'id_3', 'id_66')
            self.ego_delay = 1
            self.traffic_delay = 0

            # self.cars.append(VehicleInfo(spawn_point_id='id_66', destination_point_id='id_12', model_category='sedan', behavior_type='sadistic').data)
            # self.cars.append(
            #     VehicleInfo(spawn_point_id='id_31', destination_point_id='id_12', model_category='sedan', behavior_type='altruistic').data)
            # self.cars.append(
            #     VehicleInfo(spawn_point_id='id_113', destination_point_id='id_12', model_category='sedan', behavior_type='individualistic').data)
            #
            # self.pedestrians.append(
            #     PedestrianInfo(spawn_point_id='id_172', destination_point_id='id_754', model_category='adult', behavior_type='cooperative').data)
            # self.pedestrians.append(
            #     PedestrianInfo(spawn_point_id='id_52', destination_point_id='id_172', model_category='adult', behavior_type='sadistic').data)

        elif scenario_name == 'training_lane_changes':
            self.origin = 'id_66'

            self.destination = ('id_147', 'id_32','id_77', 'id_4','id_10', 'id_51','id_3', 'id_66')
            self.ego_delay = 1
            self.traffic_delay = 0

        elif scenario_name == 'training_30c_20p':
            self.origin = 'id_66'

            self.destination = ('id_147', 'id_32','id_77', 'id_4','id_10', 'id_51','id_3', 'id_66')
            self.ego_delay = 1
            self.traffic_delay = 0


            self.cars.append(VehicleInfo(spawn_point_id='id_113', destination_point_id='id_26', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_128', destination_point_id='id_127', behavior_type='competitive').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_29', destination_point_id='id_76', behavior_type='cooperative').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_26', destination_point_id='id_44', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_114', destination_point_id='id_12', behavior_type='sadistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_100', destination_point_id='id_3', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_13', destination_point_id='id_82', behavior_type='competitive').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_60', destination_point_id='id_60', behavior_type='cooperative').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_154', destination_point_id='id_74', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_98', destination_point_id='id_25', behavior_type='altruistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_50', destination_point_id='id_31', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_8', destination_point_id='id_91', behavior_type='competitive').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_106', destination_point_id='id_23', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_23', destination_point_id='id_112', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_20', destination_point_id='id_92', behavior_type='cooperative').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_85', destination_point_id='id_80', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_59', destination_point_id='id_97', behavior_type='competitive').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_102', destination_point_id='id_8', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_2', destination_point_id='id_154', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_45', destination_point_id='id_64', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_30', destination_point_id='id_2', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_74', destination_point_id='id_145', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_60', destination_point_id='id_113', behavior_type='cooperative').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_82', destination_point_id='id_128', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_3', destination_point_id='id_29', behavior_type='competitive').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_12', destination_point_id='id_50', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_44', destination_point_id='id_8', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_76', destination_point_id='id_100', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_127', destination_point_id='id_82', behavior_type='individualistic').data)
            self.cars.append(VehicleInfo(spawn_point_id='id_26', destination_point_id='id_122', behavior_type='cooperative').data)

            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_481', destination_point_id='id_75', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_1', destination_point_id='id_299', behavior_type='competitive').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_388', destination_point_id='id_68', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_986', destination_point_id='id_274', behavior_type='competitive').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_569', destination_point_id='id_813', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_475', destination_point_id='id_463', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_591', destination_point_id='id_88', behavior_type='cooperative').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_675', destination_point_id='id_443', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_28', destination_point_id='id_330', behavior_type='altruistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_582', destination_point_id='id_481', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_266', destination_point_id='id_1', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_798', destination_point_id='id_388', behavior_type='cooperative').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_317', destination_point_id='id_569', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_836', destination_point_id='id_475', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_135', destination_point_id='id_21', behavior_type='competitive').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_886', destination_point_id='id_869', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_733', destination_point_id='id_539', behavior_type='sadistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_722', destination_point_id='id_746', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_472', destination_point_id='id_111', behavior_type='individualistic').data)
            self.pedestrians.append(PedestrianInfo(spawn_point_id='id_512', destination_point_id='id_634', behavior_type='cooperative').data)


        elif scenario_name == 'random_30_cars_20_pedestrians':

            behaviors = ['altruistic', 'cooperative', 'individualistic', 'competitive', 'sadistic']

            count_cars = {'altruistic': 0, 'cooperative': 0, 'individualistic': 0, 'competitive': 0, 'sadistic': 0}
            count_pedestrians = {'altruistic': 0, 'cooperative': 0, 'individualistic': 0, 'competitive': 0, 'sadistic': 0}

            for i in range(30):
                behavior = random.choices(population=behaviors, weights=[5, 15, 60, 15, 5])[0]
                count_cars[behavior] += 1
                self.cars.append(VehicleInfo(spawn_point_id='random', destination_point_id='random', behavior_type=behavior).data)

            for i in range(20):
                behavior = random.choices(population=behaviors, weights=[5, 15, 60, 15, 5])[0]
                count_pedestrians[behavior] += 1
                self.pedestrians.append(PedestrianInfo(spawn_point_id='random', destination_point_id='random', behavior_type=behavior).data)

            print('VEHICLE BEHAVIORS', count_cars)
            print('PEDESTRIAN BEHAVIORS', count_pedestrians)

        elif scenario_name == 'random_60_cars_40_pedestrians':

            behaviors = ['altruistic', 'cooperative', 'individualistic', 'competitive', 'sadistic']

            count_cars = {'altruistic': 0, 'cooperative': 0, 'individualistic': 0, 'competitive': 0, 'sadistic': 0}
            count_pedestrians = {'altruistic': 0, 'cooperative': 0, 'individualistic': 0, 'competitive': 0, 'sadistic': 0}

            for i in range(60):
                behavior = random.choices(population=behaviors, weights=[5, 15, 60, 15, 5])[0]
                count_cars[behavior] += 1
                self.cars.append(VehicleInfo(spawn_point_id='random', destination_point_id='random', behavior_type=behavior).data)

            for i in range(40):
                behavior = random.choices(population=behaviors, weights=[5, 15, 60, 15, 5])[0]
                count_pedestrians[behavior] += 1
                self.pedestrians.append(PedestrianInfo(spawn_point_id='random', destination_point_id='random', behavior_type=behavior).data)

            print('VEHICLE BEHAVIORS', count_cars)
            print('PEDESTRIAN BEHAVIORS', count_pedestrians)
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