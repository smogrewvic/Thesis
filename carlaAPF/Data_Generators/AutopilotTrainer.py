import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

from Data_Generators import PotentialFieldTrainer as pft
from PathPlanners.GradientDescentPathPlanner import Gradient_path_planner

from VehicleControllers.SteeringControlPID import Steering_Control_PID
from VehicleControllers.ThrottleControlPID import Throttle_Control_PID

from SVO.ActorBehaviorAnalysers.Pedestrian_Behavior_Analyser import Pedestrian_Behavior_Analyser
from SVO.ActorBehaviorAnalysers.Vehicle_Behavior_Analyser import Vehicle_Behavior_Analyser

from Data_Generators.TrainingDataManager import Training_Data_Manager
from Tools.SpawnPoints import Spawn_Points
import random

def main(autopilot_on=True, holonomic=False, display_apf=True, display_actors=False, display_control_sys=True, ego_position=False):
    client = carla.Client('localhost', 2000)
    world = client.get_world()

    # settings = world.get_settings()
    # settings.fixed_delta_seconds = 0.16
    # settings.max_substeps = 16
    # world.apply_settings(settings)




    potential_field = pft.APF_Trainer()
    high_level_route = GlobalRoutePlanner(world.get_map(), 1)
    ego_vehicle = None

    # find ego_actor spawn point
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    navpoint_transforms = []
    origin = ego_vehicle.get_location()
    destination_coords = random.choice(list(Spawn_Points.points.value.values()))
    destination = carla.Location(x=destination_coords[0], y=destination_coords[1], z=destination_coords[2])

    # TODO: Check for minimum distance?
    for waypoint in high_level_route.trace_route(origin, destination):
        navpoint_transforms.append(waypoint[0].transform)

    potential_field.set_navpoints(navpoint_transforms)
    path_planner = Gradient_path_planner(potential_field.get_potential_field())

    ###### Steering control ###### fabcde7883b00cc3949aebb6dacb97af940906da
    steering_PID = Steering_Control_PID(ego_vehicle,
                                        potential_field.get_granularity(),
                                        potential_field=potential_field.get_potential_field())
    steering_PID.set_PID_values(0.28, 0.08, 0)  # good values p = 1, i = 0, d = 0.8      p =0.3, i = 0.1, d = 0

    ##### Throttle Control #####
    throttle_PID = Throttle_Control_PID(ego_vehicle,
                                        potential_field.get_potential_field(),
                                        potential_field.get_granularity())
    throttle_PID.set_PID_values(0.1, 0.05, 0)

    # Traffic lights
    potential_field.set_traffic_lights()

    #Behavior analysis
    pedestrian_behavior_analyser = Pedestrian_Behavior_Analyser(world)
    vehicle_behavior_analyser = Vehicle_Behavior_Analyser(world)
    svo_all_actors = {}


    while True:
        svo_all_actors.update(pedestrian_behavior_analyser.calculate_svo())
        svo_all_actors.update(vehicle_behavior_analyser.calculate_svo())
        potential_field.update_svo_actors(svo_all_actors)

        potential_field.generate_APF()

        training_output = []
        if autopilot_on == True:
            if holonomic == True:
                navigation_path = path_planner.holonomic_gradient_descent()
            else:

                navigation_path = path_planner.phi_max_regressed_descent(0.7854)

            steering_PID.set_regression_precision(path_planner.get_regression_precision())
            steering_control_output = steering_PID.get_control_output(navigation_path)
            throttle_control_output = throttle_PID.get_control_output(navigation_path, 10, kph=True)

            if throttle_control_output >= 0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle_control_output, steer=steering_control_output, brake=0))
            elif throttle_control_output < 0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=steering_control_output, brake=throttle_control_output))

        if display_apf == True:
            path_planner.save_image_APF()
            path_planner.show_APF()

        if display_actors == True:
            potential_field.plot_actor_positions()

        if display_control_sys == True:
            steering_PID.display_PID_tracking()
            throttle_PID.display_PID_tracking()

        if ego_position == True:
            print("current",
                  round(ego_vehicle.get_location().x, 4), "\t",
                  round(ego_vehicle.get_location().y, 4), "\t",
                  round(ego_vehicle.get_location().z, 4), "\t",
                  round(ego_vehicle.get_transform().rotation.pitch, 4), "\t",
                  round(ego_vehicle.get_transform().rotation.roll, 4), "\t",
                  round(ego_vehicle.get_transform().rotation.yaw, 4), "\t",
                  )


        # svo_all_actors.update(pedestrian_behavior_analyser.calculate_svo())
        # svo_all_actors.update(vehicle_behavior_analyser.calculate_svo())
        #
        # print(svo_all_actors)
        # potential_field.update_svo_actors(svo_all_actors)
        # training_data = Training_Data_Manager()
        # training_inputs = potential_field.get_input_training_data()
        #
        # training_data.save_training_data(training_inputs[0],training_inputs[1],training_inputs[2], navigation_path)

if __name__ == '__main__':
    main()
