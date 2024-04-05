import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

from ApfObjects import PotentialField as pf
from PathPlanners.GradientDescentPathPlanner import Gradient_path_planner

from VehicleControllers.SteeringControlPID import Steering_Control_PID
from VehicleControllers.ThrottleControlPID import Throttle_Control_PID

from SVO.ActorBehaviorAnalysers.Pedestrian_Behavior_Analyser import Pedestrian_Behavior_Analyser
from SVO.ActorBehaviorAnalysers.Vehicle_Behavior_Analyser import Vehicle_Behavior_Analyser
from Tools.SpawnPoints import Spawn_Points


def main(destination_id = 'id_113', autopilot_on=True, display_apf=True, display_actors=False, display_control_sys=True, generic_svo=False):
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    potential_field = pf.APF()
    high_level_route = GlobalRoutePlanner(world.get_map(), 1)
    ego_vehicle = None

    # find ego_vehicle in Carla
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    # find current ego_vehicle location and find random point to navigate to
    navpoint_transforms = []
    origin = ego_vehicle.get_location()
    # destination = carla.Location(x=105.868706, y=72.938820, z=0.000000)
    d = Spawn_Points.points.value[destination_id]
    destination = carla.Location(x=d[0], y=d[1], z=d[2])

    for waypoint in high_level_route.trace_route(origin, destination):
        navpoint_transforms.append(waypoint[0].transform)

    potential_field.set_navpoints(navpoint_transforms)
    path_planner = Gradient_path_planner(potential_field.get_potential_field())

    ###### Steering control ######
    steering_PID = Steering_Control_PID(ego_vehicle, potential_field.get_granularity(), potential_field=potential_field.get_potential_field())
    steering_PID.set_PID_values(0.25, 0, 0)  # good turning response
    steering_PID.set_look_ahead(20)

    ##### Throttle Control #####
    throttle_PID = Throttle_Control_PID(ego_vehicle, potential_field.get_potential_field(), potential_field.get_granularity())

    throttle_PID.set_PID_values(0.6, 0.001, 0)  # good for 20kph setpoint

    # Traffic lights
    potential_field.set_traffic_lights()

    # Behavior analysis
    pedestrian_behavior_analyser = Pedestrian_Behavior_Analyser(world)
    vehicle_behavior_analyser = Vehicle_Behavior_Analyser(world)
    svo_all_actors = {}

    while True:
        svo_all_actors.update(pedestrian_behavior_analyser.calculate_svo(override_svo=generic_svo))
        svo_all_actors.update(vehicle_behavior_analyser.calculate_svo(override_svo=generic_svo))
        potential_field.update_svo_actors(svo_all_actors)

        potential_field.generate_APF()

        if autopilot_on == True:
            navigation_path = path_planner.phi_max_regressed_descent(0.7854)
            steering_PID.set_regression_precision(path_planner.get_regression_precision())
            steering_control_output = steering_PID.get_control_output(navigation_path)
            throttle_control_output = throttle_PID.get_control_output(navigation_path, 20, kph=True)

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
            # steering_PID.display_PID_tracking()
            throttle_PID.display_PID_tracking()


        # # Print ego-vehicle location
        # print("current",
        #       round(ego_vehicle.get_location().x, 4), "\t",
        #       round(ego_vehicle.get_location().y, 4), "\t",
        #       round(ego_vehicle.get_location().z, 4), "\t",
        #       round(ego_vehicle.get_transform().rotation.pitch, 4), "\t",
        #       round(ego_vehicle.get_transform().rotation.roll, 4), "\t",
        #       round(ego_vehicle.get_transform().rotation.yaw, 4), "\t",
        #       )


if __name__ == '__main__':
    main()
