from ApfObjects import PotentialField as pf
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from PathPlanners.GradientDescentPathPlanner import Gradient_path_planner
from VehicleControllers.SteeringControlPID import Steering_Control_PID
from VehicleControllers.ThrottleControlPID import Throttle_Control_PID


def main(autopilot_on = True, holonomic = False, display_apf = True, display_actors = False, display_control_sys = True):
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    potential_field = pf.APF()
    high_level_route = GlobalRoutePlanner(world.get_map(), 1)
    ego_vehicle = None

    #find ego_actor spawn point
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    #find current ego_vehicle location and find random point to navigate to
    navpoint_transforms = []
    origin = ego_vehicle.get_location()
    destination = carla.Location(x=105.868706, y=72.938820, z=0.000000)

    for waypoint in high_level_route.trace_route(origin, destination):
        navpoint_transforms.append(waypoint[0].transform)

    potential_field.set_navpoints(navpoint_transforms)
    path_planner = Gradient_path_planner(potential_field.get_potential_field())

    ###### Steering control ######
    steering_PID = Steering_Control_PID(ego_vehicle,
                                        potential_field.get_granularity(),
                                        potential_field = potential_field.get_potential_field())
    steering_PID.set_PID_values(0.28, 0.08, 0) # good values p = 1, i = 0, d = 0.8      p =0.3, i = 0.1, d = 0

    ##### Throttle Control #####
    throttle_PID = Throttle_Control_PID(ego_vehicle,
                                        potential_field.get_potential_field(),
                                        potential_field.get_granularity())
    throttle_PID.set_PID_values(0.1, 0.05, 0)

    while True:
        potential_field.generate_APF()

        if holonomic == True:
            navigation_path = path_planner.holonomic_gradient_descent()
        else:
            # navigation_path = path_planner.phi_max_gradient_descent(0.7854)
            navigation_path = path_planner.phi_max_regressed_descent(0.7854)


        steering_PID.set_regression_precision(path_planner.get_regression_precision())
        steering_control_output = steering_PID.get_control_output(navigation_path)

        throttle_control_output = throttle_PID.get_control_output(navigation_path, 10, kph = True)


        if autopilot_on == True:
            if throttle_control_output>=0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle_control_output, steer=steering_control_output, brake = 0))
            elif throttle_control_output<0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0, steer=steering_control_output, brake=throttle_control_output))

        if display_apf == True:
            path_planner.save_image_APF()
            path_planner.show_APF()

        if display_actors == True:
            potential_field.plot_actor_positions()

        if display_control_sys == True:
            # steering_PID.display_PID_tracking()
            throttle_PID.display_PID_tracking()

if __name__ == '__main__':
    main()