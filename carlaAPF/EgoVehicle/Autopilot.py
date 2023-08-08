from ApfObjects import PotentialField as pf
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from PathPlanners.GradientDescentPathPlanner import Gradient_path_planner
from VehicleControllers.SteeringControlPID import Steering_Control_PID


def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    potential_field = pf.APF()
    high_level_route = GlobalRoutePlanner(world.get_map(), 1)
    ego_vehicle = None

    #find ego_actor spawn point
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    print("ego_vehicle value", ego_vehicle.get_location())
    #find current ego_vehicle location and find random point to navigate to
    navpoint_transforms = []
    origin = ego_vehicle.get_location()
    print("origin", origin)
    # destination = random.choice(world.get_map().get_spawn_points()).location
    destination = carla.Location(x=105.868706, y=72.938820, z=0.000000)

    for waypoint in high_level_route.trace_route(origin, destination):
        navpoint_transforms.append(waypoint[0].transform)

    print("navpoint[0]", navpoint_transforms[0])
    potential_field.set_navpoints(navpoint_transforms)
    path_planner = Gradient_path_planner(potential_field.get_potential_field())

    ###### Steering control ######
    steering_PID = Steering_Control_PID(ego_vehicle,
                                        potential_field.get_granularity(),
                                        potential_field = potential_field.get_potential_field())
    steering_PID.set_PID_values(1, 0.0, 0.8)


    while True:

        potential_field.generate_APF()
        # potential_field.plot_actor_positions()

        # navigation_path = path_planner.holonomic_gradient_descent()
        # navigation_path = path_planner.phi_max_gradient_descent(0.7854)
        navigation_path = path_planner.phi_max_regressed_descent(0.7854)
        steering_PID.set_regression_precision(path_planner.get_regression_precision())

        path_planner.save_image_APF()
        path_planner.show_APF()

        steering_control_output = steering_PID.get_control_output(navigation_path)
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.30, steer=steering_control_output))
        # steering_PID.display_PID_tracking()




if __name__ == '__main__':
    main()