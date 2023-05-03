import PotentialField as pf
import carla
import random
from agents.navigation.global_route_planner import GlobalRoutePlanner
from SteeringController import SteeringController
from ThrottleController import ThrottleController

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    potential_field = pf.APF()
    high_level_route = GlobalRoutePlanner(world.get_map(), 10)
    ego_vehicle = None

    #find ego_actor spawn point
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    #find current ego_vehicle location and find random point to navigate to
    navpoint_transforms = []
    origin = ego_vehicle.get_location()
    destination = random.choice(world.get_map().get_spawn_points()).location

    for waypoint in high_level_route.trace_route(origin, destination):
        navpoint_transforms.append(waypoint[0].transform)


    # potential_field.set_navpoints(navpoint_transforms)
    steering_control = SteeringController(potential_field.get_potential_field(), potential_field.get_granularity(), ego_vehicle)
    throttle_control = ThrottleController(potential_field.get_potential_field(), potential_field.get_granularity())

    while True:

        # apf_search_data = potential_field.search_box_lowest_potential()
        apf_search_data = potential_field.search_radius_lowest_potential(radius =1.8)
        steering_input = apf_search_data["normalized_angle"]
        throttle_input = throttle_control.get_throttle2(apf_search_data["absolute_position"]) # old
        # throttle_input = throttle_control.get_throttle(apf_search_data, 3)

        ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle_input, steer=steering_input))

        potential_field.generate_APF()
        # potential_field.plot_actor_positions()
        potential_field.draw_lowest_point()
        # throttle_control.draw_throttle_point()
        # potential_field.draw_debug()
        potential_field.draw_APF()
        potential_field.show_APF()



