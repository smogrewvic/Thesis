import PotentialField as pf
import carla
import random
from agents.navigation.global_route_planner import GlobalRoutePlanner
from GradientDescentPathPlanner import Gradient_path_planner


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    potential_field = pf.APF()
    high_level_route = GlobalRoutePlanner(world.get_map(), 5)
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

    # path_planner = Gradient_path_planner()
    while True:



        potential_field.generate_APF()
        potential_field.plot_actor_positions()

        # potential_field.draw_APF()
        potential_field.save_image_APF()
        potential_field.show_APF()




