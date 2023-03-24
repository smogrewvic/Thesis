import PotentialField as pf
import carla
import random
from agents.navigation.global_route_planner import GlobalRoutePlanner

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
        # navpoint_transforms.append((waypoint[0].transform.location.x, waypoint[0].transform.location.y, waypoint[0].transform.location.z))
        navpoint_transforms.append(waypoint[0].transform)


    potential_field.set_navpoints(navpoint_transforms)

    while True:
        # potential_field.set_actor_states(world.get_actors())
        potential_field.generate_APF()
        potential_field.plot_actor_positions()
        potential_field.save_image_APF()
        potential_field.show_APF()

