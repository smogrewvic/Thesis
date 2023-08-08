import carla
# import matplotlib.pyplot as plt
import plotly.express as px
import numpy as np


def plot_points(points):
    data = np.array(points)

    fig = px.scatter(x=-data[:, 0],  # flip x-axis display
                     y=data[:, 1],
                     color=data[:, 3],
                     hover_name=points,
                     range_color=[0, 2],
                     title="Spawn Points")

    fig.show()


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    map = world.get_map()

    ego_vehicle = None
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    spawn_points = []
    for point in map.get_spawn_points():
        spawn_points.append([point.location.x,
                             point.location.y,
                             point.location.z,
                             False])

    if ego_vehicle:
        spawn_points.append([ego_vehicle.get_location().x,
                             ego_vehicle.get_location().y,
                             ego_vehicle.get_location().z,
                             True])

    plot_points(spawn_points)
