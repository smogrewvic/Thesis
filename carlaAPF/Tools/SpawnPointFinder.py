import carla
# import matplotlib.pyplot as plt
import plotly.express as px
import numpy as np


def plot_points(points):
    data = np.array(points)

    titles = []
    for pt in points:
        x = str(round(pt[0],5))
        y = str(round(pt[1],5))
        z = str(round(pt[2],5))
        pitch = str(round(pt[3],5))
        roll = str(round(pt[4],5))
        yaw = str(round(pt[5],5))

        name = ['x: ' + x + '\n' +
                'y: ' + y + '\n' +
                'z: ' + z + '\n' +
                'pitch: ' + pitch + '\n' +
                'roll: ' + roll + '\n' +
                'yaw: ' + yaw + '\n']

        titles.append(name)

    fig = px.scatter(x=-data[:, 0],  # flip x-axis display
                     y=data[:, 1],
                     color=data[:, 3],
                     hover_name=titles,
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
                             point.rotation.pitch,
                             point.rotation.roll,
                             point.rotation.yaw,
                             False])

    if ego_vehicle:
        spawn_points.append([ego_vehicle.get_location().x,
                             ego_vehicle.get_location().y,
                             ego_vehicle.get_location().z,
                             True])

    plot_points(spawn_points)
