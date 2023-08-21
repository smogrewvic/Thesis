import carla
# import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from SpawnPoints import Spawn_Points


def plot_points(ego_data):
    data = []
    titles = []
    for key in Spawn_Points.points.value:
        x = str(round(Spawn_Points.points.value[key][0], 7))
        y = str(round(Spawn_Points.points.value[key][1], 7))
        z = str(round(Spawn_Points.points.value[key][2], 7))
        pitch = str(round(Spawn_Points.points.value[key][3], 7))
        roll = str(round(Spawn_Points.points.value[key][4], 7))
        yaw = str(round(Spawn_Points.points.value[key][5], 7))

        name = ['id: ' + key + '\t\t\n' +
                'x: ' + x + '\n' +
                'y: ' + y + '\n' +
                'z: ' + z + '\n' +
                'pitch: ' + pitch + '\n' +
                'roll: ' + roll + '\n' +
                'yaw: ' + yaw + '\n']

        titles.append(name)
        data.append(Spawn_Points.points.value[key])

    data = np.array(data)

    fig_spawn_points = px.scatter(x=-data[:, 0],  # flip x-axis display
                                  y=data[:, 1],
                                  color=[0] * len(data),
                                  hover_name=titles,
                                  range_color=[0, 2],
                                  title="Spawn Points, use Spawn_Points.points.value['id_VALUE']")

    ego_data = np.array(ego_data)
    if len(ego_data)>0:
        fig_ego = px.scatter(x=-ego_data[:, 0],  # flip x-axis display
                             y=ego_data[:, 1],
                             color=[1],
                             hover_name=["ego"],
                             range_color=[0, 2]
                             )

        fig = go.Figure(data=fig_spawn_points.data + fig_ego.data)
    else:
        fig = fig_spawn_points

    fig.show()


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    map = world.get_map()

    ego_vehicle = None
    for actor in world.get_actors():
        if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'ego_vehicle':
            ego_vehicle = actor

    ego_data = []
    if ego_vehicle:
        ego_data.append([ego_vehicle.get_location().x,
                         ego_vehicle.get_location().y,
                         ego_vehicle.get_location().z,
                         ego_vehicle.get_transform().rotation.pitch,
                         ego_vehicle.get_transform().rotation.roll,
                         ego_vehicle.get_transform().rotation.yaw]
                        )

    plot_points(ego_data)
