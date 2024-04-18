import carla
from ApfObjects.PedestrianAPF import PedestrianAPF
from ApfObjects.VehicleAPF import VehicleAPF
from ApfObjects.NavpointAPF import NavpointAPF
from ApfObjects.RegressionLaneAPF import Regression_Lane_APF
from ApfObjects.TrafficLightAPF import TrafficLightAPF

import matplotlib.pyplot as plt


class Actor_State_Recorder:
    def __init__(self, pf_actors, world):
        self.world = world
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        self.actor_states_memory = {}
        self.actors = pf_actors
        self.plot_data = []
        self.color_info_map = {'altruistic': (0 / 255, 155 / 255, 0 / 255),
                               'cooperative': (150 / 255, 255 / 255, 50 / 255),
                               'individualistic': (255 / 255, 255 / 255, 0 / 255),
                               'competitive': (255 / 255, 128 / 255, 0 / 255),
                               'sadistic': (155 / 255, 0 / 255, 0 / 255)}
        self.sim_time_factor = 3.83
    def _svo_to_color(self, svo):
        behavior = 'individualistic'
        if svo >= 0.6:
            behavior = 'sadistic'
        elif 0.6 > svo > 0.2:
            behavior = 'competitive'
        elif 0.2 >= svo >= -0.2:
            behavior = 'individualistic'
        elif -0.2 > svo > -0.6:
            behavior = 'cooperative'
        elif svo < -0.6:
            behavior = 'altruistic'

        return self.color_info_map[behavior]

    def _velocity_to_transparecy(self, velocity):
        transparency_min = 0.1
        transparency_max = 1
        velocity_min = 0
        velocity_max = 5  # m/s

        transparency = (velocity - velocity_min) / (velocity_max - velocity_min) * (transparency_max - transparency_min) + transparency_min
        return max(min(transparency, transparency_max), transparency_min)

    def record_data(self, filters=['vehicles']):
        for id in self.actors:
            if id == 'ego_vehicle' and 'ego_vehicle' in filters:
                rgb = (0, 0, 255 / 255)  # blue
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                data = {'sim_time': current_time - self.start_time,
                        'x': self.actors[id].get_state()['position'][0],
                        'y': self.actors[id].get_state()['position'][1],
                        'a_x': self.actors[id].get_state()['acceleration'][0],
                        'a_y': self.actors[id].get_state()['acceleration'][1],
                        'a_z': self.actors[id].get_state()['acceleration'][2],
                        'color': rgba}

                self.plot_data.append(data)

            elif type(self.actors[id]) is VehicleAPF and 'vehicles' in filters:
                rgb = self._svo_to_color(self.actors[id].get_svo())
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                data = {'sim_time': current_time - self.start_time,
                        'x': self.actors[id].get_state()['position'][0],
                        'y': self.actors[id].get_state()['position'][1],
                        'a_x': self.actors[id].get_state()['acceleration'][0],
                        'a_y': self.actors[id].get_state()['acceleration'][1],
                        'a_z': self.actors[id].get_state()['acceleration'][2],
                        'color': rgba}
                self.plot_data.append(data)

            elif type(self.actors[id]) is NavpointAPF and 'navpoints' in filters:
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                data = {'sim_time': current_time - self.start_time,
                        'x': self.actors[id].get_state()['position'][0],
                        'y': self.actors[id].get_state()['position'][1],
                        'a_x': self.actors[id].get_state()['acceleration'][0],
                        'a_y': self.actors[id].get_state()['acceleration'][1],
                        'a_z': self.actors[id].get_state()['acceleration'][2],
                        'color': 'pink'}
                self.plot_data.append(data)

            elif type(self.actors[id]) is TrafficLightAPF and 'traffic_lights' in filters:
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                data = {'sim_time': current_time - self.start_time,
                        'x': self.actors[id].get_state()['position'][0],
                        'y': self.actors[id].get_state()['position'][1],
                        'a_x': self.actors[id].get_state()['acceleration'][0],
                        'a_y': self.actors[id].get_state()['acceleration'][1],
                        'a_z': self.actors[id].get_state()['acceleration'][2],
                        'color': 'pink'}
                self.plot_data.append(data)

    def plot_positions(self):
        print('inside plot actor')
        plt.cla()

        x = [data['x'] for data in self.plot_data]
        y = [data['y'] for data in self.plot_data]
        colors = [data['color'] for data in self.plot_data]

        plt.scatter(x, y, c=colors)
        # plt.plot(time, x_ax, color=colors, linestyle='solid')
        # plt.draw()
        plt.pause(0.01)

    def plot_accelerations(self):
        print('inside actor accel')
        # plt.cla()
        # plt.clf()

        x_ax = [data['a_x'] for data in self.plot_data]
        x_ay = [data['a_y'] for data in self.plot_data]
        x_az = [data['a_z'] for data in self.plot_data]

        time = [data['sim_time'] for data in self.plot_data]
        color = self.plot_data[-1]['color']

        fig, axs = plt.subplots(3, 1, figsize=(15, 10))

        # Plot for subplot 1
        axs[0].plot(time, x_ax, color=color)
        axs[0].set_title('Longitudinal Acceleration (x)')
        axs[0].set_ylabel('Acceleration (m/s$^2$)')
        axs[0].set_xlabel('Time (s)')

        # Plot for subplot 2
        axs[1].plot(time, x_ay, color='green')
        axs[1].set_title('Lateral Acceleration (y)')
        axs[1].set_ylabel('Acceleration (m/s$^2$)')
        axs[1].set_xlabel('Time (s)')

        # Plot for subplot 3
        axs[2].plot(time, x_az, color='red')
        axs[2].set_title('Vertical Acceleration (z)')
        axs[2].set_ylabel('Acceleration (m/s$^2$)')
        axs[2].set_xlabel('Time (s)')

        plt.subplots_adjust(hspace=0.5)

        plt.pause(0.01)

    def plot_comparison(self, filter):
        pass
