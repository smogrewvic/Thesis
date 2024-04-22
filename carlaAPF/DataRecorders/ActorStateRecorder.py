import carla
from ApfObjects.PedestrianAPF import PedestrianAPF
from ApfObjects.VehicleAPF import VehicleAPF
from ApfObjects.NavpointAPF import NavpointAPF
from ApfObjects.RegressionLaneAPF import Regression_Lane_APF
from ApfObjects.TrafficLightAPF import TrafficLightAPF
import matplotlib.pyplot as plt
import numpy as np

class Actor_State_Recorder:
    def __init__(self, pf_actors, world):
        self.world = world
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        self.start_position = [0,0]
        self.actor_states_memory = {}
        self.actors = pf_actors
        self.plot_data = []
        self.color_info_map = {'altruistic': (0 / 255, 155 / 255, 0 / 255),
                               'cooperative': (150 / 255, 255 / 255, 50 / 255),
                               'individualistic': (255 / 255, 255 / 255, 0 / 255),
                               'competitive': (255 / 255, 128 / 255, 0 / 255),
                               'sadistic': (155 / 255, 0 / 255, 0 / 255)}
        self.sim_time_factor = 3.83
        self.recording_distance = 30
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
                if self.start_position == [0,0]:
                    self.start_position = [self.actors[id].get_state()['position'][0],
                                           self.actors[id].get_state()['position'][1]
                                           ]
                rgb = (0, 0, 255 / 255)  # blue
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'throttle': state['throttle'],
                        'brake': state['brake'],
                        'steering': state['steering'],
                        'svo': round(state['svo'],3),
                        'color': rgba}

                self.plot_data.append(data)

            elif type(self.actors[id]) is VehicleAPF and 'vehicles' in filters:
                if self.start_position == [0, 0]: continue
                rgb = self._svo_to_color(self.actors[id].get_svo())
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                distance = np.linalg.norm(self.actors[id].get_relative_state()["position"])
                if distance > self.recording_distance: continue
                data = {'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'throttle': state['throttle'],
                        'brake': state['brake'],
                        'steering': state['steering'],
                        'svo': round(state['svo'],3),
                        'color': rgba}
                self.plot_data.append(data)

            elif type(self.actors[id]) is NavpointAPF and 'navpoints' in filters:
                if self.start_position == [0, 0]: continue
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'sim_time': current_time - self.start_time,
                        'x': state[0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'color': 'pink'}
                self.plot_data.append(data)

            elif type(self.actors[id]) is TrafficLightAPF and 'traffic_lights' in filters:
                if self.start_position == [0, 0]: continue
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'color': 'pink'}
                self.plot_data.append(data)

    def plot_positions(self):
        print('Plotting Actor Postions')
        label_frequency = 3  # odd number to plot other actors
        # plt.cla()
        plt.clf()
        x = [data['x'] for data in self.plot_data]
        y = [data['y'] for data in self.plot_data]
        # y = [data['sim_time'] for data in self.plot_data]
        colors = [data['color'] for data in self.plot_data]
        labels = [data['svo'] for data in self.plot_data]

        plt.scatter(y, x, c=colors)


        for i, label in enumerate(labels):
            if i % label_frequency != 0 or label == 0:
                continue

            plt.annotate(label,  # this is the text
                         (y[i], x[i]),  # these are the coordinates to position the label
                         textcoords="offset points",  # how to position the text
                         xytext=(20, 0),  # distance from text to points (x,y)
                         ha='center')  # horizontal alignment can be left, right or center


        # plt.title('Actor Positions', fontsize=22)  # Set title font size
        plt.ylabel('Longitudinal Position (m)', fontsize=18)  # Set ylabel font size
        plt.xlabel('Lateral Position (m)', fontsize=18)  # Set xlabel font size

        plt.draw()
        plt.pause(0.01)


    def plot_accelerations(self):
        print('Plotting Actor Accelerations')
        # plt.cla()
        # plt.clf()

        x_ax = [data['a_x'] for data in self.plot_data]
        x_ay = [data['a_y'] for data in self.plot_data]
        steering = [data['steering'] for data in self.plot_data]
        throttle = [data['throttle']*100 for data in self.plot_data]
        brake = [data['brake']*100 for data in self.plot_data]

        time = [data['sim_time'] for data in self.plot_data]
        color = self.plot_data[-1]['color']

        fig, axs = plt.subplots(4, 1, figsize=(15, 10))

        # Plot for subplot 1
        axs[0].plot(time, x_ax, color=color)
        axs[0].set_title('Longitudinal Acceleration (x)', fontsize=14)
        axs[0].set_ylabel('Acceleration (m/s$^2$)', fontsize=14)
        axs[0].set_xlabel('Time (s)', fontsize=14)
        axs[0].tick_params(axis='x', labelsize=14)  # Set x ticks font size
        axs[0].tick_params(axis='y', labelsize=14)  # Set y ticks font size

        # Plot for subplot 2
        axs[1].plot(time, throttle, color='green')  # [0,1]
        axs[1].plot(time, brake, color='red')  # [0,1]
        axs[1].set_title('Throttle & Steering Inputs', fontsize=14)
        axs[1].set_ylabel('Proportion of Maximum (%)', fontsize=14)
        axs[1].set_xlabel('Time (s)', fontsize=14)
        axs[1].tick_params(axis='x', labelsize=14)  # Set x ticks font size
        axs[1].tick_params(axis='y', labelsize=14)  # Set y ticks font size

        axs[2].plot(time, x_ay, color='green')  # acceleration
        axs[2].set_title('Lateral Acceleration (y)', fontsize=14)
        axs[2].set_ylabel('Acceleration (m/s$^2$)', fontsize=14)
        axs[2].set_xlabel('Time (s)', fontsize=14)
        axs[2].tick_params(axis='x', labelsize=14)  # Set x ticks font size
        axs[2].tick_params(axis='y', labelsize=14)  # Set y ticks font size

        axs[3].plot(time, steering, color='green')  # acceleration
        axs[3].set_title('Lateral Acceleration (y)', fontsize=14)
        axs[3].set_ylabel('Acceleration (m/s$^2$)', fontsize=14)
        axs[3].set_xlabel('Time (s)', fontsize=14)
        axs[3].tick_params(axis='x', labelsize=14)  # Set x ticks font size
        axs[3].tick_params(axis='y', labelsize=14)  # Set y ticks font size



        plt.subplots_adjust(hspace=0.8)

        # plt.xticks(fontsize=14, rotation=45)

        plt.draw()

        plt.pause(0.01)

    def plot_comparison(self, filter):
        pass
