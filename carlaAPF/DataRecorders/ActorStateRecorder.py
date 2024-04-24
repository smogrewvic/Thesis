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
        self.start_position = [0, 0]
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
        self.record_delay = 2  # seconds
        self.record_time = 8  # seconds

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
                if self.start_position == [0, 0]:
                    self.start_position = [self.actors[id].get_state()['position'][0],
                                           self.actors[id].get_state()['position'][1]
                                           ]
                rgb = (0, 0, 255 / 255)  # blue
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'type': 'ego_vehicle',
                        'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'a_z': state['acceleration'][2],
                        'throttle': state['throttle'],
                        'brake': state['brake'],
                        'steering': state['steering'],
                        'svo': round(state['svo'], 3),
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
                data = {'type': 'vehicle',
                        'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'a_z': state['acceleration'][2],
                        'throttle': state['throttle'],
                        'brake': state['brake'],
                        'steering': state['steering'],
                        'svo': round(state['svo'], 3),
                        'relative_distance': distance,
                        'color': rgba}
                self.plot_data.append(data)

            elif type(self.actors[id]) is NavpointAPF and 'navpoints' in filters:
                if self.start_position == [0, 0]: continue
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'type': 'navpoint',
                        'sim_time': current_time - self.start_time,
                        'x': state[0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'a_z': state['acceleration'][2],
                        'color': 'pink'}
                self.plot_data.append(data)

            elif type(self.actors[id]) is TrafficLightAPF and 'traffic_lights' in filters:
                if self.start_position == [0, 0]: continue
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                data = {'type': 'traffic_light',
                        'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'a_z': state['acceleration'][2],
                        'color': 'pink'}
                self.plot_data.append(data)

    def plot_positions(self):
        print('Plotting Actor Positions')
        label_frequency = 3  # odd number to plot other actors
        # plt.cla()
        plt.clf()

        start_index, stop_index = self.get_start_stop_index()

        x = [data['x'] for data in self.plot_data[start_index:stop_index]]
        y = [data['y'] for data in self.plot_data[start_index:stop_index]]
        colors = [data['color'] for data in self.plot_data[start_index:stop_index]]
        labels = [data['svo'] for data in self.plot_data[start_index:stop_index]]

        plt.scatter(y, x, c=colors)

        for i, label in enumerate(labels):
            if i % label_frequency != 0 or label == 0:
                continue

            plt.annotate(label,  # this is the text
                         (y[i], x[i]),  # these are the coordinates to position the label
                         textcoords="offset points",  # how to position the text
                         xytext=(20, 0),  # distance from text to points (x,y)
                         ha='left')  # horizontal alignment can be left, right or center

        closest_distance = float('inf')
        for i, data in enumerate(self.plot_data[start_index:stop_index]):
            if data['type'] == 'vehicle':
                closest_distance = min(closest_distance, data['relative_distance'])

        plt.annotate('Closest Distance: ' + str(round(closest_distance, 3)),
                     (10, 3),
                     textcoords="offset points",
                     xytext=(20, 0),
                     ha='left')

        total_distance = 0
        for i in range(1, len(x)):
            total_distance += (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2

        plt.annotate('Total Distance: ' + str(round(total_distance, 3)),
                     (y[-1], x[-1]),
                     textcoords="offset points",
                     xytext=(20, 0),
                     ha='left')

        plt.ylabel('Longitudinal Position (m)', fontsize=18)  # Set ylabel font size
        plt.xlabel('Lateral Position (m)', fontsize=18)  # Set xlabel font size

        plt.draw()
        plt.pause(0.01)

    def plot_accelerations(self):
        print('Plotting Actor Accelerations')

        start_index, stop_index = self.get_start_stop_index()
        x_ax_raw, x_ay_raw, steering, throttle, brake, time = [], [], [], [], [], []

        for data in self.plot_data[start_index:stop_index]:
            if data['type'] == 'ego_vehicle':
                x_ax_raw.append(data['a_x'])
                x_ay_raw.append(data['a_y'])
                steering.append(data['steering'] * 100)
                throttle.append(data['throttle'] * 100)
                brake.append(data['brake'] * 100)
                time.append(data['sim_time'])

        x_ax = self.low_pass_filter(x_ax_raw, alpha=0.4)
        x_ay = self.low_pass_filter(x_ay_raw, alpha=0.4)

        fig, axs = plt.subplots(4, 1, figsize=(17, 10))

        # Plot for subplot 1
        axs[0].plot(time, x_ax, color='blue')
        axs[0].set_title('Longitudinal Acceleration (x)', fontsize=18, weight='bold')
        axs[0].set_ylabel('Accel. (m/s$^2$)', fontsize=14, weight='bold')
        axs[0].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[0].tick_params(axis='x', labelsize=14)
        axs[0].tick_params(axis='y', labelsize=14)

        # Plot for subplot 2
        axs[1].plot(time, throttle, color='green')  # [0,1]
        axs[1].plot(time, brake, color='red')  # [0,1]
        axs[1].set_title('Throttle & Brake Inputs', fontsize=18, weight='bold')
        axs[1].set_ylabel('Pedal Position (%)', fontsize=14, weight='bold')
        axs[1].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[1].tick_params(axis='x', labelsize=14)
        axs[1].tick_params(axis='y', labelsize=14)

        axs[2].plot(time, x_ay, color='blue')  # acceleration
        axs[2].set_title('Lateral Acceleration (y)', fontsize=18, weight='bold')
        axs[2].set_ylabel('Accel.   (m/s$^2$)', fontsize=14, weight='bold')
        axs[2].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[2].tick_params(axis='x', labelsize=14)
        axs[2].tick_params(axis='y', labelsize=14)

        axs[3].plot(time, steering, color='green')  # acceleration
        axs[3].set_title('Steering Input', fontsize=18, weight='bold')
        axs[3].set_ylabel('Steering Input (%)', fontsize=14, weight='bold')
        axs[3].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[3].tick_params(axis='x', labelsize=14)
        axs[3].tick_params(axis='y', labelsize=14)

        plt.subplots_adjust(hspace=1.0)

        plt.draw()

        plt.pause(0.01)
        print('RMS',self.rms_acceleration(), '\tWEIGHTED RMS:', self.weighted_rms_acceleration())


    def plot_relative_distance(self):
        print('Plotting Relative Distance')
        label_frequency = 3
        start_index, stop_index = self.get_start_stop_index()

        distance = []
        time = []
        labels = []
        closest_distance = float('inf')
        for i, data in enumerate(self.plot_data[start_index:stop_index]):
            if data['type'] == 'vehicle':
                distance.append(data['relative_distance'])
                time.append(data['sim_time'])
                labels.append(data['svo'])
                closest_distance = min(closest_distance, data['relative_distance'])

        plt.scatter(distance, time, c='red')

        for i, label in enumerate(labels):
            if i % label_frequency != 0 or label == 0:
                continue

            plt.annotate(label,  # this is the text
                         (distance[i], time[i]),  # these are the coordinates to position the label
                         textcoords="offset points",  # how to position the text
                         xytext=(20, 0),  # distance from text to points (x,y)
                         ha='center')  # horizontal alignment can be left, right or center

        plt.annotate('Closest Distance: ' + str(round(closest_distance, 3)),
                     (10, 3),
                     textcoords="offset points",
                     xytext=(20, 0),
                     ha='center')

        plt.ylabel('Relative Distance (m)', fontsize=18)
        plt.xlabel('Time (s)', fontsize=18)

        plt.draw()
        plt.pause(0.01)

    def get_start_stop_index(self):
        start_index = 0
        stop_index = len(self.plot_data)
        for i, data in enumerate(self.plot_data):
            if start_index == 0 and data['sim_time'] >= self.record_delay:
                start_index = i
            if data['sim_time'] >= self.record_time:
                stop_index = i
                break
        return start_index, stop_index


    def low_pass_filter(self, data, alpha=0.1):
        filtered_data = [data[0]]  # Initialize filtered data with first value
        for i in range(1, len(data)):
            filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[-1]
            filtered_data.append(filtered_value)
        return filtered_data

    def rms_acceleration(self):
        return self.weighted_rms_acceleration(1,1,1)
    def weighted_rms_acceleration(self, k_x=1.4, k_y=1.4, k_z=1):

        a_x, a_y, a_z = [], [], []

        start_index, stop_index = self.get_start_stop_index()
        for i, data in enumerate(self.plot_data[start_index:stop_index]):
            if data['type'] == 'ego_vehicle':
                a_x.append(data['a_x'])
                a_y.append(data['a_y'])
                a_z.append(data['a_z'])

        rms_w = np.sqrt((k_x * np.mean(a_x)) ** 2 + (k_y * np.mean(a_y)) ** 2 + (k_z * np.mean(a_z)) ** 2)

        return rms_w

    def plot_comparison(self, filter):
        pass
