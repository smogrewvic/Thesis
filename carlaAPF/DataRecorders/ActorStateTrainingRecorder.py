import carla
from ApfObjects.PedestrianAPF import PedestrianAPF
from ApfObjects.VehicleAPF import VehicleAPF
from ApfObjects.NavpointAPF import NavpointAPF
from ApfObjects.RegressionLaneAPF import Regression_Lane_APF
from ApfObjects.TrafficLightAPF import TrafficLightAPF
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.rcParams['text.usetex'] = True
import numpy as np
import os
import copy
import re


class Actor_State_Recorder:
    def __init__(self, pf_actors, world, svo_estimation_type):
        self.world = world
        self.start_time = self.world.get_snapshot().timestamp.elapsed_seconds
        self.start_position = [0, 0]
        self.actor_states_memory = {}
        self.actors = pf_actors
        self.svo_estimation_type = svo_estimation_type
        self.plot_data = []
        self.color_info_map = {'altruistic': (0 / 255, 155 / 255, 0 / 255),
                               'cooperative': (150 / 255, 255 / 255, 50 / 255),
                               'individualistic': (255 / 255, 255 / 255, 0 / 255),
                               'competitive': (255 / 255, 128 / 255, 0 / 255),
                               'sadistic': (155 / 255, 0 / 255, 0 / 255)}
        self.sim_time_factor = 3.83
        self.recording_distance = 30

        ### Pedestrian Crossing
        self.record_delay = 4  # seconds
        self.record_time = 20  # seconds

        # ### Immobile Vehicle
        # self.record_delay = 5  # seconds
        # self.record_time = 14

        # ### Emergency Merge
        # self.record_delay = 2  # seconds
        # self.record_time = 8  # seconds

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
            print('ACTOR:',self.actors[id])
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
                        'speed': state['speed'],
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
                        'speed': state['speed'],
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

            elif type(self.actors[id]) is PedestrianAPF and 'pedestrians' in filters:
                if self.start_position == [0, 0]: continue
                rgb = self._svo_to_color(self.actors[id].get_svo())
                transparency = self._velocity_to_transparecy(self.actors[id].get_state()['speed'])
                rgba = rgb + (transparency,)
                current_time = self.world.get_snapshot().timestamp.elapsed_seconds
                state = self.actors[id].get_state()
                distance = np.linalg.norm(self.actors[id].get_relative_state()["position"])
                if distance > self.recording_distance: continue
                data = {'type': 'pedestrian',
                        'sim_time': current_time - self.start_time,
                        'x': state['position'][0] - self.start_position[0],
                        'y': state['position'][1] - self.start_position[1],
                        'speed': state['speed'],
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
                        'speed': state['speed'],
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
                        'speed': state['speed'],
                        'a_x': state['acceleration'][0],
                        'a_y': state['acceleration'][1],
                        'a_z': state['acceleration'][2],
                        'color': 'pink'}
                self.plot_data.append(data)

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

    def save_simulation_to_file(self):
        print("Saving data")
        start_index, stop_index = self.get_start_stop_index()
        initial_time = self.plot_data[start_index]['sim_time']
        ax, ay, steering, throttle, brake, time = [], [], [], [], [], []
        ego_x, ego_y = [], []
        speed = []

        all_x = [data['x'] for data in self.plot_data[start_index:stop_index]]
        all_y = [data['y'] for data in self.plot_data[start_index:stop_index]]
        colors = [data['color'] for data in self.plot_data[start_index:stop_index]]
        labels = [data['svo'] for data in self.plot_data[start_index:stop_index]]


        for data in self.plot_data[start_index:stop_index]:
            if data['type'] == 'ego_vehicle':
                ax.append(data['a_x'])
                ay.append(data['a_y'])
                steering.append(data['steering'] * 100)
                throttle.append(data['throttle'] * 100)
                brake.append(data['brake'] * 100)
                time.append(data['sim_time']-initial_time)
                ego_x.append(data['x'])
                ego_y.append(data['y'])
                speed.append(data['speed'])

        parent_folder_path = r"C:\Users\victor\Desktop\SVO Comparison"
        folder_path = os.path.join(parent_folder_path, f"{self.svo_estimation_type}")
        os.makedirs(folder_path, exist_ok=True)

        filename = os.path.join(folder_path, "sim_all_positions.txt")
        with open(filename, "w") as file:
            for x, y in zip(all_x, all_y):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_colors.txt")
        with open(filename, "w") as file:
            for x in zip(colors):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_svo_labels.txt")
        with open(filename, "w") as file:
            for x in zip(labels):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_speed.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, speed):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_longitudinal_accel.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, ax):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_lateral_accel.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, ay):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_throttle.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, throttle):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_brake.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, brake):
                file.write(f"{x} {y}\n")

        filename = os.path.join(folder_path, "sim_steering.txt")
        with open(filename, "w") as file:
            for x, y in zip(time, steering):
                file.write(f"{x} {y}\n")

    def extract_file(self, filename):
        if os.path.exists(filename):
            temp_x_data = []
            temp_y_data = []
            pattern = r'[-+]?\d*\.\d+|\d+'
            with open(filename, "r") as file:
                for line in file:
                    # x, y = map(float, line.split())
                    # temp_x_data.append(x)
                    # temp_y_data.append(y)

                    cleaned_line = [re.findall(pattern, line.split()[0])[0], line.split()[1]]
                    x, y = map(float, cleaned_line)
                    temp_x_data.append(x)
                    temp_y_data.append(y)



            return temp_x_data, temp_y_data




