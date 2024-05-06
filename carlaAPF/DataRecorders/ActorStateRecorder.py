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

        ego_x, ego_y = [], []
        for data in self.plot_data[start_index:stop_index]:
            if data['type'] == 'ego_vehicle':
                ego_x.append(data['x'])
                ego_y.append(data['y'])

        plt.scatter(y, x, c=colors)

        for i, label in enumerate(labels):
            if i % label_frequency != 0 or label == 0:
                continue

            plt.annotate(label,  # this is the text
                         (y[i], x[i]),  # these are the coordinates to position the label
                         textcoords="offset points",  # how to position the text
                         xytext=(15, 0),  # distance from text to points (x,y)
                         ha='left')  # horizontal alignment can be left, right or center

        total_distance = 0
        closest_distance = float('inf')
        for i in range(1, len(ego_x)):
            total_distance += np.sqrt((ego_x[i] - ego_x[i-1]) ** 2 + (ego_y[i] - ego_y[i-1]) ** 2)

        for i, data in enumerate(self.plot_data[start_index:stop_index]):
            if data['type'] == 'vehicle':
                closest_distance = min(closest_distance, data['relative_distance'])

        plt.annotate('Closest Distance: ' + str(round(closest_distance, 2))+'m',
                     (2.5, 35),
                     textcoords="offset points",
                     xytext=(0, 0),
                     fontsize=10,
                     weight = 'bold',
                     ha='left')

        plt.annotate('Total Distance: ' + str(round(total_distance, 2))+'m',
                     (2.5, 38),
                     textcoords="offset points",
                     xytext=(0, 0),
                     fontsize=10,
                     weight='bold',
                     ha='left')

        plt.annotate('Ego-Vehicle',
                     (y[0],x[0]),
                     textcoords="offset points",
                     xytext=(0, -15),
                     fontsize=10,
                     weight='bold',
                     ha='center')

        plt.annotate('Actor',
                     (y[1],x[1]),
                     textcoords="offset points",
                     xytext=(0, -15),
                     fontsize=10,
                     weight='bold',
                     ha='center')

        plt.ylabel('Longitudinal Position (m)', fontsize=14, weight='bold')  # Set ylabel font size
        plt.xlabel('Lateral Position (m)', fontsize=14, weight='bold')  # Set xlabel font size
        plt.xlim(-1.5, 5)
        plt.ylim(-5)

        file_name = '\\Positions ' + self.svo_estimation_type + '.png'
        folder_path = r'C:\Users\victor\Desktop\SVO Results'
        plt.savefig(folder_path + file_name, bbox_inches='tight')

        plt.show()
    def plot_accelerations(self):
        print('Plotting Actor Accelerations')

        start_index, stop_index = self.get_start_stop_index()
        initial_time = self.plot_data[start_index]['sim_time']
        x_ax_raw, x_ay_raw, steering, throttle, brake, time = [], [], [], [], [], []

        for data in self.plot_data[start_index:stop_index]:
            if data['type'] == 'ego_vehicle':
                x_ax_raw.append(data['a_x'])
                x_ay_raw.append(data['a_y'])
                steering.append(data['steering'] * 100)
                throttle.append(data['throttle'] * 100)
                brake.append(data['brake'] * 100)
                time.append(data['sim_time']-initial_time)

        x_ax = self.low_pass_filter(x_ax_raw, alpha=0.5)
        x_ay = self.low_pass_filter(x_ay_raw, alpha=0.5)

        fig, axs = plt.subplots(4, 1, figsize=(17, 15))
        num_ticks = 5
        # Plot for subplot 1
        axs[0].plot(time, x_ax, color='blue')
        axs[0].set_title('Longitudinal Acceleration (x)', fontsize=18, weight='bold')
        axs[0].set_ylabel('Accel. (m/s$^2$)', fontsize=14, weight='bold')
        axs[0].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[0].tick_params(axis='x', labelsize=14)
        axs[0].tick_params(axis='y', labelsize=14)
        # axs[0].set_yticks([])
        # axs[0].set_ylim()
        axs[0].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        # Plot for subplot 2
        axs[1].plot(time, throttle, color='green')  # [0,1]
        axs[1].plot(time, brake, color='red')  # [0,1]
        axs[1].set_title('Throttle & Brake Inputs', fontsize=18, weight='bold')
        axs[1].set_ylabel('Pedal Position (%)', fontsize=14, weight='bold')
        axs[1].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[1].tick_params(axis='x', labelsize=14)
        axs[1].tick_params(axis='y', labelsize=14)
        axs[1].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        axs[2].plot(time, x_ay, color='blue')  # acceleration
        axs[2].set_title('Lateral Acceleration (y)', fontsize=18, weight='bold')
        axs[2].set_ylabel('Accel.   (m/s$^2$)', fontsize=14, weight='bold')
        axs[2].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[2].tick_params(axis='x', labelsize=14)
        axs[2].tick_params(axis='y', labelsize=14)
        axs[2].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))


        axs[3].plot(time, steering, color='green')  # acceleration
        axs[3].set_title('Steering Input', fontsize=18, weight='bold')
        axs[3].set_ylabel('Steering Input (%)', fontsize=14, weight='bold')
        axs[3].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[3].tick_params(axis='x', labelsize=14)
        axs[3].tick_params(axis='y', labelsize=14)
        axs[3].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        plt.subplots_adjust(hspace=1.0)

        rms, wrms = self.rms_acceleration() , self.weighted_rms_acceleration()
        peak_ax, peak_ay = self.peak_accelerations()

        file_name = '\\Accelerations ' + self.svo_estimation_type + ' rms '+ str(rms)+ ' wrms '+ str(wrms) +'.png'
        folder_path = r'C:\Users\victor\Desktop\SVO Results'
        plt.savefig(folder_path + file_name, bbox_inches='tight' )
        plt.show()
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

        # plt.draw()
        # plt.pause(0.01)
        # file_name = 'Positions' + self.svo_estimation_type + '.png'
        # plt.savefig('Postions.png')


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

    def peak_accelerations(self, acc_data_x = [0], acc_data_y = [0]):
        peak_ax, peak_ay = 0, 0
        if acc_data_x == [0] and acc_data_y == [0]:
            start_index, stop_index = self.get_start_stop_index()
            for i, data in enumerate(self.plot_data[start_index:stop_index]):
                if data['type'] == 'ego_vehicle':
                    peak_ax = max(peak_ax, abs(data['a_x']))
                    peak_ay = max(peak_ay, abs(data['a_y']))

            return round(peak_ax,3), round(peak_ay,3)
        else:
            for value in acc_data_x:
                peak_ax = max(peak_ax, abs(value))
            for value in acc_data_y:
                peak_ay = max(peak_ay, abs(value))

            return round(peak_ax, 3), round(peak_ay, 3)

    def rms_acceleration(self, acc_data_x = [0], acc_data_y = [0], acc_data_z = [0]):
        return self.weighted_rms_acceleration(1,1,1, acc_data_x, acc_data_y, acc_data_z)
    def weighted_rms_acceleration(self, k_x=1.4, k_y=1.4, k_z=1, acc_data_x = [0], acc_data_y = [0], acc_data_z = [0]):

        a_x, a_y, a_z = [], [], []
        if acc_data_x == [0] and acc_data_y == [0] and acc_data_z == [0]:
            start_index, stop_index = self.get_start_stop_index()
            for i, data in enumerate(self.plot_data[start_index:stop_index]):
                if data['type'] == 'ego_vehicle':
                    a_x.append(data['a_x'])
                    a_y.append(data['a_y'])
                    a_z.append(data['a_z'])
            rms_w = np.sqrt((k_x * np.mean(a_x)) ** 2 + (k_y * np.mean(a_y)) ** 2 + (k_z * np.mean(a_z)) ** 2)

            return round(rms_w,3)

        else:
            rms_w = np.sqrt((k_x * np.mean(acc_data_x)) ** 2 + (k_y * np.mean(acc_data_y)) ** 2 + (k_z * np.mean(acc_data_z)) ** 2)

            return round(rms_w, 3)


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
            with open(filename, "r") as file:
                for line in file:
                    x, y = map(float, line.split())
                    temp_x_data.append(x)
                    temp_y_data.append(y)


            return temp_x_data, temp_y_data

    def plot_comparison(self):
        filenames = [f"sim_longitudinal_accel.txt",
                     f"sim_lateral_accel.txt",
                     f"sim_throttle.txt",
                     f"sim_brake.txt",
                     f"sim_steering.txt",
                     "sim_speed.txt"]

        folders = [r"C:\Users\victor\Desktop\SVO Comparison\none",
                   r"C:\Users\victor\Desktop\SVO Comparison\type_1",
                   r"C:\Users\victor\Desktop\SVO Comparison\type_2"]


        filename = os.path.join(folders[0], filenames[0])
        time_none, ax_none = self.extract_file(filename)
        filename = os.path.join(folders[0], filenames[1])
        time_none, ay_none = self.extract_file(filename)
        filename = os.path.join(folders[0], filenames[2])
        time_none, throttle_none = self.extract_file(filename)
        filename = os.path.join(folders[0], filenames[3])
        time_none, brake_none = self.extract_file(filename)
        filename = os.path.join(folders[0], filenames[4])
        time_none, steering_none = self.extract_file(filename)
        filename = os.path.join(folders[0], filenames[5])
        time_none, speed_none = self.extract_file(filename)

        filename = os.path.join(folders[1], filenames[0])
        time_t1, ax_t1 = self.extract_file(filename)
        filename = os.path.join(folders[1], filenames[1])
        time_t1, ay_t1 = self.extract_file(filename)
        filename = os.path.join(folders[1], filenames[2])
        time_t1, throttle_t1 = self.extract_file(filename)
        filename = os.path.join(folders[1], filenames[3])
        time_t1, brake_t1 = self.extract_file(filename)
        filename = os.path.join(folders[1], filenames[4])
        time_t1, steering_t1 = self.extract_file(filename)
        filename = os.path.join(folders[1], filenames[5])
        time_t1, speed_t1 = self.extract_file(filename)

        filename = os.path.join(folders[2], filenames[0])
        time_t2, ax_t2 = self.extract_file(filename)
        filename = os.path.join(folders[2], filenames[1])
        time_t2, ay_t2 = self.extract_file(filename)
        filename = os.path.join(folders[2], filenames[2])
        time_t2, throttle_t2 = self.extract_file(filename)
        filename = os.path.join(folders[2], filenames[3])
        time_t2, brake_t2 = self.extract_file(filename)
        filename = os.path.join(folders[2], filenames[4])
        time_t2, steering_t2 = self.extract_file(filename)
        filename = os.path.join(folders[2], filenames[5])
        time_t2, speed_t2 = self.extract_file(filename)

        ax_none = self.low_pass_filter(ax_none, alpha=0.5)
        ay_none = self.low_pass_filter(ay_none, alpha=0.5)

        ax_t1 = self.low_pass_filter(ax_t1, alpha=0.5)
        ay_t1 = self.low_pass_filter(ay_t1, alpha=0.5)

        ax_t2 = self.low_pass_filter(ax_t2, alpha=0.5)
        ay_t2 = self.low_pass_filter(ay_t2, alpha=0.5)

        fig, axs = plt.subplots(6, 1, figsize=(15, 17))
        num_ticks = 5

        # Plot for subplot 1
        axs[0].plot(time_none, ax_none, color=(1, 0.5, 0, 0.4), linestyle = 'dotted', label='None')
        axs[0].plot(time_t1, ax_t1, color=(1, 0.5, 0, 1), linestyle = 'dashed', label='Type-1')
        axs[0].plot(time_t2, ax_t2, color=(1, 0.5, 0, 1), label='Type-2' )
        axs[0].set_title('Longitudinal Acceleration (x)', fontsize=18, weight='bold')
        axs[0].set_ylabel('Accel. (m/s$^2$)', fontsize=14, weight='bold')
        axs[0].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[0].tick_params(axis='x', labelsize=14)
        axs[0].tick_params(axis='y', labelsize=14)
        axs[0].legend(loc='upper right', fontsize=12)
        axs[0].set_xlim(0,8)
        axs[0].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        # Plot for subplot 2
        axs[1].plot(time_none, throttle_none, color=(0, 0.5, 0, 0.4), linestyle = 'dotted', label='None')
        axs[1].plot(time_t1, throttle_t1, color=(0, 0.5, 0, 1), linestyle = 'dashed', label='Type-1')
        axs[1].plot(time_t2, throttle_t2, color=(0, 0.5, 0, 1), label='Type-2')
        axs[1].set_title('Throttle Input', fontsize=18, weight='bold')
        axs[1].set_ylabel('Pedal Position (%)', fontsize=14, weight='bold')
        axs[1].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[1].tick_params(axis='x', labelsize=14)
        axs[1].tick_params(axis='y', labelsize=14)
        axs[1].legend(loc='upper right', fontsize=12)
        axs[1].set_xlim(0, 8)
        axs[1].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        axs[2].plot(time_none, brake_none, color=(1, 0, 0, 0.4), linestyle = 'dotted', label='None')
        axs[2].plot(time_t1, brake_t1, color=(1, 0, 0, 1), linestyle = 'dashed', label='Type-1')
        axs[2].plot(time_t2, brake_t2, color=(1, 0, 0, 1) , label='Type-2')
        axs[2].set_title('Brake Input', fontsize=18, weight='bold')
        axs[2].set_ylabel('Pedal Position (%)', fontsize=14, weight='bold')
        axs[2].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[2].tick_params(axis='x', labelsize=14)
        axs[2].tick_params(axis='y', labelsize=14)
        axs[2].legend(loc='upper right', fontsize=12)
        axs[2].set_xlim(0, 8)
        axs[2].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        axs[3].plot(time_none, ay_none, color=(0, 0, 1, 0.4), linestyle = 'dotted', label='None')
        axs[3].plot(time_t1, ay_t1, color=(0, 0, 1, 1), linestyle = 'dashed', label='Type-1')
        axs[3].plot(time_t2, ay_t2, color=(0, 0, 1, 1) , label='Type-2')
        axs[3].set_title('Lateral Acceleration (y)', fontsize=18, weight='bold')
        axs[3].set_ylabel('Accel.   (m/s$^2$)', fontsize=14, weight='bold')
        axs[3].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[3].tick_params(axis='x', labelsize=14)
        axs[3].tick_params(axis='y', labelsize=14)
        axs[3].legend(loc='upper right', fontsize=12)
        axs[3].set_xlim(0, 8)
        axs[3].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        axs[4].plot(time_none, steering_none, color=(0.75, 0, 0.5, 0.4), linestyle = 'dotted', label='None')
        axs[4].plot(time_t1, steering_t1, color=(0.75, 0, 0.5, 1), linestyle = 'dashed', label='Type-1')
        axs[4].plot(time_t2, steering_t2, color=(0.75, 0, 0.5, 1) , label='Type-2')
        axs[4].set_title('Steering Input', fontsize=18, weight='bold')
        axs[4].set_ylabel('Steering Input (%)', fontsize=14, weight='bold')
        axs[4].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[4].tick_params(axis='x', labelsize=14)
        axs[4].tick_params(axis='y', labelsize=14)
        axs[4].legend(loc='upper right', fontsize=12)
        axs[4].set_xlim(0, 8)
        axs[4].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        axs[5].plot(time_none, speed_none, color=(1, 0.25, 0.5, 0.4), linestyle = 'dotted', label='None')
        axs[5].plot(time_t1, speed_t1, color=(1, 0.25, 0.5, 1), linestyle = 'dashed', label='Type-1')
        axs[5].plot(time_t2, speed_t2, color=(1, 0.25, 0.5, 1) , label='Type-2')
        axs[5].set_title('Velocity', fontsize=18, weight='bold')
        axs[5].set_ylabel('Velocity (m/s)', fontsize=14, weight='bold')
        axs[5].set_xlabel('Time (s)', fontsize=14, weight='bold')
        axs[5].tick_params(axis='x', labelsize=14)
        axs[5].tick_params(axis='y', labelsize=14)
        axs[5].legend(loc='upper right', fontsize=12)
        axs[5].set_xlim(0, 8)
        axs[5].yaxis.set_major_locator(plt.MaxNLocator(num_ticks))

        plt.subplots_adjust(hspace=1.0)

        rms_none = self.rms_acceleration(acc_data_x=ax_none,acc_data_y=ay_none)
        wrms_none = self.weighted_rms_acceleration(acc_data_x=ax_none,acc_data_y=ay_none)
        peak_ax_none, peak_ay_none = self.peak_accelerations(acc_data_x=ax_none,acc_data_y=ay_none)

        rms_t1 = self.rms_acceleration(acc_data_x=ax_t1,acc_data_y=ay_t1)
        wrms_t1 = self.weighted_rms_acceleration(acc_data_x=ax_t1,acc_data_y=ay_t1)
        peak_ax_t1, peak_ay_t1 = self.peak_accelerations(acc_data_x=ax_t1,acc_data_y=ay_t1)

        rms_t2 = self.rms_acceleration(acc_data_x=ax_t2,acc_data_y=ay_t2)
        wrms_t2 = self.weighted_rms_acceleration(acc_data_x=ax_t2,acc_data_y=ay_t2)
        peak_ax_t2, peak_ay_t2 = self.peak_accelerations(acc_data_x=ax_t2,acc_data_y=ay_t2)

        print('\nNO ESTIMATION\n RMS: ',rms_none, '\tWRMS: ',wrms_none, '\tPEAK AX: ',peak_ax_none, '\tPEAK AY: ',peak_ay_none)
        print('\nTYPE-1 ESTIMATION\n RMS: ', rms_t1, '\tWRMS: ', wrms_t1, '\tPEAK AX: ', peak_ax_t1, '\tPEAK AY: ', peak_ay_t1)
        print('\nTYPE-2 ESTIMATION\n RMS: ', rms_t2, '\tWRMS: ', wrms_t2, '\tPEAK AX: ', peak_ax_t2, '\tPEAK AY: ', peak_ay_t2)

        file_name1 = '\\Accelerations Comparison.png'
        file_name2 = '\\Accelerations Comparison.svg'
        folder_path = r'C:\Users\victor\Desktop\SVO Comparison'
        plt.savefig(folder_path + file_name1, bbox_inches='tight' )
        plt.savefig(folder_path + file_name2, bbox_inches='tight')
        plt.show()



