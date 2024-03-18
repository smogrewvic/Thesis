from SVO.FuzzyEstimators.Vehicle_SVO_Fuzzy import Vehicle_SVO_Fuzzy
from SVO.FuzzyEstimators.Pedestrian_SVO_Fuzzy import Pedestrian_SVO_Fuzzy
from SVO.FuzzyEstimators.Vehicle_SVO_T2Fuzzy import Vehicle_SVO_T2Fuzzy
from SVO.FuzzyEstimators.Pedestrian_SVO_T2Fuzzy import Pedestrian_SVO_T2Fuzzy

import time
import matplotlib.pyplot as plt
import numpy as np
import random


def speed_limit_comparison():
    initial_speed = 60  # % of speed limit
    acceleration = 5  # %/sec
    final_speed = 200  # % of speed limit
    t1_estimator = Vehicle_SVO_Fuzzy()
    t2_estimator = Vehicle_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'follow_time': 3, 'lane_changes': 0,'lane_centering': 0 ,'speed_limit_percent': initial_speed}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= ((final_speed - initial_speed )/ acceleration):
        input_time = time.time() - start_time
        speed_limit = input_time * acceleration + initial_speed
        input_vector['speed_limit_percent'] = speed_limit

        # print(speed_limit)
        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Speed Limit (%)', 'Social Value'

def lane_changes_comparison():
    initial_lane_changes = 0 # changes/min
    lane_change_rate = 12  # changes/min
    final_lane_changes = 6 # changes/min
    t1_estimator = Vehicle_SVO_Fuzzy()
    t2_estimator = Vehicle_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'follow_time': 3, 'lane_changes': initial_lane_changes, 'lane_centering': 0, 'speed_limit_percent':100}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= (final_lane_changes/lane_change_rate)*60:
        input_time = time.time() - start_time
        lane_changes = input_time * lane_change_rate/60 - initial_lane_changes
        input_vector['lane_changes'] = lane_changes

        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Lane Changes (Changes/min)', 'Social Value'

def lane_centering_comparison():
    merge_speed = 1  # m/s
    initial_distance = 0  # m
    final_distance = 2  # m
    t1_estimator = Vehicle_SVO_Fuzzy()
    t2_estimator = Vehicle_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'follow_time': 3, 'lane_changes': 0,'lane_centering': initial_distance ,'speed_limit_percent':100}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= (final_distance / merge_speed):
        input_time = time.time() - start_time
        distance = input_time * merge_speed - initial_distance
        input_vector['lane_centering'] = distance

        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Lane Centering (m)', 'Social Value'

def follow_time_comparison():
    closing_speed = 10  # m/s
    initial_follow_time = 10  # s
    initial_speed = 25  # m/s
    initial_distance = initial_speed*initial_follow_time
    t1_estimator = Vehicle_SVO_Fuzzy()
    t2_estimator = Vehicle_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'follow_time': initial_follow_time,  'lane_changes': 0,'lane_centering': 0 ,'speed_limit_percent':100}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0

    while input_time <= (initial_distance / closing_speed):
        input_time = time.time() - start_time
        distance = initial_distance - input_time * closing_speed
        follow_time = distance/initial_speed
        input_vector['follow_time'] = follow_time

        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Follow Time (s)', 'Social Value'

def distance_to_crosswalk_comparison():
    walking_speed = 1  # m/s
    initial_distance = 20  # m
    t1_estimator = Pedestrian_SVO_Fuzzy()
    t2_estimator = Pedestrian_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'distance_to_crosswalk': initial_distance, 'time_looking': 0, 'time_waiting': 0}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= (initial_distance/walking_speed):
        input_time = time.time() - start_time
        distance = initial_distance - input_time*walking_speed
        input_vector['distance_to_crosswalk'] = distance

        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Distance to Crosswalk (m)', 'Social Value'

def time_waiting_comparison():
    t1_estimator = Pedestrian_SVO_Fuzzy()
    t2_estimator = Pedestrian_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'distance_to_crosswalk': 2, 'time_looking': 0, 'time_waiting': 0}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= 4:
        input_time = time.time() - start_time
        input_vector['time_waiting'] = input_time

        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data, 'Time Waiting (s)', 'Social Value'

def time_looking_comparison():
    t1_estimator = Pedestrian_SVO_Fuzzy()
    t2_estimator = Pedestrian_SVO_T2Fuzzy()

    t1_data = []
    t2_data = []

    input_vector = {'distance_to_crosswalk': 2, 'time_looking': 0, 'time_waiting': 0}

    svo_t1 = t1_estimator.calculate_output(input_vector)
    svo_t2 = t2_estimator.calculate_output(input_vector)

    t1_data.append([0, svo_t1])
    t2_data.append([0, svo_t2])

    start_time = time.time()
    input_time = 0
    while input_time <= 4:
        input_time = time.time() - start_time
        input_vector['time_looking'] = input_time
        # input_vector['time_waiting'] = input_time
        svo_t1 = t1_estimator.calculate_output(input_vector)
        svo_t2 = t2_estimator.calculate_output(input_vector)

        t1_data.append([input_time, svo_t1])
        t2_data.append([input_time, svo_t2])

    return t1_data, t2_data,  'Time Looking (s)', 'Social Value'


if __name__ == "__main__":
    t1_data, t2_data, data_x_label, data_y_label = distance_to_crosswalk_comparison()
    times1 = [item[0] for item in t1_data]
    values1 = [item[1] for item in t1_data]

    times2 = [item[0] for item in t2_data]
    values2 = [item[1] for item in t2_data]

    plt.plot(times1, values1, label='Type-1')
    plt.plot(times2, values2, label='Type-2')
    plt.xlabel(data_x_label)
    plt.ylabel(data_y_label)
    # plt.title('Plot of Measured Values over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
