from SVO.FuzzyEstimators.Vehicle_SVO_Fuzzy import Vehicle_SVO_Fuzzy
from SVO.FuzzyEstimators.Pedestrian_SVO_Fuzzy import Pedestrian_SVO_Fuzzy
from SVO.FuzzyEstimators.Vehicle_SVO_T2Fuzzy import Vehicle_SVO_T2Fuzzy
from SVO.FuzzyEstimators.Pedestrian_SVO_T2Fuzzy import Pedestrian_SVO_T2Fuzzy

import time
import matplotlib.pyplot as plt
import numpy as np
import random



def type1_loop(n):
    pedestrian_input = {'distance_to_crosswalk':10,'time_looking':3, 'time_waiting':2}
    vehicle_input = {'follow_time': 3, 'lane_changes': 0,'lane_centering': 0.2 ,'speed_limit_percent':100}

    vehicle_estimator = Vehicle_SVO_Fuzzy()
    # pedestrian_estimator = Pedestrian_SVO_Fuzzy()


    start_time = time.time()*1000
    for _ in range(n):
        vehicle_estimator.calculate_output(vehicle_input)
        # pedestrian_estimator.calculate_output(pedestrian_input)
    end_time = time.time()*1000
    return end_time - start_time

def type2_loop(n):
    pedestrian_input = {'distance_to_crosswalk':10,'time_looking':3, 'time_waiting':2}
    vehicle_input = {'follow_time': 3, 'lane_changes': 0,'lane_centering': 0.2 ,'speed_limit_percent':100}

    vehicle_estimator = Vehicle_SVO_T2Fuzzy()
    # pedestrian_estimator = Pedestrian_SVO_T2Fuzzy()
    start_time = time.time()*1000
    for _ in range(n):
        vehicle_estimator.calculate_output(vehicle_input)
        # pedestrian_estimator.calculate_output(pedestrian_input)

    end_time = time.time()*1000
    return end_time - start_time

if __name__ == "__main__":
    loop_iterations = 1000
    num_runs = 50  # Number of times to run each method

    # Perform timings for the first method
    method1_times = [type1_loop(loop_iterations)/loop_iterations for _ in range(num_runs)]

    # Perform timings for the second method
    method2_times = [type2_loop(loop_iterations)/loop_iterations for _ in range(num_runs)]

    # Plotting the grouped bar chart
    labels = ['Type-1', 'Type-2']
    method1_mean = np.mean(method1_times)
    method2_mean = np.mean(method2_times)
    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width/2, [method1_mean, method2_mean], width, label='Mean Time (ms)')
    rects2 = ax.bar(x + width/2, [np.std(method1_times), np.std(method2_times)], width, label='Standard Deviation')

    ax.set_ylabel('Time (ms)', fontsize = 18)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=18)
    ax.legend()
    ax.tick_params(axis='y', labelsize=14)
    plt.rc('font', size=14)

    def autolabel(rects):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(round(height, 3)),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')

    autolabel(rects1)
    autolabel(rects2)

    fig.tight_layout()

    plt.show()