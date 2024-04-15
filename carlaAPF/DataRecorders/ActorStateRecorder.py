import carla
from ApfObjects.PedestrianAPF import PedestrianAPF
from ApfObjects.VehicleAPF import VehicleAPF
from ApfObjects.NavpointAPF import NavpointAPF
from ApfObjects.RegressionLaneAPF import Regression_Lane_APF
from ApfObjects.TrafficLightAPF import TrafficLightAPF

import matplotlib.pyplot as plt
class Actor_State_Recorder:
    def __init__(self, pf_actors):
        self.sim_time_factor = 0
        self.actor_states_memory = {}
        self.actors = pf_actors
        self.plot_data = []

    def calculate_time_factor(self):
        pass

    def record_data(self, filters = ['vehicles'], traces = True):

        data = {'x':None, 'y':None, 'color':None}
        for id in self.actors:
            if id == "ego_vehicle" and 'ego_vehicle' in filters:
                data = {'x': self.actors[id].get_state()["position"][0],
                        'y': self.actors[id].get_state()["position"][1],
                        'color': "blue"}
            elif type(self.actors[id]) is VehicleAPF and 'vehicles' in filters:
                data = {'x': self.actors[id].get_state()["position"][0],
                        'y': self.actors[id].get_state()["position"][1],
                        'color': "red"}
            elif type(self.actors[id]) is NavpointAPF and 'navpoints' in filters:
                data = {'x': self.actors[id].get_state()["position"][0],
                        'y': self.actors[id].get_state()["position"][1],
                        'color': "pink"}
            elif type(self.actors[id]) is TrafficLightAPF and 'traffic_lights' in filters:
                data = {'x': self.actors[id].get_state()["position"][0],
                        'y': self.actors[id].get_state()["position"][1],
                        'color': "pink"}
            self.plot_data.append(data)


        # return plot_data

    def plot_data(self):
        plt.cla()

        for data in self.plot_data:
            plt.scatter(data['x'], data['y'], c=data['color'])


        plt.xlim(150, -150)
        plt.ylim(-150, 150)
        plt.draw()
        plt.pause(0.01)
    def plot_accelerations(self, filter=None):
        pass

    def plot_comparison(self,filter):
        pass

