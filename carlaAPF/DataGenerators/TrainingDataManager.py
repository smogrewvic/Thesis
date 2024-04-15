import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

from ApfObjects import PotentialField as pf
from PathPlanners.GradientDescentPathPlanner import Gradient_path_planner

from VehicleControllers.SteeringControlPID import Steering_Control_PID
from VehicleControllers.ThrottleControlPID import Throttle_Control_PID

from SVO.ActorBehaviorAnalysers.Pedestrian_Behavior_Analyser import Pedestrian_Behavior_Analyser
from SVO.ActorBehaviorAnalysers.Vehicle_Behavior_Analyser import Vehicle_Behavior_Analyser

import random

import json
import os


class Training_Data_Manager:

    # def __init__(self):
    #     self.client = carla.Client('localhost', 2000)
    #     self.world = self.client.get_world()
    #     self.potential_field = pf.APF()
    #
    # def generate_navpoints(self):
    #     origin = None
    #     destination = None
    #
    #     potential_field = pf.APF()
    #     high_level_route = GlobalRoutePlanner(self.world.get_map(), 1)
    #     ego_vehicle = None
    #
    #     spawn_points = self.world.get_map().get_spawn_points()
    #
    #     # find current ego_vehicle location and find random point to navigate to
    #     self.navpoint_transforms = []
    #     while origin == destination:
    #         origin = random.choice(spawn_points)
    #         destination = random.choice(spawn_points)
    #
    #     #TODO: Check for minimum distance?
    #     for waypoint in high_level_route.trace_route(origin, destination):
    #         self.navpoint_transforms.append(waypoint[0].transform)
    #
    #     potential_field.set_navpoints(self.navpoint_transforms)
    #     path_planner = Gradient_path_planner(potential_field.get_potential_field())
    #
    #
    # def generate_ego_vehicle_position(self):
    #     spawn_navpoint = random.choice(self.navpoint_transforms)
    #
    #     spawn_variance = random.gauss(0, 0.3)
    #     ego_x = spawn_navpoint.position.x
    #     ego_y = spawn_navpoint.position.y + spawn_variance

    def save_training_data(self, local_actor_ids, local_napoints, lane_coeffs, planned_path):

        print(
              # "ACTOR_IDS\n",
              # actors_ids,
              # "\n\n",
              "LOCAL_ACTOR_IDS\n",
              local_actor_ids,
              "\n\n",
              "local_napoints\n",
              local_napoints,
              "\n\n",
              "lane_coeffs\n",
              lane_coeffs,
              "\n\n",
              "planned_path",
              planned_path,
              "\n\n________________________________\n\n"
              )
