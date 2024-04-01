# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import numpy as np
import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption
# from agents.navigation.behavior_types import Cautious, Aggressive, Normal
from SVO.ActorBehaviorManagers.behavior_types_extended import Cautious, Aggressive, Normal, Altruistic, Cooperative, Individualistic, Competitive, \
    Sadistic

from agents.tools.misc import get_speed, positive, is_within_distance, compute_distance

from agents.navigation.behavior_agent import BehaviorAgent


class BehaviorAgentExtended(BehaviorAgent):  # Class extended to have more behavior types
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(self, vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
        """

        super().__init__(vehicle, opt_dict=opt_dict, map_inst=map_inst, grp_inst=grp_inst)
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0
        self._speed_limit = 0
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5

        # Parameters for agent behavior
        if behavior == 'cautious':
            self._behavior = Cautious()

        elif behavior == 'normal':
            self._behavior = Normal()

        elif behavior == 'aggressive':
            self._behavior = Aggressive()

        elif behavior == 'altruistic':
            self._behavior = Altruistic()
        elif behavior == 'cooperative':
            self._behavior = Cooperative()
        elif behavior == 'individualistic':
            self._behavior = Individualistic()
        elif behavior == 'competitive':
            self._behavior = Competitive()
        elif behavior == 'sadistic':
            self._behavior = Sadistic()
