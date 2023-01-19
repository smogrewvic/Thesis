import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import enum
import sys
from scipy.integrate import odeint
import skfuzzy as fuzz
import skfuzzy.control as ctrl
from mpl_toolkits.mplot3d import Axes3D

import random
from deap import base
from deap import creator
from deap import tools
import multiprocessing
from threading import Thread

import Vehicle
import Obstacle
import JEEP
import warnings


class NavigationController:
    def __init__(self, vehicle, obstacles, target):
        self.vehicle = vehicle
        self.obstacles = obstacles
        self.target = target
        self.fuzzyInputs = [[0] * 2 for _ in range(len(obstacles))]
        self.obstacleForceVectors = [0] * 2
        self.targetForceVectors = 0

        # testvars
        # self.lastHeading = 0

    def getVehicleData(self):
        return self.vehicle

    def updateTarget(self):
        # todo: acquire new target information
        # self.target = target
        pass

    def updateObstacles(self):
        # todo: acquire new obstacles from sensors
        # self.obstacles = obstacles
        pass

    def calculateAngles(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        self.targetForceVectors = targetVector * 1  # Todo: add a weighting factor, maybe multiple targets?

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]


            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))

            self.fuzzyInputs[i][1] = abs(angleObstacleTarget)
            self.obstacleForceVectors[i] = obstacleVector * 1  # Todo: add a weighting factor
            # print("angleObstacleTarget",self.fuzzyInputs[i][1])

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))
            self.fuzzyInputs[i][0] = distanceRatio
            # print("distanceRatio", distanceRatio)

        return distanceRatio

    def saturateValue(self, value, saturationHigh, saturationLow):

        if value > saturationHigh:
            value = saturationHigh
        elif value < saturationLow:
            value = saturationLow

        return value

    def navigate(self):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        # Create fuzzy variables
        relativeAngle = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        distanceRatio = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        output = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')

        # print("arange", len(np.arange(0, np.pi/2, 0.01)))
        # Create fuzzy membership functions
        relativeAngle['low'] = fuzz.zmf(relativeAngle.universe, np.pi / 4, np.pi / 2)
        relativeAngle['high'] = fuzz.smf(relativeAngle.universe, 0.2, np.pi / 2)
        distanceRatio['low'] = fuzz.zmf(distanceRatio.universe, 0.25, 2)  # alpha
        distanceRatio['high'] = fuzz.smf(distanceRatio.universe, 0, 1.5)  # alpha

        output['low'] = fuzz.zmf(output.universe, 0.25, 1)
        output['high'] = fuzz.smf(output.universe, 0, 0.75)

        # Create fuzzy rules
        rule1 = ctrl.Rule(distanceRatio['low'] & relativeAngle['low'], output['high'])
        rule2 = ctrl.Rule(relativeAngle['high'], output['low'])

        # Create fuzzy control system
        steering_control = ctrl.ControlSystem([rule1, rule2])
        forceWeight = ctrl.ControlSystemSimulation(steering_control)

        # Set inputs and compute output
        forceWeight.input['distanceRatio'] = self.fuzzyInputs[0][0]
        forceWeight.input['relativeAngle'] = self.fuzzyInputs[0][1]
        forceWeight.compute()

        repulsionVector = forceWeight.output['output'] * np.array(self.obstacleForceVectors[0])
        attractionVector = (1 - forceWeight.output['output']) * np.array(self.targetForceVectors[0])

        resultForceVector = np.add(repulsionVector, attractionVector)
        steeringAngle = math.atan2(resultForceVector[1], resultForceVector[0])  # - self.vehicle.getHeading()

        # rate limiting steering
        # heading = self.lastHeading + min(max(steeringAngle-self.lastHeading, np.pi/4), -np.pi/4)
        # self.lastHeading = heading
        # self.vehicle.setHeading(heading)

        # saturate steering angle to range
        steeringAngle = self.saturateValue(steeringAngle, self.vehicle.carModel.maxTireAngleRads.value, -self.vehicle.carModel.maxTireAngleRads.value)
        self.vehicle.setTireAngle(steeringAngle, degrees=False)

        # self.displayPlots([relativeAngle, distanceRatio], [output], [rule1, rule2])

        # return resultForceVector

    def displayPlots(self, inputMemberships, outputMemberships, rules):

        fis = ctrl.ControlSystem(rules)
        controlSystem = ctrl.ControlSystemSimulation(fis)

        for memFunc in inputMemberships:
            memFunc.view()

        for memFunc in outputMemberships:
            memFunc.view()

        # create 3D plot space
        upsampled = np.arange(0, 2, 0.1)
        x, y = np.meshgrid(upsampled, upsampled)
        z = np.zeros_like(x)
        for i in range(len(upsampled)):
            for j in range(len(upsampled)):
                # print(i, j, "angle", x[i, j], "\tdistanceRatio", y[i, j])
                controlSystem.input[inputMemberships[0].label] = x[i, j]
                controlSystem.input[inputMemberships[1].label] = y[i, j]
                controlSystem.compute()
                z[i, j] = controlSystem.output[outputMemberships[0].label]

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                               linewidth=0.4, antialiased=True)

        cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
        cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
        cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

        ax.view_init(30, 200)
        plt.show()

