import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl

class NavigationController:
    def __init__(self, vehicle, obstacles, target):
        self.vehicle = vehicle
        self.obstacles = obstacles
        self.target = target

        self.fuzzyInputs = {"relative angle": [0]*len(obstacles), "distance ratio": [0]*len(obstacles)}
        self.forceVectors = {"attraction": 0, "repulsion": [0]*len(obstacles)}


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

        if np.linalg.norm(targetVector) != 0:
            self.forceVectors["attraction"] = targetVector / np.linalg.norm(targetVector)
        else:
            self.forceVectors["attraction"] = targetVector / np.linalg.norm(targetVector)

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            # angleObstacleTarget = np.arccos(round(np.dot(obstacleVector, targetVector),5) / round((np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)),5)) # floating point error mitigation
            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))

            self.fuzzyInputs["relative angle"][i] = abs(angleObstacleTarget)
            self.forceVectors["repulsion"][i] = obstacleVector / np.linalg.norm(obstacleVector)

            return angleObstacleTarget

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))

            self.fuzzyInputs["distance ratio"][i] = distanceRatio

        return distanceRatio

    def saturateValue(self, value, saturationHigh, saturationLow):

        if value > saturationHigh:
            value = saturationHigh
        elif value < saturationLow:
            value = saturationLow

        return value

    def calculateSteeringInput(self, headingRequest):

        currentHeading = self.vehicle.getHeading()

        netAngle = headingRequest - currentHeading  # positive CCW
        if netAngle > np.pi:
            netAngle = netAngle - 2*np.pi   # convert from 360degs to left/right

        maxAngle = self.vehicle.carModel.maxTireAngleRads.value
        netAngle = self.saturateValue(netAngle, maxAngle, -maxAngle)

        return netAngle


    def navigate(self):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        weight = 0
        for i in range(len(self.obstacles)):
            weight += self.obstacles[i].computeFuzzy([self.fuzzyInputs["relative angle"][i], self.fuzzyInputs["distance ratio"][i]]) / len(self.obstacles)


        repulsionVector = -1 * weight * np.array(self.forceVectors["repulsion"][0])
        attractionVector = (1 - weight) * np.array(self.forceVectors["attraction"])

        resultForceVector = np.add(repulsionVector, attractionVector)
        resultForceAngle = math.atan2(resultForceVector[1], resultForceVector[0])

        # using tire angle to steer
        tireAngle = self.calculateSteeringInput(resultForceAngle)
        self.vehicle.setTireAngle(tireAngle)

        return resultForceVector, repulsionVector, attractionVector

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

