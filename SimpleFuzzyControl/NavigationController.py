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
        self.fuzzyInputs = [[0] * 2 for _ in range(len(obstacles))]
        self.obstacleForceVectors = [0]*len(obstacles)
        self.targetForceVectors = [0]

        self.inputMF1 = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        self.inputMF2 = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        self.outputMF1 = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')
        self.rule1 = None
        self.rule2 = None
        self.controlSystem = None
        self.fisSimulation = None

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
            self.targetForceVectors = targetVector/ np.linalg.norm(targetVector)
        else:
            self.targetForceVectors = targetVector / np.linalg.norm(targetVector)

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            # if np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector) != 0:
            #     angleObstacleTarget = np.arccos(round(np.dot(obstacleVector, targetVector),5) / round((np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)),5))

            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))

            # if np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector) !=0:
            #     angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))
            # else:
            #     angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector))

            self.fuzzyInputs[i][1] = abs(angleObstacleTarget)
            self.obstacleForceVectors[i] = obstacleVector / np.linalg.norm(obstacleVector)

            return angleObstacleTarget

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            # if np.linalg.norm(targetVector) == 0 or np.linalg.norm(obstacleVector) == 0:
            #     print("zero norm", np.linalg.norm(targetVector), np.linalg.norm(obstacleVector))
            #     distanceRatio = np.linalg.norm(obstacleVector) / 0.0001
            # else:
            #     distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))

            self.fuzzyInputs[i][0] = distanceRatio

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

    def setMemberships(self, mfParams, mfShape="sigmoid", display=False):

        if mfShape == "gauss":
            for i in range(len(mfParams)):  # check that no mfParams became zero from mutation
                if mfParams[i] <= 0:
                    mfParams[i] = 0.1
                    # return False
            self.inputMF1['low'] = fuzz.gaussmf(self.inputMF1.universe, mfParams[0], mfParams[1])
            self.inputMF1['high'] = fuzz.gaussmf(self.inputMF1.universe, mfParams[2], mfParams[3])
            self.inputMF2['low'] = fuzz.gaussmf(self.inputMF2.universe, mfParams[4], mfParams[5])  # alpha
            self.inputMF2['high'] = fuzz.gaussmf(self.inputMF2.universe, mfParams[6], mfParams[7])  # alpha

            self.outputMF1['low'] = fuzz.gaussmf(self.outputMF1.universe, mfParams[8], mfParams[9])
            self.outputMF1['high'] = fuzz.gaussmf(self.outputMF1.universe, mfParams[10], mfParams[11])

        if mfShape == "sigmoid":
            self.inputMF1['low'] = fuzz.sigmf(self.inputMF1.universe, mfParams[0], mfParams[1] * 20)
            self.inputMF1['high'] = fuzz.sigmf(self.inputMF1.universe, mfParams[2], mfParams[3] * 20)
            self.inputMF2['low'] = fuzz.sigmf(self.inputMF2.universe, mfParams[4], mfParams[5] * 20)  # alpha
            self.inputMF2['high'] = fuzz.sigmf(self.inputMF2.universe, mfParams[6], mfParams[7] * 20)  # alpha

            self.outputMF1['low'] = fuzz.sigmf(self.outputMF1.universe, mfParams[8], mfParams[9] * 20)
            self.outputMF1['high'] = fuzz.sigmf(self.outputMF1.universe, mfParams[10], mfParams[11] * 20)

        # Create fuzzy rules
        self.rule1 = ctrl.Rule(self.inputMF2['low'] & self.inputMF1['low'], self.outputMF1['high'])
        self.rule2 = ctrl.Rule(self.inputMF1['high'], self.outputMF1['low'])

        # Create fuzzy control system
        self.controlSystem = ctrl.ControlSystem([self.rule1, self.rule2])
        self.fisSimulation = ctrl.ControlSystemSimulation(self.controlSystem)

    def navigate(self, display=False):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        # Set inputs and compute output
        self.fisSimulation.input['distanceRatio'] = self.fuzzyInputs[0][0]
        self.fisSimulation.input['relativeAngle'] = self.fuzzyInputs[0][1]

        try:
            self.fisSimulation.compute()
        except:
            print("crisp output not possible")
            return False

        repulsionVector = self.fisSimulation.output['output'] * np.array(self.obstacleForceVectors[0]) * -1  # todo: check if repulsion is negative
        attractionVector = (1 - self.fisSimulation.output['output']) * np.array(self.targetForceVectors)

        resultForceVector = np.add(repulsionVector, attractionVector)
        resultForceAngle = math.atan2(resultForceVector[1], resultForceVector[0])

        # using tire angle to steer
        tireAngle = self.calculateSteeringInput(resultForceAngle)
        self.vehicle.setTireAngle(tireAngle)

        if display == True:
            self.displayPlots([self.inputMF1, self.inputMF2], [self.outputMF1], [self.rule1, self.rule2])

        return resultForceVector, repulsionVector, attractionVector

    def navigate_backup(self, display = False):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        # Create fuzzy variables
        relativeAngle = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        distanceRatio = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        output = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')

        # Create fuzzy membership functions
        # relativeAngle['low'] = fuzz.sigmf(relativeAngle.universe, 0, -0.87187*20)
        # relativeAngle['high'] = fuzz.sigmf(relativeAngle.universe, 0.203203, 0)
        # distanceRatio['low'] = fuzz.sigmf(distanceRatio.universe, 0.24924, -0.549549*20)  # alpha
        # distanceRatio['high'] = fuzz.sigmf(distanceRatio.universe, -0.44344, 0.699699*20)  # alpha
        # output['low'] = fuzz.sigmf(output.universe, 0, -0.9619*20)
        # output['high'] = fuzz.sigmf(output.universe, 0.473473, 0.1891891*20)

        # [-0.35935, -0.683683, 0.90790, 0.313313, 0.639639, 0.0, -0.561561, -0.725725, -0.091091, -0.919919, 0.803803, 0.4814814]
        relativeAngle['low'] = fuzz.sigmf(relativeAngle.universe, -0.35935, -0.683683*20)
        relativeAngle['high'] = fuzz.sigmf(relativeAngle.universe, 0.90790, 0.313313)
        distanceRatio['low'] = fuzz.sigmf(distanceRatio.universe, 0.639639, 0.0*20)  # alpha
        distanceRatio['high'] = fuzz.sigmf(distanceRatio.universe, -0.561561, -0.725725*20)  # alpha
        output['low'] = fuzz.sigmf(output.universe, -0.091091, -0.919919*20)
        output['high'] = fuzz.sigmf(output.universe, 0.803803, 0.4814814*20)
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

        repulsionVector = forceWeight.output['output'] * np.array(self.obstacleForceVectors[0]) * -1  # todo: check if repulsion is negative
        attractionVector = (1 - forceWeight.output['output']) * np.array(self.targetForceVectors)

        resultForceVector = np.add(repulsionVector, attractionVector)
        resultForceAngle = math.atan2(resultForceVector[1], resultForceVector[0])

        # rate limiting steering
        # nextHeading = self.vehicle.getHeading() + min(max(resultForceAngle-self.vehicle.getHeading(), np.pi/4), -np.pi/4)
        # self.vehicle.setHeading(nextHeading)


        # using tire angle to steer
        tireAngle = self.calculateSteeringInput(resultForceAngle)
        self.vehicle.setTireAngle(tireAngle)

        #using raw heading to steer
        # self.vehicle.setHeading(resultForceAngle, degrees = False)

        # print("w:", forceWeight.output['output'], "attractionVector",attractionVector,"repulsionVector", repulsionVector)
        print("w:", round(forceWeight.output['output'],3), "relativeAngle", round(self.fuzzyInputs[0][1],3), "distanceRatio", round(self.fuzzyInputs[0][0],3))
        if display == True:
            self.displayPlots([relativeAngle, distanceRatio], [output], [rule1, rule2])

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

