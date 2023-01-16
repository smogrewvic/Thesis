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


class JEEP(enum.Enum):
    wheelbase = 2.795
    width = 2.00
    steeringRatio = 0.0652778
    maxTireAngle = 89  # degrees
    maxTireAngleRads = maxTireAngle * 0.01745329  # radians


class Vehicle:
    def __init__(self, carModel, speed=0, position=[0, 0], heading=0):
        self.speed = speed
        self.position = position
        self.heading = heading
        self.carModel = carModel
        self.tireAngle = 0
        self.positionMemory = []
        self.lateralAccelMemory = []

    def setSpeed(self, speed):
        self.speed = speed

    def getSpeed(self):
        return self.speed

    def setPosition(self, x, y):
        self.position[0] = x
        self.position[1] = y

    def getPosition(self):
        return self.position

    def setHeading(self, heading):
        self.heading = heading

    def getHeading(self):
        return self.heading

    def setTireAngle(self, tireAngle, degrees=True):

        # todo: limit max tire angle to vehicle model settings
        if degrees == True:
            self.tireAngle = tireAngle * np.pi / 180
        else:
            self.tireAngle = tireAngle

    def getTireAngle(self):
        return self.tireAngle

    def setSteeringAngle(self, steeringWheelAngle, degrees=True):
        if degrees == True:
            self.tireAngle = steeringWheelAngle * self.carModel.steeringRatio.value * np.pi / 180
        else:
            self.tireAngle = steeringWheelAngle * self.carModel.steeringRatio.value

    def getSteeringAngle(self):
        return self.tireAngle / self.carModel.steeringRatio.value

    def odes(self, x, t):
        x1 = x[0]
        y1 = x[1]
        theta1 = x[2]

        #### ODE PERFECT MODEL - WITH HITCH
        dX1dt = self.speed * np.cos(theta1)
        dY1dt = self.speed * np.sin(theta1)
        dTheta1dt = (self.speed / self.carModel.wheelbase.value) * np.tan(self.tireAngle)

        return [dX1dt, dY1dt, dTheta1dt]

    def updateState(self):

        # ODE initial conditions
        x0 = [self.position[0], self.position[1], self.heading]

        odeStepSize = 1000
        # t = np.linspace(0, 15, odeStepSize)  # time vector: t start, t end, step size
        t = [0, 0.01, 0.02]
        x = odeint(self.odes, x0, t)  # ODE calculation


        x1 = x[:, 0]
        y1 = x[:, 1]
        theta1 = x[:, 2]
        self.position, self.heading = [x1[2], y1[2]], theta1[2]  # current positions

        # log data
        self.positionMemory.append(self.position)
        # np.append(self.positionMemory, self.position, axis = 0)

        return x1, y1, theta1  # future path


class Obstacle:

    def __init__(self, speed=0, position=[0, 0], heading=0):
        self.speed = speed
        self.position = position
        self.heading = heading

    def setSpeed(self, speed):
        self.speed - speed

    def getSpeed(self):
        return self.speed

    def setPosition(self, x, y):
        self.position[0] = x
        self.position[1] = y

    def getPosition(self):
        return self.position

    def setHeading(self, heading):
        self.heading = heading

    def getHeading(self):
        return self.heading


class Car(Obstacle):
    def __init__(self, speed=0, position=[0, 0], heading=0):
        super().__init__(speed, position, heading)
        self.longitudinalSafety = 0
        self.lateralSafety = 0
        self.speedFactor = 2

    def calculateSafety(self):
        self.longitudinalSafety = self.speed * self.speedFactor
        self.lateralSafety = self.speed * self.speedFactor * 0.1


class Pedestrian(Obstacle):
    def __init__(self, speed=0, position=[0, 0], heading=0):
        super().__init__(speed, position, heading)
        self.longitudinalSafety = 0
        self.lateralSafety = 0
        self.speedFactor = 2

    def calculateSafety(self):
        self.longitudinalSafety = self.speed * self.speedFactor
        self.lateralSafety = self.speed * self.speedFactor * 0.1


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
        # todo: aquire new target information
        # self.target = target
        pass

    def updateObstacles(self):
        # todo: aquire new obstacles from sensors
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


class Simulation:
    def __init__(self, navigationObj, obstacles, target):
        self.navigation = navigationObj
        self.vehicle = navigationObj.getVehicleData()
        self.obstacles = obstacles
        self.target = target

    def updatePlot(self, i):
        resultForce = self.navigation.navigate()

        plt.cla()
        plt.scatter(self.vehicle.getPosition()[0], self.vehicle.getPosition()[1], color='blue')
        plt.scatter(self.target[0], self.target[1], color='green')

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            plt.scatter(currentObstacle.getPosition()[0], currentObstacle.getPosition()[1], color='red')

        # plot force Vector
        # forceX = [self.vehicle.getPosition()[0], self.vehicle.getPosition()[0]+resultForce[0]]
        # forceY = [self.vehicle.getPosition()[1], self.vehicle.getPosition()[1]+resultForce[1]]
        # plt.plot(forceX, forceY)

        plt.xlim(-25, 25)
        plt.ylim(-25, 25)
        # time.sleep(5)

    def animate(self):
        plt.axis('off')
        ani = FuncAnimation(plt.gcf(), self.updatePlot, interval=10)
        plt.show()


class LearnController(NavigationController):
    def __init__(self):
        self.vehicle = Vehicle(JEEP, speed=10)
        self.obstacle = [Obstacle(position=[5, 0])]
        self.target = [10, 0]
        super().__init__(self.vehicle, self.obstacle, self.target)

        self.inputMF1 = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        self.inputMF2 = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        self.outputMF1 = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')


    def navigate2(self, gaussParams, mfShape = "gauss", display=False):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()
        # print("PARAMS", gaussParams)
        if mfShape == "gauss":
            #check that no gaussParams became zero from mutation
            for i in range(len(gaussParams)):
                if gaussParams[i] <= 0:

                    gaussParams[i]=0.1
                    # return False
            self.inputMF1['low'] = fuzz.gaussmf(self.inputMF1.universe, gaussParams[0], gaussParams[1])
            self.inputMF1['high'] = fuzz.gaussmf(self.inputMF1.universe, gaussParams[2], gaussParams[3])
            self.inputMF2['low'] = fuzz.gaussmf(self.inputMF2.universe, gaussParams[4], gaussParams[5])  # alpha
            self.inputMF2['high'] = fuzz.gaussmf(self.inputMF2.universe, gaussParams[6], gaussParams[7])  # alpha

            self.outputMF1['low'] = fuzz.gaussmf(self.outputMF1.universe, gaussParams[8], gaussParams[9])
            self.outputMF1['high'] = fuzz.gaussmf(self.outputMF1.universe, gaussParams[10], gaussParams[11])

        if mfShape == "sigmoid":
            self.inputMF1['low'] = fuzz.sigmf(self.inputMF1.universe, gaussParams[0], gaussParams[1])
            self.inputMF1['high'] = fuzz.sigmf(self.inputMF1.universe, gaussParams[2], gaussParams[3])
            self.inputMF2['low'] = fuzz.sigmf(self.inputMF2.universe, gaussParams[4], gaussParams[5])  # alpha
            self.inputMF2['high'] = fuzz.sigmf(self.inputMF2.universe, gaussParams[6], gaussParams[7])  # alpha

            self.outputMF1['low'] = fuzz.sigmf(self.outputMF1.universe, gaussParams[8], gaussParams[9])
            self.outputMF1['high'] = fuzz.sigmf(self.outputMF1.universe, gaussParams[10], gaussParams[11])

        # Create fuzzy rules
        rule1 = ctrl.Rule(self.inputMF2['low'] & self.inputMF1['low'], self.outputMF1['high'])
        rule2 = ctrl.Rule(self.inputMF1['high'], self.outputMF1['low'])

        # Create fuzzy control system
        steering_control = ctrl.ControlSystem([rule1, rule2])
        forceWeight = ctrl.ControlSystemSimulation(steering_control)

        # Set inputs and compute output
        forceWeight.input['distanceRatio'] = self.fuzzyInputs[0][0]
        forceWeight.input['relativeAngle'] = self.fuzzyInputs[0][1]

        try:
            forceWeight.compute()
        except:
            print("crisp output not possible", " inputMF1[low]:", gaussParams[0], gaussParams[1], " inputMF1[high]:", gaussParams[2], gaussParams[3], " GAUSSPARAMS", gaussParams)
            # self.inputMF1.view()
            # self.inputMF2.view()
            # plt.show()
            return False

        repulsionVector = forceWeight.output['output'] * np.array(self.obstacleForceVectors[0])
        attractionVector = (1 - forceWeight.output['output']) * np.array(self.targetForceVectors[0])

        resultForceVector = np.add(repulsionVector, attractionVector)
        steeringAngle = math.atan2(resultForceVector[1], resultForceVector[0])  # - self.vehicle.getHeading()

        # saturate steering angle to range
        steeringAngle = self.saturateValue(steeringAngle, self.vehicle.carModel.maxTireAngleRads.value, -self.vehicle.carModel.maxTireAngleRads.value)
        self.vehicle.setTireAngle(steeringAngle, degrees=False)

        if display == True:
            self.displayPlots([self.inputMF1, self.inputMF2], [self.outputMF1], [rule1, rule2])
        return True


    def simulateBest(self, gaussParams):
        while len(self.vehicle.positionMemory) < 100:
            self.navigate2(gaussParams, mfShape = "sigmoid", display = True)
        self.plotTrajectory()
        self.costFunction([gaussParams])

    def plotTrajectory(self):
        positions = np.array(self.vehicle.positionMemory)
        xPositions = []
        yPositions = []
        for x, y in positions:
            xPositions.append(x)
            yPositions.append(y)

        plt.cla()
        plt.plot(xPositions, yPositions)
        plt.scatter(self.obstacle[0].getPosition()[0], self.obstacle[0].getPosition()[1], color="red")
        plt.scatter(self.target[0], self.target[1], color='green')
        self.inputMF1.view()
        self.inputMF2.view()
        self.outputMF1.view()
        plt.show()

    def costFunction(self, individual):

        gaussParams = individual[0]
        while len(self.vehicle.positionMemory) < 100:
            if self.navigate2(gaussParams, mfShape = "sigmoid") == False:
                cost = 1000
                print("COST", cost)
                return cost  #invalid fuzzy controller, return inf cost

        time = len(self.vehicle.positionMemory)
        collision = False
        missedTarget = True
        lateralAccel = 0
        lateralJerk = 0
        longitudinalAccel = 0
        longitudinalJerk = 0

        # check for collisions and reached target
        iteration = 0
        for obstacle in self.obstacles:
            for position in self.vehicle.positionMemory:
                if obstacle == position:
                    collision = True
                    cost =  pow(collision, 2) * 200 + pow(missedTarget, 2) * 5
                    print("hit obstacle")
                    return cost

                elif self.target == position:
                    missedTarget = False
                    time = iteration
                    cost =  + pow(collision, 2) * 2000 + pow(missedTarget, 2)*500 * 5
                    print("hit target")
                    return cost
                iteration += 1

        # final distance to target
        distanceError = pow(pow(self.target[0] - self.vehicle.positionMemory[-1][0], 2) + pow(self.target[1] - self.vehicle.positionMemory[-1][1], 2), 0.5)  # pythagorean distance

        cost = pow(collision, 2) * 2000 + pow(missedTarget, 2)*500 * 5 + pow(distanceError, 2)
        print("COST", cost)
        self.plotTrajectory()
        self.vehicle = Vehicle(JEEP, speed=10)  # reset vehicle
        return cost,

    def gaussGenerator(self):
        sampleRange = np.linspace(-5, 5,1000)
        return random.choices(sampleRange, k=12)

    def learnGenetic(self):

        # creator.create("FitnessMin", base.Fitness, weights = (-1.0,))
        # creator.create("Individual", list, fitness=creator.FitnessMin)
        #
        # toolbox = base.Toolbox()
        # toolbox.register("gaussGenerator", self.gaussGenerator)
        # toolbox.register("individual",tools.initRepeat, creator.Individual, toolbox.gaussGenerator,n=1)
        # toolbox.register("population", tools.initRepeat, list, toolbox.individual, n=1)
        #
        # toolbox.register("evaluate",self.costFunction)
        # toolbox.register("mate", tools.cxTwoPoint)
        # toolbox.register("mutate",tools.mutFlipBit, indpb=0.05)
        # toolbox.register("select",tools.selTournament,tournsize=3)

        print("i got here -1")
        pop = toolbox.population(n=300)
        fitnesses = list(map(toolbox.evaluate, pop))

        # print("sizeof pop", len(pop),"sizeof pop[0]", len(pop), "sizeof fitnesses", len(fitnesses))
        # print("pop", pop)
        # print("fitnesses", fitnesses)
        # print("Zipped", zip(pop, fitnesses))
        for ind, fit in zip(pop, fitnesses):
            # print("i got here 0")
            ind.fitness.values = fit

        CXPB, MUTPB = 0.5, 0.2

        fits = [ind.fitness.values[0] for ind in pop]
        print("i got here 1")
        generation = 0
        while generation < 500:
            generation += 1
            print("GENERATION", generation)

            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))

            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))

            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(child1[0], child2[0])
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant[0])
                    del mutant.fitness.values

            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit

            pop[:] = offspring

            # Gather all the fitnesses in one list and print the stats
            fits = [ind.fitness.values[0] for ind in pop]

        best = pop[np.argmin([toolbox.evaluate(x) for x in pop])]
        print("BEST", best)

        return best


geneticController = LearnController()
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("gaussGenerator", geneticController.gaussGenerator)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.gaussGenerator, n=300)
toolbox.register("population", tools.initRepeat, list, toolbox.individual, n=1)

toolbox.register("evaluate", geneticController.costFunction)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
toolbox.register("select", tools.selTournament, tournsize=3)


def exit_application():
    """Exit program event handler"""

    sys.exit(1)


if __name__ == '__main__':
    # myVehicle = Vehicle(JEEP, 5, [5, 8.66], 3)
    # myVehicle.setTireAngle(0)
    #
    # xPos = random.randint(-25,25)
    # yPos = random.randint(-25,25)
    # target = [xPos, yPos]
    #
    # # target = [0,0]
    # obstacles = [Obstacle(0, [10,0], 0)]  # 1 obstacles
    # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles

    # navigationController = NavigationController(myVehicle, obstacles, target)
    # navigationController.navigate()
    # navSimulation = Simulation(navigationController, obstacles, target)
    # navSimulation.animate()

    # navigationController.adaptiveNavigate()

    # geneticContoller = LearnController()
    # geneticController.simulateBest([0.7591836734693878, 0.1, 1.689795918367347, 0.1, 0.8755102040816326, 0.1, 2.0, 0.5265306122448979, 0.1, 0.1, 0.9530612244897959, 0.1])
    # geneticController.simulateBest([0.810204081632653, 0.15918367346938775, 2.8816326530612244, 0.336734693877551, 0.810204081632653, 1.993877551020408, 2.4081632653061225, 0.9877551020408162, 0.1, 0.1, 1.7571428571428571, 0.336734693877551])
    # geneticController.simulateBest([-0.6565656565656566, -3.9898989898989896, -0.45454545454545503, 2.3737373737373737, 4.595959595959595, 5.0, -3.888888888888889, -2.1717171717171717, 5.0, 4.292929292929292, 2.6767676767676765, 4.494949494949495])
    best = geneticController.learnGenetic()

    for params in best:
        geneticController.simulateBest(params)


