import numpy as np
import math
import matplotlib.pyplot as plt
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import random
from deap import base
from deap import creator
from deap import tools
import multiprocessing
from threading import Thread
import time
from Vehicle import Vehicle
from Obstacle import Obstacle
from JEEP import JEEP
from NavigationController import NavigationController

class LearnController(NavigationController):
    def __init__(self):
        self.vehicle = Vehicle(JEEP, speed=20)
        self.obstacles = [Obstacle(position=[10, 0])]
        self.target = [20, 0]
        super().__init__(self.vehicle, self.obstacles, self.target)

        self.inputMF1 = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        self.inputMF2 = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        self.outputMF1 = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')
        self.rule1 = None
        self.rule2 = None
        self.controlSystem = None
        self.fisSimulation = None


    def simulateBest(self, mfParams, mfShape="sigmoid"):
        self.setMemberships(mfParams, mfShape=mfShape, display=True)
        while len(self.vehicle.dataLog["position"]) < 100:
            self.navigate(display=False)
        self.displayPlots([self.inputMF1, self.inputMF2], [self.outputMF1], [self.rule1, self.rule2])
        self.plotTrajectory()
        self.costFunction([mfParams])

    def plotTrajectory(self):
        positions = np.array(self.vehicle.dataLog["position"])
        xPositions = []
        yPositions = []
        for x, y in positions:
            xPositions.append(x)
            yPositions.append(y)

        plt.cla()
        plt.plot(xPositions, yPositions)
        plt.scatter(self.obstacles[0].getPosition()[0], self.obstacles[0].getPosition()[1], color="red")
        plt.scatter(self.target[0], self.target[1], color='green')
        self.inputMF1.view()
        self.inputMF2.view()
        self.outputMF1.view()
        plt.show()

    def evaluate_threaded(self, individual):
        mfParams = individual[0]
        totalCost = 0
        threads = [None] * 100
        simulationCosts = [0]

        for i in range(len(threads)):
            randomObstacles = []
            for j in range(len(self.obstacles)):
                x = random.randint(-15, 15)
                y = random.randint(-15, 15)
                while [x,y] == self.target: x,y = random.randint(-15, 15), random.randint(-15, 15) #make sure random obstacle isnt over target

                randomObstacles.append(Obstacle(speed = 0, position = [x,y], heading = 0))
            simContainer = LearnControllerContainer(mfParams, "sigmoid", randomObstacles)
            threads[i] = Thread(target=simContainer.costFunction, args=(mfParams, simulationCosts))
            threads[i].start()

        for i in range(len(threads)):
           threads[i].join()

        totalCost += simulationCosts[0]
        print("totalCost",totalCost)
        return totalCost,

    def evaluate2(self, individual):
        mfParams = individual[0]
        totalCost = 0

        for i in range(10):
            randomObstacles = []
            for j in range(len(self.obstacles)):
                x = random.randint(-15, 15)
                y = random.randint(-15, 15)

                while [x,y] == self.target or [x,y] == self.vehicle.getPosition():
                    x,y = random.randint(-15, 15), random.randint(-15, 15) #make sure random obstacle isnt over target

                randomObstacles.append(Obstacle(speed = 0, position = [x,y], heading = 0))
            simContainer = LearnControllerContainer(mfParams, "sigmoid", randomObstacles)
            totalCost += simContainer.costFunction(mfParams)

        print("totalCost",totalCost)
        return totalCost,

    def evaluate(self, individual):
        mfParams = individual[0]
        totalCost = 0

        obstacleAvoidance = [Obstacle(position = [10,0])]
        obstacleNoAvoidance = [Obstacle(position = [5,4])]

        simContainer1 = LearnControllerContainer(mfParams, "sigmoid", obstacleAvoidance)
        simContainer2 = LearnControllerContainer(mfParams, "sigmoid", obstacleNoAvoidance)
        # totalCost = simContainer1.costFunction(mfParams) + simContainer2.costFunction(mfParams)
        totalCost = simContainer1.costFunction(mfParams)

        # print("totalCost", totalCost)
        return totalCost,

    def costFunction(self, mfParams, mutableReturn = None):

        # self.setMemberships(mfParams, mfShape="sigmoid", display=False)
        while len(self.vehicle.dataLog["position"]) < 100:
            if self.navigate() == False:
                cost = 100000
                return cost,  # invalid fuzzy controller, return inf cost

        time = len(self.vehicle.dataLog["position"])
        totalTravel = len(self.vehicle.dataLog["position"]) * self.vehicle.getSpeed()  # not true simulation speed
        closestDistanceToTarget = float('inf')
        collision = False
        missedTarget = True
        lateralAccel = 0
        lateralJerk = 0
        longitudinalAccel = 0
        longitudinalJerk = 0



        # check how close we got to target
        for i in range(len(self.vehicle.dataLog["position"])):
            currentPosition = self.vehicle.dataLog["position"][i]
            currentDistanceToTarget = pow(pow(self.target[0] - currentPosition[0], 2) + pow(self.target[1] - currentPosition[1], 2), 0.5)
            closestDistanceToTarget = min(currentDistanceToTarget, closestDistanceToTarget)

            if closestDistanceToTarget <= 0.2 and missedTarget == True:  # first time target reached
                totalTravel = i * self.vehicle.getSpeed()  # todo: use dataLog["speed"]
                missedTarget = False

            for obstacle in self.obstacles:  # check if safe distance around all obstacles
                currentDistanceToObstacle = pow(pow(obstacle.getPosition()[0] - currentPosition[0], 2) + pow(obstacle.getPosition()[1] - currentPosition[1], 2), 0.5)
                if currentDistanceToObstacle < obstacle.getSafetyRadius():
                    collision = True

        # final distance to target
        # distanceError = pow(pow(self.target[0] - self.vehicle.dataLog["position"][-1][0], 2) + pow(self.target[1] - self.vehicle.dataLog["position"][-1][1], 2), 0.5)  # pythagorean distance
        # cost = pow(collision, 2) * 2000 + pow(missedTarget, 2) * 500 * 5 + pow(distanceError, 2)

        cost = collision*2000 + missedTarget*300 + totalTravel +closestDistanceToTarget*100
        print("COST", str(cost)[0:9], "\t--- Collision:", collision*1, "\tmissedTarget:",missedTarget*300, "\ttotalTravel", totalTravel, "\tclosestDistanceToTarget", closestDistanceToTarget*100)
        self.vehicle = Vehicle(JEEP, speed=10)  # reset vehicle

        if mutableReturn != None:
            mutableReturn[0] += cost

        return cost

    def parameterGenerator(self):
        sampleRange = np.linspace(-1, 1, 1000)
        return random.choices(sampleRange, k=12)


    def learnGenetic(self, population, generations):

        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        toolbox = base.Toolbox()
        pool = multiprocessing.Pool()
        toolbox.register("map", pool.map)  # multicore
        # toolbox.register("map", futures.map)  # scoop

        toolbox.register("parameterGenerator", self.parameterGenerator)
        toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.parameterGenerator, n=1)
        toolbox.register("population", tools.initRepeat, list, toolbox.individual, n=1)

        toolbox.register("evaluate", self.evaluate)
        toolbox.register("mate", tools.cxTwoPoint)
        toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
        toolbox.register("select", tools.selTournament, tournsize=3)
        print("started")
        pop = toolbox.population(n=population)
        fitnesses = list(toolbox.map(toolbox.evaluate, pop))

        for ind, fit in zip(pop, fitnesses):
            # print("i got here 0")
            ind.fitness.values = fit

        CXPB, MUTPB = 0.5, 0.4

        fits = [ind.fitness.values[0] for ind in pop]

        generation = 0
        while generation < generations:
            generation += 1
            print("\n\nGENERATION -------------------------", generation, "\n\n")

            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))

            # Clone the selected individuals
            offspring = list(toolbox.map(toolbox.clone, offspring))

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
            fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit

            pop[:] = offspring

            # Gather all the fitnesses in one list and print the stats
            fits = [ind.fitness.values[0] for ind in pop]

        best = pop[np.argmin([toolbox.evaluate(x) for x in pop])]
        print("BEST", best)

        return best


class LearnControllerContainer(LearnController):
    def __init__(self, mfParams, mfShape, obstacles):
        super().__init__()
        self.obstacles = obstacles
        self.setMemberships(mfParams, mfShape, display = False)


