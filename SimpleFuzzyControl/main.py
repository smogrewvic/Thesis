import sys
from deap import base
from deap import creator
from deap import tools
import random

from Vehicle import Vehicle
from Obstacle import Obstacle
from JEEP import JEEP
from NavigationController import NavigationController
from LearnController import LearnController
from Simulator import Simulator


geneticController = LearnController()
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
# toolbox.register("map", futures.map)  # scoop
toolbox.register("parameterGenerator", geneticController.parameterGenerator)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.parameterGenerator, n=300)
toolbox.register("population", tools.initRepeat, list, toolbox.individual, n=1)

toolbox.register("evaluate", geneticController.evaluate)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
toolbox.register("select", tools.selTournament, tournsize=3)

def exit_application():
    """Exit program event handler"""

    sys.exit(1)


if __name__ == '__main__':
    # myVehicle = Vehicle(JEEP, 5, [-15, 0], heading = 0)
    # myVehicle.setTireAngle(0)
    #
    # target = [random.randint(-25,25), random.randint(-25,25)]
    # # target = [7, 0]
    # # target = [0,0]

    # obstacles = [Obstacle(0, [5,0], 0)]  # 1 obstacles
    # # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles
    #
    # navigationController = NavigationController(myVehicle, obstacles, target)
    # navigationController.setMemberships([-0.017017017017017078, -0.9819819819819819, 0.0, 0.2832832832832832, 0.4254254254254255, 0.0, 0.11511511511511507, 0.0, 0.0, -0.987987987987988, 0.3793793793793794, 0.4974974974974975])
    # navigationController.navigate()
    # navSimulator = Simulator(navigationController, obstacles, target)
    # navSimulator.animate()


    # geneticController.simulateBest([0.1, -10 / 20, 0.5, 10 / 20, 0.2, -20 / 20, 0.5, 20 / 20, 0, -20 / 20, 0.6, 10 / 20], mfShape="sigmoid")  # manually tuned
    # geneticController.simulateBest([0.0, -0.87187, 0.203203, 0.0, 0.249249, -0.54954, -0.443443, 0.69969, 0.0, -0.96196, 0.47347, 0.18918], mfShape="sigmoid")  # Verified works
    # geneticController.simulateBest([-0.35935, -0.683683, 0.90790, 0.313313, 0.639639, 0.0, -0.561561, -0.725725, -0.091091, -0.919919, 0.803803, 0.4814814], mfShape="sigmoid")  # Verified works
    geneticController = LearnController()
    best = geneticController.learnGenetic(1000,25)

    for params in best:
        geneticController.simulateBest(params, mfShape = "sigmoid")
