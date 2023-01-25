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

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)


def exit_application():
    """Exit program event handler"""

    sys.exit(1)


if __name__ == '__main__':
    myVehicle = Vehicle(JEEP, 5, [-15, 0], heading = 0)
    myVehicle.setTireAngle(0)

    target = [random.randint(-25,25), random.randint(-25,25)]
    # target = [7, 0]
    # target = [0,0]

    obstacles = [Obstacle(0, [5,0], 0)]  # 1 obstacles
    # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles

    navigationController = NavigationController(myVehicle, obstacles, target)
    navigationController.setMemberships([0.0, -0.9619619619619619, 0.15715715715715706, 0.91991991991992, 0.0, -0.27127127127127126, 0.3793793793793794, 0.0, 0.0, -0.7877877877877878, 0.7177177177177176, 0.8118118118118118])
    navigationController.navigate()
    navSimulator = Simulator(navigationController, obstacles, target)
    navSimulator.animate()


    # geneticController.simulateBest([0.1, -10 / 20, 0.5, 10 / 20, 0.2, -20 / 20, 0.5, 20 / 20, 0, -20 / 20, 0.6, 10 / 20], mfShape="sigmoid")  # manually tuned
    # geneticController.simulateBest([0.0, -0.87187, 0.203203, 0.0, 0.249249, -0.54954, -0.443443, 0.69969, 0.0, -0.96196, 0.47347, 0.18918], mfShape="sigmoid")  # Verified works
    # geneticController.simulateBest([-0.35935, -0.683683, 0.90790, 0.313313, 0.639639, 0.0, -0.561561, -0.725725, -0.091091, -0.919919, 0.803803, 0.4814814], mfShape="sigmoid")  # Verified works

    # geneticController = LearnController()
    # best = geneticController.learnGenetic(1000,25)
    #
    # for params in best:
    #     geneticController.simulateBest(params, mfShape = "sigmoid")
