import sys
from deap import base
from deap import creator
from deap import tools

from Vehicle import Vehicle
from Obstacle import Obstacle
from JEEP import JEEP
from NavigationController import NavigationController
from LearnController import LearnController


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
    # navSimulation = Simulator(navigationController, obstacles, target)
    # navSimulator.animate()

    # navigationController.adaptiveNavigate()

    geneticController = LearnController()
    # geneticController.simulateBest([0.7591836734693878, 0.1, 1.689795918367347, 0.1, 0.8755102040816326, 0.1, 2.0, 0.5265306122448979, 0.1, 0.1, 0.9530612244897959, 0.1])
    # geneticController.simulateBest([0.810204081632653, 0.15918367346938775, 2.8816326530612244, 0.336734693877551, 0.810204081632653, 1.993877551020408, 2.4081632653061225, 0.9877551020408162, 0.1, 0.1, 1.7571428571428571, 0.336734693877551])
    # geneticController.simulateBest([0.0, 0.9595959595959593, -4.898989898989899, -3.787878787878788, 0.5555555555555554, -3.5858585858585856, -1.6666666666666665, 0.050505050505050164, 0.0, 0.0, 1.666666666666667, 0.2525252525252526], mfShape = "sigmoid")  # very good, pop = 300 gen = 25
    # geneticController.simulateBest([0.6065606560656063, -3.0408040804080407, 0.9805980598059802, -4.683968396839684, 0.4815481548154814, -3.194819481948195, 4.822982298229823, -4.033903390339034, 0.8845884588458848, 2.587758775877588, 2.130713071307131, -4.176917691769177], mfShape = "sigmoid") # pop = 3000 gen = 35
    # geneticController.simulateBest([-2.3137313731373137, -0.11251125112511229, 0.0, -4.368936893689369, 0.955595559555956, -3.2488248824882486, 2.6467646764676465, 0.0, 2.6967696769676968, 0.33553355335533563, 0.911591159115912, 4.4519451945194515], mfShape = "sigmoid") # mega run, overtuned pop = 3000 gen = 55

    best = geneticController.learnGenetic(300,25)

    for params in best:
        geneticController.simulateBest(params, mfShape = "sigmoid")
