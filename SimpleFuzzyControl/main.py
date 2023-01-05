import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import enum
import sys
from scipy.integrate import odeint
import skfuzzy as fuzz
import skfuzzy.control as ctrl

class JEEP(enum.Enum):
    wheelbase = 2.795
    width = 2.00
    steeringRatio = 0.0652778
    maxTireAngle = 22.5  # degrees


class Vehicle:
    def __init__(self, carModel, speed = 0, position = [0,0], heading = 0):
        self.speed = speed
        self.position = position
        self.heading = heading
        self.carModel = carModel
        self.tireAngle = 0

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

    def setTireAngle(self, tireAngle, degrees = True):

        # todo: limit max tire angle to vehicle model settings
        if degrees == True:
            self.tireAngle = tireAngle * np.pi/180
        else:
            self.tireAngle = tireAngle

    def getTireAngle(self):
        return self.tireAngle

    def setSteeringAngle(self, steeringWheelAngle, degrees = True):
        if degrees == True:
            self.tireAngle = steeringWheelAngle*self.carModel.steeringRatio.value * np.pi/180
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
        t = np.linspace(0, 15, odeStepSize)  # time vector: t start, t end, step size
        x = odeint(self.odes, x0, t)  # ODE calculation

        x1 = x[:, 0]
        y1 = x[:, 1]
        theta1 = x[:, 2]

        self.position, self.heading = [x1[2], y1[2]], theta1[2]  # current positions

        return x1, y1, theta1  # future path


class Obstacle:

    def __init__(self, speed, position, heading):
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
    def __init__(self, speed, position, heading):
        super().__init__(speed, position, heading)
        self.longitudinalSafety = 0
        self.lateralSafety = 0
        self.speedFactor = 2

    def calculateSafety(self):
        self.longitudinalSafety = self.speed * self.speedFactor
        self.lateralSafety = self.speed * self.speedFactor * 0.1


class Pedestrian(Obstacle):
    def __init__(self, speed, position, heading):
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

    def getVehicleData(self):
        return self.vehicle

    def updateTarget(self):
        # todo: aquire new target information
        self.target = target

    def updateObstacles(self):
        # todo: aquire new obstacles from sensors
        self.obstacles = obstacles

    def calculateAngles(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))
            self.fuzzyInputs[i][1] = angleObstacleTarget

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))
            self.fuzzyInputs[i][0] = distanceRatio

    def updateFuzzyInputs(self):

        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

    def fuzzyControl(self):
        pass

    def navigate(self):
        self.updateFuzzyInputs()

        # Create fuzzy variables
        angle = ctrl.Antecedent(np.arange(-90, 90, 1), 'angle')
        distance_ratio = ctrl.Antecedent(np.arange(0, 2, 0.01), 'distance_ratio')
        steering_angle = ctrl.Consequent(np.arange(-45, 45, 1), 'steering_angle')

        # Create fuzzy membership functions
        # angle['left'] = fuzz.trapmf(angle.universe, [-90, -77.5, -22.5, -5])
        angle['left'] = fuzz.zmf(angle.universe, -45,0)
        angle['straight'] = fuzz.trimf(angle.universe, [-10, 0, 10])
        # angle['right'] = fuzz.trapmf(angle.universe, [5, 22.5, 77.5, 90])
        angle['right'] = fuzz.smf(angle.universe, 0, 45)

        # distance_ratio['close'] = fuzz.trimf(distance_ratio.universe, [0, 0, 0.5])
        distance_ratio['close'] = fuzz.zmf(distance_ratio.universe, 0.01, 0.5)
        # distance_ratio['far'] = fuzz.trimf(distance_ratio.universe, [0.5, 1, 2])
        distance_ratio['far'] = fuzz.smf(distance_ratio.universe, 0.5, 1)


        # steering_angle['left'] = fuzz.trimf(steering_angle.universe, [-45, -22.5, 0])
        steering_angle['left'] = fuzz.zmf(steering_angle.universe, -45, 0)
        # steering_angle['right'] = fuzz.trimf(steering_angle.universe, [0, 22.5, 45])
        steering_angle['right'] = fuzz.smf(steering_angle.universe, 0, 45)

        # Create fuzzy rules
        rule1 = ctrl.Rule(angle['left'] & distance_ratio['close'], steering_angle['left'])
        rule2 = ctrl.Rule(angle['left'] & distance_ratio['far'], steering_angle['left'])
        rule3 = ctrl.Rule(angle['straight'] & distance_ratio['close'], steering_angle['left'])
        rule4 = ctrl.Rule(angle['straight'] & distance_ratio['far'], steering_angle['right'])
        rule5 = ctrl.Rule(angle['right'] & distance_ratio['close'], steering_angle['right'])
        rule6 = ctrl.Rule(angle['right'] & distance_ratio['far'], steering_angle['right'])

        # Create fuzzy control system
        steering_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
        steering = ctrl.ControlSystemSimulation(steering_control)

        # Set inputs and compute output
        steering.input['distance_ratio'] = self.fuzzyInputs[0][0]
        steering.input['angle'] = self.fuzzyInputs[0][1]
        steering.compute()

        # Print output

        print("angle", self.fuzzyInputs[0][1] ,"\tdistanceRatio", self.fuzzyInputs[0][0],"\tsteering output", steering.output['steering_angle'])
        self.vehicle.setTireAngle(steering.output['steering_angle'], degrees = True)

class Simulation:
    def __init__(self, navigationObj, obstacles, target):

        self.navigation = navigationObj
        self.vehicle = navigationObj.getVehicleData()
        self.obstacles = obstacles
        self.target = target

    def updatePlot(self, i):

        self.navigation.navigate()

        plt.cla()
        plt.scatter(self.vehicle.getPosition()[0], self.vehicle.getPosition()[1], color='blue')
        plt.scatter(self.target[0], self.target[1], color='green')

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            plt.scatter(currentObstacle.getPosition()[0], currentObstacle.getPosition()[1], color='red')

        plt.xlim(-25, 25)
        plt.ylim(-25, 25)

    def animate(self):
        plt.axis('off')
        ani = FuncAnimation(plt.gcf(), self.updatePlot, interval=10)
        plt.show()



def exit_application():
    """Exit program event handler"""

    sys.exit(1)


if __name__ == '__main__':
    myVehicle = Vehicle(JEEP, 1, [0,0], 0)
    myVehicle.setTireAngle(-5)

    obstacles = [Obstacle(0, [5, 5], 0)]  # 1 obstacles
    # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles

    target = [10, 10]

    navigationController = NavigationController(myVehicle, obstacles, target)



    navSimulation = Simulation(navigationController, obstacles, target)
    navSimulation.animate()
