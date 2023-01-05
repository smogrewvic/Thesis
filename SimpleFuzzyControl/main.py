import numpy as np
import math
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
    maxTireAngleRads = maxTireAngle * 0.01745329  # radians


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
        self.obstacleForceVectors = [0]*2
        self.targetForceVectors = 0

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

        self.targetForceVectors = targetVector * 1  # Todo: add a weighting factor, maybe multiple targets?

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))

            self.fuzzyInputs[i][1] = abs(angleObstacleTarget)
            self.obstacleForceVectors[i] = obstacleVector * 1   # Todo: add a weighting factor

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))
            self.fuzzyInputs[i][0] = distanceRatio

    def fuzzyControl(self):
        pass

    def navigate(self):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        # Create fuzzy variables
        relativeAngle = ctrl.Antecedent(np.arange(0, np.pi/2, 1), 'angle')
        distanceRatio = ctrl.Antecedent(np.arange(0, 2, 0.01), 'distanceRatio')
        resultForce = ctrl.Consequent(np.arange(0, 1, 0.01), 'resultForce')

        # Create fuzzy membership functions
        relativeAngle['low'] = fuzz.zmf(relativeAngle.universe, 0, np.pi/8)
        relativeAngle['high'] = fuzz.smf(relativeAngle.universe, np.pi/12, np.pi/4)
        distanceRatio['low'] = fuzz.zmf(distanceRatio.universe, 0, 0.5)
        distanceRatio['high'] = fuzz.smf(distanceRatio.universe, 0.3, 1)

        resultForce['low'] = fuzz.zmf(resultForce.universe, 0, 0.4)
        resultForce['high'] = fuzz.smf(resultForce.universe, 0.2, 0.6)

        # Create fuzzy rules
        rule1 = ctrl.Rule(relativeAngle['low'] & distanceRatio['low'], resultForce['high'])
        rule2 = ctrl.Rule(relativeAngle['high'], resultForce['low'])

        # Create fuzzy control system
        steering_control = ctrl.ControlSystem([rule1, rule2])
        forceWeight = ctrl.ControlSystemSimulation(steering_control)

        # Set inputs and compute output
        forceWeight.input['distanceRatio'] = self.fuzzyInputs[0][0]
        forceWeight.input['angle'] = self.fuzzyInputs[0][1]
        forceWeight.compute()

        repulsionVector = forceWeight.output['resultForce']*np.array(self.obstacleForceVectors[0])
        attractionVector = (1-forceWeight.output['resultForce'])*np.array(self.targetForceVectors[0])

        resultForceVector = np.add(repulsionVector, attractionVector)
        steeringAngle = math.atan2(resultForceVector[1],resultForceVector[0]) - self.vehicle.getHeading()
        steeringAngle = min(self.vehicle.carModel.maxTireAngleRads.value, abs(steeringAngle))  # saturate to max steering angle
        self.vehicle.setTireAngle(steeringAngle, degrees = False)

        print("steeringAngle",steeringAngle*57.7, "vehicleheading",self.vehicle.getHeading())
        # print("angle", self.fuzzyInputs[0][1], "\tdistanceRatio", self.fuzzyInputs[0][0], "\tsteering output", forceWeight.output['resultForce'])


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
    myVehicle = Vehicle(JEEP, 5, [-2,-7], 0)
    myVehicle.setTireAngle(0)

    target = [10, 10]
    obstacles = [Obstacle(0, [5, 5], 0)]  # 1 obstacles
    # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles

    navigationController = NavigationController(myVehicle, obstacles, target)

    navSimulation = Simulation(navigationController, obstacles, target)
    navSimulation.animate()
