import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import enum
import sys
from scipy.integrate import odeint

class JEEP(enum.Enum):
    wheelbase = 2.795
    width = 2.00
    hitchOffset = 1.34
    steeringRatio = 0.0652778
    maxTireAngle = 22.5  # degrees


class Vehicle:
    def __init__(self, carModel, speed = 0, position = [0,0], heading = 0):
        self.speed = speed
        self.position = position
        self.heading = heading
        self.carModel = carModel
        self.icTireAngle = 0

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

    def setTireAngle(self, tireAngle):
        self.icTireAngle = tireAngle

    def getTireAngle(self):
        return self.icTireAngle

    def setSteeringAngle(self, steeringWheelAngle):
        self.icTireAngle = steeringWheelAngle*self.carModel.steeringRatio.value

    def getSteeringAngle(self):
        return self.icTireAngle / self.carModel.steeringRatio.value

    def odes(self, x, t):
        x1 = x[0]
        y1 = x[1]
        theta1 = x[2]

        #### ODE PERFECT MODEL - WITH HITCH
        dX1dt = self.speed * np.cos(theta1)
        dY1dt = self.speed * np.sin(theta1)
        dTheta1dt = (self.speed / self.carModel.wheelbase.value) * np.tan(self.icTireAngle)

        return [dX1dt, dY1dt, dTheta1dt]

    def updatePositionHeading(self):

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



class Navigation:
    def __init__(self, vehicle, obstacles, target):
        self.vehicle = vehicle
        self.obstacle = obstacles
        self.target = target
        self.fuzzyInputs = [[0] * 2 for _ in range(len(obstacles))]

    def updateTarget(self, target):
        self.target = target

    def updateObstacles(self, obstacles):
        self.obstacle = obstacles

    def calculateAngles(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacle)):
            currentObstacle = self.obstacle[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            angleObstacleTarget = np.arccos(np.dot(obstacleVector, targetVector) / (np.linalg.norm(obstacleVector) * np.linalg.norm(targetVector)))
            self.fuzzyInputs[i][1] = angleObstacleTarget

    def calculateDistanceRatios(self):
        targetVector = [self.target[0] - self.vehicle.getPosition()[0],
                        self.target[1] - self.vehicle.getPosition()[1]]

        for i in range(len(self.obstacle)):
            currentObstacle = self.obstacle[i]
            obstacleVector = [currentObstacle.getPosition()[0] - self.vehicle.getPosition()[0],
                              currentObstacle.getPosition()[1] - self.vehicle.getPosition()[1]]

            distanceRatio = (np.linalg.norm(obstacleVector) / np.linalg.norm(targetVector))
            self.fuzzyInputs[i][0] = distanceRatio

    def updateFuzzyInputs(self):
        # self.vehicle.setTireAngle(self.vehicle.getTireAngle()+1)
        print("hi")
        self.vehicle.updatePositionHeading()
        self.updateTarget(self.target)
        self.updateObstacles(self.obstacle)
        self.calculateAngles()
        self.calculateDistanceRatios()


class Simulation:
    def __init__(self, vehicle, obstacles, target):
        self.vehicle = vehicle
        self.obstacle = obstacles
        self.target = target

    def updatePlot(self, i):

        self.vehicle.updatePositionHeading()

        plt.cla()
        plt.scatter(self.vehicle.getPosition()[0], self.vehicle.getPosition()[1], color='blue')
        plt.scatter(self.target[0], self.target[1], color='green')

        for i in range(len(self.obstacle)):
            currentObstacle = self.obstacle[i]
            plt.scatter(currentObstacle.getPosition()[0], currentObstacle.getPosition()[1], color='red')

        plt.xlim(-10, 10)
        plt.ylim(-10, 10)

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


    obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)]
    target = [2, 4]

    navigation = Navigation(myVehicle, obstacles, target)
    fuzzySteering = Navigation(myVehicle, obstacles, target)
    fuzzySteering.updateFuzzyInputs()

    navSimulation = Simulation(myVehicle, obstacles, target)

    navSimulation.animate()
