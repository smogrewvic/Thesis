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
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401,E402
import random


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

        #testvars
        # self.lastHeading = 0

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

    def saturateValue(self, value, saturationHigh, saturationLow):

        if value > saturationHigh:
            value = saturationHigh
        elif value < saturationLow:
            value = saturationLow

        return value


    def costFunction(self, input1, input2, output):

        time = 0
        crash = 0
        target = 0
        lateralAccel = 0
        lateralJerk = 0
        longitudinalAccel = 0
        longitudinalJerk = 0

        cost = sum(pow(time,2), pow(crash,2)*100, pow(target)*5)


        return (input1 - output) ** 2 + (input2 - output) ** 2


    def navigate(self):
        self.vehicle.updateState()
        self.updateTarget()
        self.updateObstacles()
        self.calculateAngles()
        self.calculateDistanceRatios()

        # Create fuzzy variables
        relativeAngle = ctrl.Antecedent(np.arange(0, np.pi/2, 0.1), 'relativeAngle')
        distanceRatio = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        output = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')


        # print("arange", len(np.arange(0, np.pi/2, 0.01)))
        # Create fuzzy membership functions
        relativeAngle['low'] = fuzz.zmf(relativeAngle.universe, np.pi/4, np.pi/2)
        relativeAngle['high'] = fuzz.smf(relativeAngle.universe, 0.2, np.pi/2)
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

        repulsionVector = forceWeight.output['output']*np.array(self.obstacleForceVectors[0])
        attractionVector = (1-forceWeight.output['output'])*np.array(self.targetForceVectors[0])

        resultForceVector = np.add(repulsionVector, attractionVector)
        steeringAngle = math.atan2(resultForceVector[1],resultForceVector[0]) #- self.vehicle.getHeading()

        # rate limiting steering
        # heading = self.lastHeading + min(max(steeringAngle-self.lastHeading, np.pi/4), -np.pi/4)
        # self.lastHeading = heading
        # self.vehicle.setHeading(heading)

        # saturate steering angle to range
        steeringAngle = self.saturateValue(steeringAngle, self.vehicle.carModel.maxTireAngleRads.value, -self.vehicle.carModel.maxTireAngleRads.value)
        self.vehicle.setHeading(steeringAngle)
        self.vehicle.setTireAngle(steeringAngle, degrees = False)



        # self.displayPlots([relativeAngle, distanceRatio], [output], [rule1, rule2])



    def displayPlots(self, inputMemberships,outputMemberships,rules):

        fis = ctrl.ControlSystem(rules)
        controlSystem = ctrl.ControlSystemSimulation(fis)

        for memFunc in inputMemberships:
            memFunc.view()

        for memFunc in outputMemberships:
            memFunc.view()

        #create 3D plot space
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

        #
        # #plot force Vector
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



def exit_application():
    """Exit program event handler"""

    sys.exit(1)


if __name__ == '__main__':
    myVehicle = Vehicle(JEEP, 5, [5,8.66], 3)
    myVehicle.setTireAngle(0)

    xPos = random.randint(-25,25)
    yPos = random.randint(-25,25)
    target = [xPos, yPos]

    # target = [0,0]
    obstacles = [Obstacle(0, [10,0], 0)]  # 1 obstacles
    # obstacles = [Obstacle(0, [2, 2], 0), Obstacle(0, [2, 3], 0)] # 2 obstacles

    navigationController = NavigationController(myVehicle, obstacles, target)
    navigationController.navigate()
    navSimulation = Simulation(navigationController, obstacles, target)
    navSimulation.animate()

    # navigationController.adaptiveNavigate()