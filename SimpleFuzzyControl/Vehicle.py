import numpy as np
from scipy.integrate import odeint

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

    def setHeading(self, heading, degrees = True):
        if degrees == True:
            self.heading = heading* np.pi / 180
        else:
            self.heading = heading

    def getHeading(self, degrees = True):
        if degrees == True:
            return self.heading * 180/np.pi
        else:
            return self.heading

    def setTireAngle(self, tireAngle, degrees=True):

        # todo: limit max tire angle to vehicle model settings
        if degrees == True:
            self.tireAngle = tireAngle * np.pi / 180
        else:
            self.tireAngle = tireAngle

    def getTireAngle(self, degrees = True):
        if degrees == True:
            return self.tireAngle * 180/np.pi
        else:
            return self.tireAngle

    def setSteeringAngle(self, steeringWheelAngle, degrees=True):
        if degrees == True:
            self.tireAngle = steeringWheelAngle * self.carModel.steeringRatio.value * np.pi / 180
        else:
            self.tireAngle = steeringWheelAngle * self.carModel.steeringRatio.value

    def getSteeringAngle(self, degrees = True):
        if degrees == True:
            return (self.tireAngle * np.pi / 180) / self.carModel.steeringRatio.value
        else:
            return self.tireAngle / self.carModel.steeringRatioRads.value


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
        t = [0, 0.02]
        x = odeint(self.odes, x0, t)  # ODE calculation

        x1 = x[:, 0]
        y1 = x[:, 1]
        theta1 = x[:, 2]
        self.position, self.heading = [x1[1], y1[1]], theta1[1]%(2*np.pi)  # current positions

        # log data
        self.positionMemory.append(self.position)
        # np.append(self.positionMemory, self.position, axis = 0)

        return x1, y1, theta1  # future path