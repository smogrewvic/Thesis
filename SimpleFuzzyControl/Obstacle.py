
class Obstacle:

    def __init__(self, speed=0, position=[0, 0], heading=0):
        self.speed = speed
        self.position = position
        self.heading = heading
        self.safetyRadius = 1

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

    def setSafetyRadius(self, safetyRadius):
        self.safetyRadius = safetyRadius

    def getSafetyRadius(self):
        return self.safetyRadius


class Car(Obstacle):
    def __init__(self, speed=0, position=[0, 0], heading=0):
        super().__init__(speed, position, heading)
        self.longitudinalSafety = 0
        self.lateralSafety = 0.2
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