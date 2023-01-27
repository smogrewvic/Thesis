import skfuzzy as fuzz
import skfuzzy.control as ctrl
import numpy as np

class Obstacle:

    def __init__(self, speed=0, position=[0, 0], heading=0):
        self.speed = speed
        self.position = position
        self.heading = heading
        self.safetyRadius = 1
        self.fisSimulation = 0


        self.inputMF1 = None
        self.inputMF2 = None
        self.outputMF1 = None
        self.rule1 = None
        self.rule2 = None
        self.controlSystem = None
        self.fisSimulation = None
        self.defaultSettings()

    def defaultSettings(self):
        # Create fuzzy variables
        self.inputMF1 = ctrl.Antecedent(np.arange(0, np.pi / 2, 0.1), 'relativeAngle')
        self.inputMF2 = ctrl.Antecedent(np.arange(0, 2, 0.1), 'distanceRatio')
        self.outputMF1 = ctrl.Consequent(np.arange(0, 1, 0.1), 'output')

        # Create fuzzy membership functions
        self.inputMF1['low'] = fuzz.sigmf(self.inputMF1.universe, -0.35935, -0.683683*20)
        self.inputMF1['high'] = fuzz.sigmf(self.inputMF1.universe, 0.90790, 0.313313*20)
        self.inputMF2['low'] = fuzz.sigmf(self.inputMF2.universe, 0.639639, 0.0*20)  # alpha
        self.inputMF2['high'] = fuzz.sigmf(self.inputMF2.universe, -0.561561, -0.725725*20)  # alpha
        self.outputMF1['low'] = fuzz.sigmf(self.outputMF1.universe, -0.091091, -0.919919*20)
        self.outputMF1['high'] = fuzz.sigmf(self.outputMF1.universe, 0.803803, 0.4814814*20)

        # Create fuzzy rules
        self.rule1 = ctrl.Rule(self.inputMF2['low'] & self.inputMF1['low'], self.outputMF1['high'])
        self.rule2 = ctrl.Rule(self.inputMF1['high'], self.outputMF1['low'])

        # Create fuzzy control system
        self.controlSystem = ctrl.ControlSystem([self.rule1, self.rule2])
        self.fisSimulation = ctrl.ControlSystemSimulation(self.controlSystem)

    def setMemberships(self, mfParams, mfShape="sigmoid", display=False):

        if mfShape == "gauss":
            for i in range(len(mfParams)):  # check that no mfParams became zero from mutation (zero std dev)
                if mfParams[i] <= 0:
                    mfParams[i] = 0.01
                    # return False
            self.inputMF1['low'] = fuzz.gaussmf(self.inputMF1.universe, mfParams[0], mfParams[1])
            self.inputMF1['high'] = fuzz.gaussmf(self.inputMF1.universe, mfParams[2], mfParams[3])
            self.inputMF2['low'] = fuzz.gaussmf(self.inputMF2.universe, mfParams[4], mfParams[5])  # alpha
            self.inputMF2['high'] = fuzz.gaussmf(self.inputMF2.universe, mfParams[6], mfParams[7])  # alpha

            self.outputMF1['low'] = fuzz.gaussmf(self.outputMF1.universe, mfParams[8], mfParams[9])
            self.outputMF1['high'] = fuzz.gaussmf(self.outputMF1.universe, mfParams[10], mfParams[11])

        if mfShape == "sigmoid":
            self.inputMF1['low'] = fuzz.sigmf(self.inputMF1.universe, mfParams[0], mfParams[1] * 20)
            self.inputMF1['high'] = fuzz.sigmf(self.inputMF1.universe, mfParams[2], mfParams[3] * 20)
            self.inputMF2['low'] = fuzz.sigmf(self.inputMF2.universe, mfParams[4], mfParams[5] * 20)  # alpha
            self.inputMF2['high'] = fuzz.sigmf(self.inputMF2.universe, mfParams[6], mfParams[7] * 20)  # alpha

            self.outputMF1['low'] = fuzz.sigmf(self.outputMF1.universe, mfParams[8], mfParams[9] * 20)
            self.outputMF1['high'] = fuzz.sigmf(self.outputMF1.universe, mfParams[10], mfParams[11] * 20)

        # Create fuzzy rules
        self.rule1 = ctrl.Rule(self.inputMF2['low'] & self.inputMF1['low'], self.outputMF1['high'])
        self.rule2 = ctrl.Rule(self.inputMF1['high'], self.outputMF1['low'])

        # Create fuzzy control system
        self.controlSystem = ctrl.ControlSystem([self.rule1, self.rule2])
        self.fisSimulation = ctrl.ControlSystemSimulation(self.controlSystem)


    def computeFuzzy(self, inputs):

        # Set inputs and compute output
        self.fisSimulation.input['distanceRatio'] = inputs[0]
        self.fisSimulation.input['relativeAngle'] = inputs[1]

        try:
            self.fisSimulation.compute()
        except:
            print("crisp output not possible")
            return False

        return self.fisSimulation.output['output']


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



class RoadLimit(Obstacle):
    pass
