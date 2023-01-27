import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



class Simulator:
    def __init__(self, navigationObj, obstacles, target):
        self.navigation = navigationObj
        self.vehicle = navigationObj.getVehicleData()
        self.obstacles = obstacles
        self.target = target

    def updatePlot(self, i):
        resultForce, repulsionVector, attractionVector = self.navigation.navigate()

        plt.cla()
        plt.scatter(self.vehicle.getPosition()[0], self.vehicle.getPosition()[1], color='blue')
        plt.scatter(self.target[0], self.target[1], color='green')

        for i in range(len(self.obstacles)):
            currentObstacle = self.obstacles[i]
            plt.scatter(currentObstacle.getPosition()[0], currentObstacle.getPosition()[1], color='red')

        # plot force Vector
        forceX = [self.vehicle.getPosition()[0], self.vehicle.getPosition()[0]+resultForce[0]*10]
        forceY = [self.vehicle.getPosition()[1], self.vehicle.getPosition()[1]+resultForce[1]*10]
        plt.plot(forceX, forceY)
        repulseX =[self.vehicle.getPosition()[0], self.vehicle.getPosition()[0]+repulsionVector[0]*10]
        repulseY = [self.vehicle.getPosition()[1], self.vehicle.getPosition()[1]+repulsionVector[1]*10]

        attractX =[self.vehicle.getPosition()[0], self.vehicle.getPosition()[0]+attractionVector[0]*10]
        attractY = [self.vehicle.getPosition()[1], self.vehicle.getPosition()[1]+attractionVector[1]*10]
        plt.plot(repulseX, repulseY)
        plt.plot(attractX, attractY)

        plt.xlim(-25, 25)
        plt.ylim(-25, 25)
        # time.sleep(5)

    def animate(self):
        plt.axis('off')
        ani = FuncAnimation(plt.gcf(), self.updatePlot, interval=10)
        plt.show()
