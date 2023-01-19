import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



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

        # plot force Vector
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
