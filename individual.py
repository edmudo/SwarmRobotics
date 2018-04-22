import pyrosim
import random
import math
import numpy

import constants as c
from robot import Robot

class Individual:

    def __init__(self):
        self.genome = numpy.random.random((9, 5)) * 2 - 1

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def send_to_sim(self, sim, position, plt=None):
        lgt = 1
        wdt = 1
        pcx = 0
        pcy = 0
        if plt is not None:
            lgt = plt.l
            wdt = plt.w
            pcx = plt.x
            pcy = plt.y
        self.robot = Robot(sim, self.genome, pcx + position[0]*lgt/2,
                pcy + position[1]*wdt/2, 0)

    def mutate(self):
        i = random.randint(0, len(self.genome) - 1);
        j = random.randint(0, len(self.genome[0]) - 1);
        self.genome[i][j] = random.gauss(self.genome[i][j],
                math.fabs(self.genome[i][j]))

        if self.genome[i][j] > 1:
            self.genome[i][j] = 1
        elif self.genome[i][j] < -1:
            self.genome[i][j] = -1
