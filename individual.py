import pyrosim
import random
import math
import numpy

import constants as c
from robot import Robot

class Individual:

    def __init__(self):
        self.genome = numpy.random.random((10, 5)) * 2 - 1

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def send_to_sim(self, sim, position, b_x=0, b_y=0, b_lgt=1, b_wdt=1, rand=False):
        pcx = b_x
        pcy = b_y
        if rand:
            pcx = random.randint(-1, 1)
            pcy = random.randint(-1, 1)
        self.robot = Robot(sim, self.genome, pcx + position[0]*b_lgt/2,
                pcy + position[1]*b_wdt/2, 0)

    def mutate(self):
        i = random.randint(0, len(self.genome) - 1);
        j = random.randint(0, len(self.genome[0]) - 1);
        self.genome[i][j] = random.gauss(self.genome[i][j],
                math.fabs(self.genome[i][j]))

        if self.genome[i][j] > 1:
            self.genome[i][j] = 1
        elif self.genome[i][j] < -1:
            self.genome[i][j] = -1
