import pyrosim
import random
import math
import numpy

import constants as c
from robot import Robot

class Individual:

    def __init__(self, id, positions):
        self.id = id

        self.genome = numpy.random.random((10, 5)) * 2 - 1

        self.position = None
        self.gen_pos(positions)

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def gen_pos(self, positions):
        # determine if a generated position is suitable (no collisions)
        self.position = numpy.random.rand(1, 2)[0] * 2 - 1
        while not self._is_pos_suitable(self.position, positions):
            self.position = numpy.random.rand(1, 2)[0] * 2 - 1

    def send_to_sim(self, sim, position, b_x=0, b_y=0, b_lgt=1, b_wdt=1, rand=False):
        pcx = b_x
        pcy = b_y
        if rand:
            pcx = random.randint(-1, 1)
            pcy = random.randint(-1, 1)
        self.robot = Robot(sim, self.genome, pcx + position[0]*b_lgt/2,
                pcy + position[1]*b_wdt/2, 0)

    def mutate(self, positions):
        m_var = random.randint(0, 1)
        if m_var:
            self._mutate_genome()
        else:
            self._mutate_pos(positions)

    def _mutate_genome(self):
        i = random.randint(0, len(self.genome) - 1);
        j = random.randint(0, len(self.genome[0]) - 1);
        self.genome[i][j] = random.gauss(self.genome[i][j],
                math.fabs(self.genome[i][j]))

        if self.genome[i][j] > 1:
            self.genome[i][j] = 1
        elif self.genome[i][j] < -1:
            self.genome[i][j] = -1

    def _mutate_pos(self, positions):
        factor = 1.0        # to encourage converging on a pt and settling
        m_position = self._mutate_pos_helper(self.position)

        # check for collisions, ignoring this individual's position
        while not self._is_pos_suitable(m_position, positions, self.position):
            factor *= 10.0
            m_position = self._mutate_pos_helper(self.position, factor)

        self.position = m_position

    def _mutate_pos_helper(self, position, factor=1):
        m_position = numpy.copy(position)

        axis = random.randint(0,1)

        # in case if the robot has coordingates of (0,0)
        sd = math.fabs(position[axis])/factor if position[axis] != 0 else 0.001

        m_position[axis] = random.gauss(position[axis], sd)
        return m_position

    def _is_pos_suitable(self, xypos, positions, ignore=None):
        # when out of bounds of the plate
        if abs(xypos[0]) > c.PLT_LENGTH/2 or abs(xypos[1] > c.PLT_WIDTH/2):
            return False

        # determine if a generated position is suitable (no collisions)
        for indv_id in positions:
            position = positions[indv_id]
            if position is ignore or position is xypos:
                continue
            # base collision check on the robot dimensions
            if (abs(position[0] - xypos[0]) < c.R_WIDTH + 2*c.R_WLENGTH
                    and abs(position[1] - xypos[1]) < c.R_LENGTH + 2*c.R_RADIUS):
                return False

        return True

