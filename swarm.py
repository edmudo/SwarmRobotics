import pyrosim
import random
import math
import numpy

import constants as c
from individual import Individual
from robot import Robot

class Swarm:

    def __init__(self, lineage_id):
        # evolution info
        self.lineage_id = lineage_id
        self.gen = 0

        # swarm
        self.swm_pop = []
        self.positions = []
        self.fitness = 0

        for i in range(c.swm_size):
            indv = Individual()

            # determine if a generated position is suitable (no collisions)
            p_arr = numpy.random.rand(1, 2)[0] * 2 - 1
            while not self._is_pos_suitable(p_arr):
                p_arr = numpy.random.rand(1, 2)[0] * 2 - 1

            # append the suitable positions
            self.positions.append(p_arr)
            self.swm_pop.append(indv)

    def __str__(self):
        str_arr = [str(self.gen), '[', str(self.lineage_id), ' ',
                str(self.fitness), ']']
        return ''.join(str_arr)

    def start_evaluation(self, env, pp, pb):
        self.sim = pyrosim.Simulator(eval_time=c.eval_time, play_blind = pb,
                play_paused = pp, dt = 0.025)

        for i, indv in enumerate(self.swm_pop):
            indv.send_to_sim(self.sim, self.positions[i],
                    0, 0, c.PLT_LENGTH, c.PLT_WIDTH)
        env.send_to_sim(self.sim)

        self.sim.assign_collision('robot', 'robot')
        self.sim.assign_collision('robot', 'env')
        self.sim.assign_collision('env', 'env_c')

        self.sim.start()

    def compute_fitness(self, env):
        self.sim.wait_to_finish()

        # plate position data
        plt_posx_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=0)
        plt_posy_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=1)
        plt_posz_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=2)

        max_sep = 0
        tch_arr = []

        # perform calculations for each robot
        for i in range(c.swm_size):
            # gather sensor information
            tch_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.T)[-1])
            r_posx = numpy.amax(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=0))
            r_posy = numpy.amax(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=1))

            # compute the separation between the robot and the plate
            max_sep = max(max_sep, math.sqrt(math.pow(r_posx-plt_posx_data[-1], 2) +
                math.pow(r_posy-plt_posy_data[-1], 2)))

        # calucate fitness variables
        touch_thd = min(tch_arr)
        sep_penalty = max_sep
        if max_sep - c.PLT_LENGTH/2 < 0:
            sep_penalty = 0

        # We want to minimize the stdev in height, maximize height, position of
        # the plate from the origin, minimize separation, and ensure that the
        # robots are touching the plate
        self.fitness = touch_thd*(plt_posy_data[-1]
                + plt_posz_data[-1]/(1 + numpy.std(plt_posz_data))
                - sep_penalty)

        del self.sim

    def mutate(self):
        # select a random individual
        m_indv = random.randint(0, c.swm_size - 1)

        # either mutate the individual or its position
        m_var = random.randint(0, 1)
        if m_var:
            self.swm_pop[m_indv].mutate()
        else:
            factor = 1.0        # to encourage converging on a pt and settling
            m_position = self._mutate_pos(self.positions[m_indv])

            while not self._is_pos_suitable(m_position, self.positions[m_indv]):
                factor *= 10.0
                m_position = self._mutate_pos(self.positions[m_indv], factor)

            self.positions[m_indv] = m_position

        self.gen += 1

    def _mutate_pos(self, position, factor=1):
        m_position = numpy.copy(position)

        axis = random.randint(0,1)

        # in case if the robot has coordingates of (0,0)
        sd = math.fabs(position[axis])/factor if position[axis] != 0 else 0.001

        m_position[axis] = random.gauss(position[axis], sd)
        return m_position

    def _is_pos_suitable(self, xypos, ignore=None):
        # when out of bounds of the plate
        if abs(xypos[0]) > c.PLT_LENGTH/2 or abs(xypos[1] > c.PLT_WIDTH/2):
            return False

        # determine if a generated position is suitable (no collisions)
        for position in self.positions:
            if position is ignore or position is xypos:
                continue
            # base collision check on the robot dimensions
            if (abs(position[0] - xypos[0]) < c.R_WIDTH + 2*c.R_WLENGTH
                    and abs(position[1] - xypos[1]) < c.R_LENGTH + 2*c.R_RADIUS):
                return False

        return True

