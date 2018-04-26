import pyrosim
import random
import math
import numpy

import constants as c
from individual import Individual
from robot import Robot

class Swarm:

    def __init__(self, id):
        self.id = id
        self.swm_pop = []
        self.positions = []
        self.fitness = 0

        for i in range(c.swm_size):
            indv = Individual()
            self.positions.append(numpy.random.rand(1, 2)[0] * 2 - 1)
            self.swm_pop.append(indv)

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def start_evaluation(self, env, pp, pb):
        self.sim = pyrosim.Simulator(eval_time=c.eval_time, play_blind = pb,
                play_paused = pp, dt = 0.025)

        for i, indv in enumerate(self.swm_pop):
            indv.send_to_sim(self.sim, self.positions[i],
                    0, 0, c.PLT_LENGTH, c.PLT_WIDTH)
        env.send_to_sim(self.sim)

        self.sim.assign_collision('robot', 'env')
        self.sim.assign_collision('env', 'env_c')

        self.sim.start()

    def compute_fitness(self, env):
        self.sim.wait_to_finish()

        plt_posx_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=0)
        plt_posy_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=1)
        plt_posz_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=2)

        max_sep = 0
        tch_arr = []
        for i in range(c.swm_size):
            tch_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.T)[-1])
            r_posx = numpy.amax(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=0))
            r_posy = numpy.amax(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=1))
            max_sep = max(max_sep, math.sqrt(math.pow(r_posx-plt_posx_data[-1], 2) +
                math.pow(r_posy-plt_posy_data[-1], 2)))

        touch_thd = min(tch_arr)
        sep_penalty = max_sep
        if max_sep - c.PLT_LENGTH/2 < 0:
            sep_penalty = 0
        # print(tch_arr)
        # print("TESTING:", touch_thd, sep_penalty)

        # print(plt_posy_data, plt_posz_data, numpy.std(plt_posz_data),
        #         plt_posy_data[-1], plt_posz_data[-1])

        # We want to minimize the stdev in height, maximize height, and
        # position of the plate from the origin
        self.fitness = touch_thd*(plt_posy_data[-1]
                + plt_posz_data[-1]/(1 + numpy.std(plt_posz_data))
                - sep_penalty)
        del self.sim

    def mutate(self):
        m_var = random.randint(0, 1)
        m_indv = random.randint(0, c.swm_size - 1)

        if m_var:
            self.swm_pop[m_indv].mutate()
        else:
            i = random.randint(0, 1)
            self.positions[m_indv][i] = random.gauss(self.positions[m_indv][i],
                    math.fabs(self.positions[m_indv][i]))

            if i == 0:
                if self.positions[m_indv][i] < c.PLT_LENGTH/2:
                    self.positions[m_indv][i] = -c.PLT_LENGTH/2
                elif self.positions[m_indv][i] > c.PLT_LENGTH/2:
                    self.positions[m_indv][i] = c.PLT_LENGTH/2
            else:
                if self.positions[m_indv][i] < c.PLT_WIDTH/2:
                    self.positions[m_indv][i] = -c.PLT_WIDTH/2
                elif self.positions[m_indv][i] > c.PLT_WIDTH/2:
                    self.positions[m_indv][i] = c.PLT_WIDTH/2

