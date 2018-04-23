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
            self.positions.append(numpy.random.rand(1, 2)[0])
            self.swm_pop.append(indv)

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def start_evaluation(self, env, pp, pb):
        self.sim = pyrosim.Simulator(eval_time=c.eval_time, play_blind = pb,
                play_paused = pp, dt = 0.025)

        for i, indv in enumerate(self.swm_pop):
            indv.send_to_sim(self.sim, self.positions[i], env.eobjs['plt'])
        env.send_to_sim(self.sim)

        self.sim.assign_collision('robot', 'env')
        # self.sim.assign_collision('env', 'env_c')

        self.sim.start()

    def compute_fitness(self, env):
        self.sim.wait_to_finish()

        plt_pos_data_y = self.sim.get_sensor_data(sensor_id=env.ids['plt'][1], svi=1)
        plt_pos_data_z = self.sim.get_sensor_data(sensor_id=env.ids['plt'][1], svi=2)

        max_sep = 0
        for i in range(0, c.swm_size - 1):
            for j in range(i, c.swm_size - 1):
                pos_1 = numpy.amax(self.sim.get_sensor_data(self.swm_pop[i].robot.P))
                pos_2 = numpy.amax(self.sim.get_sensor_data(self.swm_pop[j].robot.P))
                max_sep = max(max_sep, pos_1, pos_2)

        sep_fitness_penalty = 0
        if max_sep > c.PLT_LENGTH:
            sep_fitness_penalty = max_sep - c.PLT_LENGTH

        # print(plt_pos_data_y, plt_pos_data_z, numpy.std(plt_pos_data_z),
        #         plt_pos_data_y[-1], plt_pos_data_z[-1])

        # print(plt_pos_data_y[-1] + plt_pos_data_z[-1]/numpy.std(plt_pos_data_z))

        # We want to minimize the stdev in height, maximize height, and
        # position of the plate from the origin
        self.fitness = (plt_pos_data_y[-1]
                + plt_pos_data_z[-1]/(1 + numpy.std(plt_pos_data_z))
                - sep_fitness_penalty)
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

