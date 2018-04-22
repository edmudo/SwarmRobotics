import pyrosim
import random
import math
import numpy

import constants as c
from individual import Individual
from robot import Robot

class Swarm:

    def __init__(self):
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
        self.sim.assign_collision('env', 'env_c')

        self.sim.start()

    def compute_fitness(self, env):
        self.sim.wait_to_finish()

        plt_pos_data_y = self.sim.get_sensor_data(sensor_id=env.ids['plt'], svi=1)
        plt_pos_data_z = self.sim.get_sensor_data(sensor_id=env.ids['plt'], svi=2)

        print(plt_pos_data_y, plt_pos_data_z, numpy.std(plt_pos_data_z),
                plt_pos_data_y[-1], plt_pos_data_z[-1])

        print(plt_pos_data_y[-1] + plt_pos_data_z[-1]/numpy.std(plt_pos_data_z))

        # We want to minimize the stdev in height, maximize height, and
        # position of the plate from the origin
        self.fitness = plt_pos_data_y[-1] + plt_pos_data_z[-1]/numpy.std(plt_pos_data_z)
        print(self.fitness)
        del self.sim

    def mutate(self):
        i = random.randint(0, 4);
        j = random.randint(0, 7);
        self.genome[i][j] = random.gauss(self.genome[i][j],
                math.fabs(self.genome[i][j]))

        if self.genome[i][j] > 1:
            self.genome[i][j] = 1
        elif self.genome[i][j] < -1:
            self.genome[i][j] = -1

