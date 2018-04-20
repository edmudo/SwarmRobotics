import pyrosim
import random
import math
import numpy

import constants as c
from robot import Robot

class Swarm:

    def __init__(self, num_idvs):
        self.genome = []
        for i in range(0, num_idvs):
            self.genome.append(numpy.random.random((9, 5)) * 2 - 1)
        self.fitness = 0

    def __str__(self):
        return  "[" + str(self.id) + " " + str(self.fitness) + "]"

    def start_evaluation(self, env, pp, pb):
        self.sim = pyrosim.Simulator(eval_time=c.eval_time, play_blind = pb,
                play_paused = pp, dt = 0.025)
        env.send_to_sim(self.sim)

        self.sim.assign_collision('robot', 'env')
        self.sim.assign_collision('env', 'env')

        self.sim.start()

    def compute_fitness(self):
        self.sim.wait_to_finish()

        r1_data = self.sim.get_sensor_data(sensor_id = self.robot.S[1])
        r2_data = self.sim.get_sensor_data(sensor_id = self.robot.S[2])
        print(r1_data, r2_data)

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

