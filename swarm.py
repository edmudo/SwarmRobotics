import pyrosim
import random
import math
import numpy

import constants as c
from individual import Individual

class Swarm:

    SWARM_ID = 0

    def __init__(self):
        # evolution info
        self.lineage_id = Swarm.SWARM_ID
        self.lineage_age = 0
        self.gen = 0

        self.last_data = {}

        # swarm
        self.swm_pop = []
        self.positions = {}
        self.fitness = 0

        Swarm.SWARM_ID += 1

        for i in range(c.swm_size):
            indv = Individual(i, self.positions)

            # track the positions
            self.positions[indv.id] = indv.position
            self.swm_pop.append(indv)

    def __str__(self):
        str_arr = [str(self.gen), '[', str(self.lineage_id), ' ',
                str(self.fitness), ']']
        return ''.join(str_arr)

    def start_evaluation(self, env, pp, pb):
        self.sim = pyrosim.Simulator(eval_time=c.eval_time, play_blind = pb,
                play_paused = pp, dt = 0.025)

        for i, indv in enumerate(self.swm_pop):
            indv.send_to_sim(self.sim, indv.position,
                    0, 0, c.PLT_LENGTH, c.PLT_WIDTH)
        env.send_to_sim(self.sim)

        self.sim.assign_collision('robot', 'robot')
        self.sim.assign_collision('robot', 'env')
        self.sim.assign_collision('env', 'env_c')

        self.sim.start()

    def compute_fitness(self, env):
        self.sim.wait_to_finish()

        # center plate position data
        plt1_pos_x_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=0)
        plt1_pos_y_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=1)
        plt1_pos_z_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=2)

        plt2_pos_x_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=0)
        plt2_pos_y_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=1)
        plt2_pos_z_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=2)

        plt_pos_x_data = (plt1_pos_x_data + plt2_pos_x_data)/2
        plt_pos_y_data = (plt1_pos_y_data + plt2_pos_y_data)/2
        plt_pos_z_data = (plt1_pos_z_data + plt2_pos_z_data)/2

        # gather sensor data for each robot
        tch_arr = []
        ryd_arr = []
        pos_x_arr = []
        pos_y_arr = []

        for i in range(c.swm_size):
            ryd_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.AV)[-1])
            tch_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.T)[-1])
            pos_x_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=0))
            pos_y_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.P, svi=1))

        # calucate fitness variables
        touch_thd = 1 if min(tch_arr) == 1 and max(ryd_arr) < 0.1 else 0
        plt_x_end = plt_pos_x_data[-1]
        plt_y_end = plt_pos_y_data[-1]
        plt_z_end = plt_pos_z_data[-1]
        plt_dist = math.sqrt(math.pow(plt_x_end, 2) + math.pow(plt_y_end, 2))

        # We want to minimize the stdev in height, maximize height and position
        # of the plate from the origin, and ensure that the robots are touching
        # the plate
        self.fitness = touch_thd * plt_dist * plt_z_end/(1 + numpy.std(plt_pos_z_data))

        self.lineage_age += 1

        self.last_data['plt_pos_x'] = plt_pos_x_data
        self.last_data['plt_pos_y'] = plt_pos_y_data
        self.last_data['plt_pos_z'] = plt_pos_z_data
        self.last_data['bots_pos_x'] = pos_x_arr
        self.last_data['bots_pos_y'] = pos_y_arr

        del self.sim

    def mutate(self):
        for indv in self.swm_pop:
            indv.mutate(self.positions)
            self.positions[indv.id] = indv.position

        self.gen += 1

    def get_data(self, key=None):
        if key is not None:
            return self.last_data[key]

        data = {}

        bots_pos_x = self.last_data['bots_pos_x']
        bots_pos_y = self.last_data['bots_pos_y']
        plt_pos_x = self.last_data['plt_pos_x']
        plt_pos_y = self.last_data['plt_pos_y']

        # for graphing purposes
        for i in range(c.swm_size):
            start_rel_x = bots_pos_x[i][0] - plt_pos_x[0]
            start_rel_y = bots_pos_y[i][0] - plt_pos_y[0]

            end_rel_x = bots_pos_x[i][-1] - plt_pos_x[-1]
            end_rel_y = bots_pos_y[i][-1] - plt_pos_y[-1]

            data['start_pos'] = (start_rel_x, start_rel_y)
            data['end_pos'] = (end_rel_x, end_rel_y)

        return data

