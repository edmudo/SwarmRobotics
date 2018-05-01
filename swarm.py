import pyrosim
import random
import math
import numpy

import constants as c
from individual import Individual
from robot import Robot

class Swarm:

    SWARM_ID = 0

    def __init__(self):
        # evolution info
        self.lineage_id = Swarm.SWARM_ID
        self.lineage_age = 0
        self.gen = 0

        # swarm
        self.swm_pop = []
        self.positions = []
        self.fitness = 0

        Swarm.SWARM_ID += 1

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
        plt1_posx_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=0)
        plt1_posy_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=1)
        plt1_posz_data = self.sim.get_sensor_data(sensor_id=env.ids['plt1'][1], svi=2)

        plt2_posx_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=0)
        plt2_posy_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=1)
        plt2_posz_data = self.sim.get_sensor_data(sensor_id=env.ids['plt2'][1], svi=2)

        plt_posz_data = numpy.stack((plt1_posz_data, plt2_posz_data))

        tch_arr = []
        vry_arr = []

        # perform calculations for each robot
        for i in range(c.swm_size):
            # gather sensor information
            vry_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.AV)[-1])
            tch_arr.append(self.sim.get_sensor_data(self.swm_pop[i].robot.T)[-1])

        # calucate fitness variables
        touch_thd = 1 if min(tch_arr) == 1 and max(vry_arr) < 0.1 else 0
        x_pos = (plt1_posx_data[-1] + plt2_posx_data[-1])/2
        y_pos = (plt1_posy_data[-1] + plt2_posy_data[-1])/2
        z_pos = (plt1_posz_data[-1] + plt2_posz_data[-1])/2
        dist = math.sqrt(math.pow(x_pos, 2) + math.pow(y_pos, 2))

        # We want to minimize the stdev in height, maximize height and position
        # of the plate from the origin, and ensure that the robots are touching
        # the plate
        self.fitness = touch_thd * dist * z_pos/(1 + numpy.std(plt_posz_data))

        self.lineage_age += 1

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
