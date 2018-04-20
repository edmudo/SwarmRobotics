import pyrosim

import constants as c
from environment import Environment

class Environments:

    def __init__(self):
        self.envs = {}
        for eid in range(0, c.num_envs):
            self.envs[eid] = Environment()
