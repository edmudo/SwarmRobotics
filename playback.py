import pickle

import constants as c

from environments import Environments
from population import Population

file = open ('robot.p', 'rb')
best_individual = pickle.load(file)
file.close()

envs = Environments()

for e in range(0, c.num_envs):
    best_individual.start_evaluation(envs.envs[e], pp = False, pb = False)
    best_individual.compute_fitness()
