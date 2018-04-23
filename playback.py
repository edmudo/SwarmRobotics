import pickle

import constants as c

from environments import Environments
from population import Population

filea = open ('robota.p', 'rb')
fileb = open ('robotb.p', 'rb')
strt_individual = pickle.load(filea)
best_individual = pickle.load(fileb)
filea.close()
fileb.close()

envs = Environments()

for e in range(0, c.num_envs):
    strt_individual.start_evaluation(envs.envs[e], pp = False, pb = False)
    strt_individual.compute_fitness(envs.envs[e])
    best_individual.start_evaluation(envs.envs[e], pp = False, pb = False)
    best_individual.compute_fitness(envs.envs[e])
