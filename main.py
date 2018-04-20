import pyrosim
import copy
import pickle

import constants as c
from environments import Environments
from population import Population

envs = Environments()

# Create population
parents = Population(1)
parents.initialize()

# Evaluate population
parents.evaluate(envs, pp = True, pb = False)

# Save best to file
file = open('robot.p', 'wb')
pickle.dump(parents.p[0], file)
file.close()

