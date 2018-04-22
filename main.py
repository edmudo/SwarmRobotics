import pyrosim
import copy
import pickle

import constants as c
from environments import Environments
from population import Population

envs = Environments()

# Create population
parents = Population(c.pop_size)
parents.initialize()

# Evaluate population
parents.evaluate(envs, pp=False, pb=True)
print("g: 0", parents)

for g in range(1, c.num_gens):
    # create an empty population
    children = Population(c.pop_size)

    # fill with mutated versions of (likely) the best parents
    children.fill_from(parents)
    children.evaluate(envs, pp = False, pb = True)

    print("g:", g, children)
    parents = children

# Save best to file
file = open('robot.p', 'wb')
pickle.dump(parents.p[0], file)
file.close()

