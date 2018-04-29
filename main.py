import pyrosim

import copy
import pickle
import time

import argparse

import constants as c
from environments import Environments
from population import Population

def simulate(init_pop=None, interval=-1, len_time=-1, save_indv=True,
        save_pop=False):
    # set up variables
    num_iter = interval if interval > 0 else c.num_iter

    # Create population
    if init_pop is None:
        parents = Population(c.pop_size)
        parents.initialize()
    else:
        parents = init_pop

    start_time = time.time()

    # Evaluate population
    envs = Environments()
    parents.evaluate(envs, pp=False, pb=True)
    print("i: 0", parents)

    end_time = time.time()

    # save the first population
    if save_indv:
        save('robota.p', parents.p[0])

    for i in range(1, num_iter):
        # check if time limit has been exceeded
        end_time = time.time()
        if len_time > 0 and end_time - start_time > len_time:
            break

        # create an empty population
        children = Population(c.pop_size)

        # fill with mutated versions of (likely) the best parents
        children.fill_from(parents)
        children.evaluate(envs, pp = False, pb = True)

        print("i:", i, children)
        parents = children

    print("Time elapsed: ", end_time - start_time, "sec")

    # Save best to file
    if save_indv:
        save('robotb.p', parents.p[0])
    if save_pop:
        save('pop.p', parents)

def save(filename, obj):
    file = open(filename, 'wb')
    pickle.dump(obj, file)
    file.close()

def load(filename):
    file = open(filename, 'rb')
    obj = pickle.load(file)
    file.close()
    return obj

def playback(filename):
    indv = load(filename)
    envs = Environments()

    for e in range(0, c.num_envs):
        indv.start_evaluation(envs.envs[e], pp = False, pb = False)
        indv.compute_fitness(envs.envs[e])
    print(indv.fitness)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Runs a evo robotics sim.")
    parser.add_argument('-c', '--continue', metavar='FILENAME', dest='cont',
            help='A filename containing population object.')
    parser.add_argument('-i', '--iterations', type=int, default=-1, metavar='N',
            help='The number of iterations to run for.')
    parser.add_argument('-p', '--playback', nargs='*', default=argparse.SUPPRESS,
            help='Playback the list of saved individuals')
    parser.add_argument('-t', '--time', type=int, default=-1, metavar='SECONDS',
            help='The length of time in seconds to run the simulation. Takes precedence over iterations.')
    parser.add_argument('--no-save', action='store_true',
            help='Whether to save the first and last individuals.')
    parser.add_argument('--save-pop', action='store_true',
            help='Whether to save the population.')
    args = parser.parse_args()

    print(vars(args))

    if hasattr(args, 'playback'):
        if len(args.playback) < 1:
            playback('robota.p')
            playback('robotb.p')
        else:
            for filename in args.playback:
                playback(filename)
        exit()

    init_pop = load(args.cont) if args.cont is not None else args.cont
    len_time = args.time
    i = args.iterations
    save_indv = not args.no_save
    save_pop = args.save_pop

    simulate(init_pop, i, len_time, save_indv, save_pop)
