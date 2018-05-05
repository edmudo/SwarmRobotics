import pyrosim

import numpy
import pickle
import time

import argparse

import constants as c
import data_fct as df
from environments import Environments
from population import Population

def simulate(init_pop=None, interval=-1, len_time=-1, save_indv=True,
        save_pop=False, show_plots=False, save_csv=False):
    # set up variables
    num_iter = interval if interval > 0 else c.num_iter

    # Create population
    if init_pop is None:
        parents = Population(c.pop_size)
    else:
        parents = init_pop

    start_time = time.time()

    # set up for data collection
    start_pos_data = ([], [])
    end_pos_data = ([], [])
    fitness_data = {}

    # Evaluate population
    envs = Environments()
    parents.evaluate(envs, pp=False, pb=True)
    print("i: 0", parents)

    end_time = time.time()

    # save the first population
    if save_indv:
        save('robota.p', parents.p[0])

    process_pop_dat(0, parents, start_pos_data, end_pos_data,
            fitness_data)

    for i in range(1, num_iter):
        # check if time limit has been exceeded
        if len_time > 0 and end_time - start_time > len_time:
            break

        # create an empty population
        children = Population(0)

        # fill with mutated versions of (likely) the best parents
        children.fill_from(parents)
        children.evaluate(envs, pp=False, pb=True)

        print("i:", i, children)
        parents = children

        end_time = time.time()

        process_pop_dat(i, parents, start_pos_data, end_pos_data,
                fitness_data)

    print("Time elapsed: ", end_time - start_time, "sec")

    if show_plots:
        plot_scatter_data(start_pos_data, 0)
        plot_scatter_data(end_pos_data, 1)
        plot_line_data_dict(fitness_data, 2)
        df.show_plot()

    if save_csv:
        df.write_csv("start.csv", ['x', 'y'], start_pos_data)
        df.write_csv("end.csv", ['x', 'y'], end_pos_data)

        # convert into csv dictionary format
        fitness_data_names = ['gen_time']
        fitness_data_names.extend(numpy.r_[0:len(fitness_data)])
        csv_fitness_data = [numpy.r_[0:num_iter].tolist()]
        for key in fitness_data:
            csv_fitness_data.append(fitness_data[key][1])
        df.write_csv("fitness.csv", fitness_data_names, csv_fitness_data)

    # Save best to file
    if save_indv:
        save('robotb.p', parents.p[0])

    if save_pop:
        save('population.p', parents)

def process_pop_dat(gen_time, population, start_pos_arr, end_pos_arr, fitness_dict):
        pop_start_pos_data, pop_end_pos_data, pop_fitness_data = population.get_population_data()

        for swarm_start_pos_data in pop_start_pos_data:
            start_pos_arr[0].append(swarm_start_pos_data[0])
            start_pos_arr[1].append(swarm_start_pos_data[1])

        for swarm_end_pos_data in pop_end_pos_data:
            end_pos_arr[0].append(swarm_end_pos_data[0])
            end_pos_arr[1].append(swarm_end_pos_data[1])

        for swarm_fitness_data in pop_fitness_data:
            lineage = swarm_fitness_data[0]
            fitness = swarm_fitness_data[1]

            if lineage not in fitness_dict:
                fitness_dict[lineage] = ([gen_time], [0.0])
            elif fitness_dict[lineage][0][-1] != gen_time:
                fitness_dict[lineage][0].append(gen_time)
                fitness_dict[lineage][1].append(fitness)

            # update if the fitness is greater
            if fitness_dict[lineage][1][-1] < fitness:
                fitness_dict[lineage][1][-1] = fitness

def plot_scatter_data(data, fig_num=-1):
    if fig_num != -1:
        df.set_up_plot(fig_num)
    df.plot_scatter(data)

def plot_line_data_dict(data, fig_num=-1):
    if fig_num != -1:
        df.set_up_plot(fig_num)
    for key in data:
        df.plot_line(data[key])

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
    parser.add_argument('--show-plots', action='store_true',
            help='Whether to show plots')
    parser.add_argument('--save-csv', action='store_true',
            help='Whether to save data as csv\'s')
    args = parser.parse_args()

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
    show_plots = args.show_plots
    save_csv = args.save_csv

    simulate(init_pop, i, len_time, save_indv, save_pop, show_plots, save_csv)

