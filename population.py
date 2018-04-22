import copy
import random

import constants as c
from swarm import Swarm

class Population:
    def __init__(self, pop_size = 0):
        self.p = {}
        self.pop_size = pop_size

    def __str__(self):
        s = ""
        for i in self.p:
            s += str(self.p[i]) + " "
        return s

    def initialize(self):
        for i in range(self.pop_size):
            self.p[i] = Swarm(i)

    def evaluate(self, envs, pp, pb, wait_finish = False):
        for i in self.p:
            self.p[i].fitness = 0

        for e in range(0, c.num_envs):
            if wait_finish:
                for i in self.p:
                    self.p[i].start_evaluation(envs.envs[e], pp, pb)
                    self.p[i].compute_fitness()
            else:
                for i in self.p:
                    self.p[i].start_evaluation(envs.envs[e], pp, pb)
                for i in self.p:
                    self.p[i].compute_fitness(envs.envs[e])

    def mutate(self):
        for i in self.p:
            self.p[i].mutate()

    def replace_with(self, other):
        for i in self.p:
            if self.p[i].fitness < other.p[i].fitness:
                self.p[i] = other.p[i]

    def copy_best_from(self, other):
        best_individual_key = 0
        for individual_key in other.p:
            if other.p[individual_key].fitness > other.p[best_individual_key].fitness:
                best_individual_key = individual_key

        self.p[0] = copy.deepcopy(other.p[best_individual_key])

    def winner_of_tournament_selection(self):
        p1 = random.randint(0, self.pop_size - 1)
        p2 = random.randint(0, self.pop_size - 1)

        while p2 == p1:
            p2 = random.randint(0, self.pop_size - 1)

        if self.p[p1].fitness > self.p[p2].fitness:
            return self.p[p1]
        else:
            return self.p[p2]

    def collect_children_from(self, other):
        for i in range(1, self.pop_size):
            winner = other.winner_of_tournament_selection()
            self.p[i] = copy.deepcopy(winner)
            self.p[i].mutate()

    def fill_from(self, other):
        self.copy_best_from(other)
        self.collect_children_from(other)

