import copy
import random

import constants as c
from swarm import Swarm

class Population:
    def __init__(self, pop_size = 0):
        self.p = {}
        self.pop_size = pop_size
        for i in range(self.pop_size):
            self.p[i] = Swarm()

    def __str__(self):
        s = ""
        for i in self.p:
            s += str(self.p[i]) + " "
        return s

    def evaluate(self, envs, pp, pb, wait_finish = False):
        for i in self.p:
            self.p[i].fitness = 0

        if random.random() <= c.inj_thd:
            self.p[self.pop_size] = Swarm()

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
        # search for the most fit individual
        best_individual_key = 0
        for individual_key in other.p:
            if other.p[individual_key].fitness > other.p[best_individual_key].fitness:
                best_individual_key = individual_key

        self.p[0] = copy.deepcopy(other.p[best_individual_key])

    def winner_of_tournament_selection(self):
        # randomly select individuals
        p1 = random.randint(0, len(self.p) - 1)
        p2 = random.randint(0, len(self.p) - 1)
        while p2 == p1:
            p2 = random.randint(0, len(self.p) - 1)

        # only allow for the younger individual to compete with older ones
        if (self.p[p1].lineage_age <= self.p[p2].lineage_age
                and self.p[p1].fitness >= self.p[p2].fitness):
            return self.p[p1]
        elif (self.p[p2].lineage_age <= self.p[p1].lineage_age
                and self.p[p2].fitness >= self.p[p1].fitness):
            return self.p[p2]

        return self.p[p1]

    def collect_children_from(self, other):
        self.pop_size = other.pop_size
        for i in range(1, self.pop_size):
            winner = other.winner_of_tournament_selection()
            self.p[i] = copy.deepcopy(winner)
            self.p[i].mutate()

        if len(other.p) == other.pop_size + 1:
            # this protects the injected individual
            self.p[self.pop_size] = copy.deepcopy(other.p[other.pop_size])
            self.p[self.pop_size].mutate()

    def fill_from(self, other):
        self.copy_best_from(other)
        self.collect_children_from(other)

