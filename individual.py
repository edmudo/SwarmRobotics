import pyrosim

class Swarm:
    def __init__(num_idvs):
        self.swarm = {}
        for i in range(0, num_indvs):
            self.swarm[i] = Individual()
