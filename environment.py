import pyrosim

import constants as c

from env_shape import EnvShape, Rectangle

class Environment:
    def __init__(self, debug = False):
        # the object environment objs
        self.o = []

        # ids of parts that have sensors
        self.ids = {}

        # the x,y position for each agent
        self.p = {}

        self.debug = debug

        self.set_up_env()

    def set_up_env(self):
        ped = Rectangle(0, 0, c.PED_HEIGHT/2, c.PED_LENGTH, c.PED_WIDTH,
                c.PED_HEIGHT, True)
        plate = Rectangle(0, 0, c.PED_HEIGHT + c.PLT_HEIGHT/2, c.PLT_LENGTH,
                c.PLT_WIDTH, c.PLT_HEIGHT, False, sensor='position', ref='plt')
        self.o.append(ped)
        self.o.append(plate)

    def set_placement(self, id, x, y):
        for o in self.o:
            excl_zone = o.get_base_area()
            print(excl_zone)
            if not o.is_off_ground() and (x >= excl_zone[0][0] and x <= excl_zone[1][0]
                    or y >= excl_zone[0][1] and y <= excl_zone[1][1]):
                return False;

        self.p[id] = (x,y)
        return True;

    def send_to_sim(self, sim):
        for o in self.o:
            if isinstance(o, EnvShape):
                id = sim.send_box(x=o.x, y=o.y, z=o.z, length=o.l, width=o.w,
                        height=o.h, collision_group='env', r=0, g=0, b=0)
                if o.fixed:
                    sim.send_fixed_joint(id, -1)

                if o.sensor == 'position':
                    sim.send_position_sensor(id)
                    self.ids[o.ref] = id

        # if self.debug:
        #     sim.send_box(x=0, y=1.5, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=0, y=-3, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=3, y=0, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=-3, y=0, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=-3, y=3, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=3, y=-3, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=-3, y=-3, z=2, length=1, width=1, height=2)
        #     sim.send_box(x=3, y=3, z=2, length=1, width=1, height=2)
