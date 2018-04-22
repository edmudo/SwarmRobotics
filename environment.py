import pyrosim

import constants as c

from env_shape import EnvShape, Rectangle

class Environment:
    def __init__(self, debug = False):
        # the object environment objs
        self.eobjs = {}

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
        self.eobjs[ped.ref] = ped
        self.eobjs[plate.ref] = plate

    def set_placement(self, id, x, y):
        for o in self.eobjs:
            excl_zone = o.get_base_area()
            print(excl_zone)
            if not o.is_off_ground() and (x >= excl_zone[0][0] and x <= excl_zone[1][0]
                    or y >= excl_zone[0][1] and y <= excl_zone[1][1]):
                return False;
        self.p[id] = (x,y)
        return True;

    def send_to_sim(self, sim):
        for eobj_key in self.eobjs:
            eobj = self.eobjs[eobj_key]
            b_id = sim.send_box(x=eobj.x, y=eobj.y, z=eobj.z, length=eobj.l, width=eobj.w,
                    height=eobj.h, collision_group='env', r=0, g=0, b=0)
            if eobj.fixed:
                sim.send_fixed_joint(first_body_id=b_id, second_body_id=-1)
            if eobj.sensor == 'position':
                self.ids[eobj.ref] = sim.send_position_sensor(body_id=b_id)

