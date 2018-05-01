import pyrosim

import constants as c

from env_shape import EnvShape, Rectangle

class Environment:
    def __init__(self, debug = False):
        # the object environment objs
        self.eobjs = {}

        # ids of parts that have sensors
        self.ids = {}

        self.debug = debug
        self.set_up_env()

    def set_up_env(self):
        plt_comp_x_adj = c.PLT_LENGTH/2 - c.PLT_COMP_WIDTH/2
        plt_comp_y_adj = c.PLT_WIDTH/2 - c.PLT_COMP_WIDTH/2
        plt_comp_hgt_adj = c.PED_HEIGHT + c.PLT_COMP_HEIGHT/2

        # ped = Rectangle(x=0, y=0, z=c.PED_HEIGHT/2, l=c.PED_LENGTH, w=c.PED_WIDTH,
        #         h=c.PED_HEIGHT, fixed=True, cg='env_c')
        # self.eobjs[ped.ref] = ped

        # plate = Rectangle(x=0, y=0, z=c.PED_HEIGHT + c.PLT_HEIGHT/2, l=c.PLT_LENGTH,
        #         w=c.PLT_WIDTH, h=c.PLT_HEIGHT, fixed=False, sensor='position', ref='plt')
        # self.eobjs[plate.ref] = plate

        plate1 = Rectangle(x=0, y=plt_comp_y_adj, z=plt_comp_hgt_adj,
                l=c.PLT_COMP_WIDTH, w=c.PLT_COMP_LENGTH, h=c.PLT_COMP_HEIGHT,
                fixed=False, sensor='position', ref='plt1')
        self.eobjs[plate1.ref] = plate1

        plate2 = Rectangle(x=0, y=-plt_comp_y_adj, z=plt_comp_hgt_adj,
                l=c.PLT_COMP_WIDTH, w=c.PLT_COMP_LENGTH, h=c.PLT_COMP_HEIGHT,
                fixed=True, sensor='position', fixed_to=plate1.ref, ref='plt2')
        self.eobjs[plate2.ref] = plate2

        plate3 = Rectangle(x=plt_comp_x_adj, y=0, z=plt_comp_hgt_adj,
                l=c.PLT_COMP_LENGTH - 2*c.PLT_COMP_WIDTH, w=c.PLT_COMP_WIDTH, h=c.PLT_COMP_HEIGHT,
                fixed=True, fixed_to=plate1.ref, ref='plt3')
        self.eobjs[plate3.ref] = plate3

        plate4 = Rectangle(x=-plt_comp_x_adj, y=0, z=plt_comp_hgt_adj,
                l=c.PLT_COMP_LENGTH - 2*c.PLT_COMP_WIDTH, w=c.PLT_COMP_WIDTH, h=c.PLT_COMP_HEIGHT,
                fixed=True, fixed_to=plate1.ref, ref='plt4')
        self.eobjs[plate4.ref] = plate4

    def is_placement(self, x, y, robot):
        for o in self.eobjs:
            excl_zone = o.get_base_area()
            if not o.is_off_ground() and (x >= excl_zone[0][0] + robot.x/2 and x <= excl_zone[1][0 - robot.x/2]
                    or y >= excl_zone[0][1] + robot.y/2 and y <= excl_zone[1][1] + robot.y/2):
                return False;
        return True;

    def send_to_sim(self, sim):
        for eobj_key in self.eobjs:
            eobj = self.eobjs[eobj_key]
            b_id = sim.send_box(x=eobj.x, y=eobj.y, z=eobj.z, length=eobj.l, width=eobj.w,
                    height=eobj.h, mass=10, collision_group=eobj.cg, r=1, g=0, b=0)
            if eobj.fixed:
                if eobj.fixed_to == -1:
                    sim.send_fixed_joint(first_body_id=b_id, second_body_id=-1)
                else:
                    sim.send_fixed_joint(first_body_id=b_id, second_body_id=self.ids[eobj.fixed_to][0])
            if eobj.sensor == 'position':
                self.ids[eobj.ref] = (b_id, sim.send_position_sensor(body_id=b_id))

