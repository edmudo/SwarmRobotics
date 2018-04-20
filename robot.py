import math
import pyrosim
import random

import constants as c

class Robot:
    def __init__(self, sim, wts, cx, cy, cz):

        self.cx = cx
        self.cy = cy
        self.cz = cz

        self.lgt = c.BASE
        self.wdt = c.BASE/2
        self.hgt = self.rad = c.BASE/5

        self.wheel_rel_x = self.wdt/2 + self.rad/2
        self.wheel_rel_y = self.lgt/2

        # BODY
        self.send_objects(sim)
        self.send_joints(sim)

        # SENSORS
        self.send_sensors(sim)

        # NEURONS + SYNAPSES
        self.send_neurons(sim)
        self.send_synapses(sim, wts)

        # del self.O, self.J, self.S, self.SN, self.MN


    def send_objects(self, sim):
        self.O = {}

        self.O[0] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt, width=self.wdt, height=self.hgt,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[1] = sim.send_cylinder(x=self.cx - self.wheel_rel_x,
                    y=self.cy - self.wheel_rel_y, z=self.cz + self.rad,
                length=self.rad, radius=self.rad, r1=-1, r2=0, r3=0,
                r=0.75, g=0, b=0, collision_group='robot', capped=False)

        self.O[2] = sim.send_cylinder(x=self.cx - self.wheel_rel_x,
                    y=self.cy + self.wheel_rel_y, z=self.cz + self.rad,
                length=self.rad, radius=self.rad, r1=-1, r2=0, r3=0,
                r=0.75, g=0, b=0, collision_group='robot', capped=False)

        self.O[3] = sim.send_cylinder(x=self.cx + self.wheel_rel_x,
                    y=self.cy - self.wheel_rel_y, z=self.cz + self.rad,
                length=self.rad, radius=self.rad, r1=1, r2=0, r3=0,
                r=0.75, g=0, b=0, collision_group='robot', capped=False)

        self.O[4] = sim.send_cylinder(x=self.cx + self.wheel_rel_x,
                    y=self.cy + self.wheel_rel_y, z=self.cz + self.rad,
                length=self.rad, radius=self.rad, r1=1, r2=0, r3=0,
                r=0.75, g=0, b=0, collision_group='robot', capped=False)

        self.O[5] = sim.send_cylinder(x=self.cx, y=self.cy, z=self.cz + 2*self.rad,
                length=3*self.rad, radius=self.rad, r1=0, r2=0, r3=1,
                r=0.75, g=0, b=0, collision_group='robot', capped=False)

        # dud objects for sensors

        self.O[6] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[7] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[8] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[9] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[10] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[11] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[12] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

        self.O[13] = sim.send_box(x=self.cx, y=self.cy, z=self.cz + self.rad,
                length=self.lgt/3, width=self.wdt/3, height=self.hgt/3,
                r=0.5, g=0.5, b=0.5, collision_group='robot')

    def send_joints(self, sim):
        self.J = {}

        self.J[0] = sim.send_hinge_joint(first_body_id=self.O[0],
                second_body_id=self.O[1],
                x=self.cx - self.wheel_rel_x, y=self.cy - self.wheel_rel_y,
                    z=self.cz + self.rad, speed=20,
                n1=-1, n2=0, n3=0, position_control=False)

        self.J[1] = sim.send_hinge_joint(first_body_id=self.O[0],
                second_body_id=self.O[2],
                x=self.cx - self.wheel_rel_x, y=self.cy + self.wheel_rel_y,
                    z=self.cz + self.rad, speed=20,
                n1=-1, n2=0, n3=0, position_control=False)

        self.J[2] = sim.send_hinge_joint(first_body_id=self.O[0],
                second_body_id=self.O[3],
                x=self.cx + self.wheel_rel_x, y=self.cy - self.wheel_rel_y,
                    z=self.cz + self.rad, speed=20,
                n1=1, n2=0, n3=0, position_control=False)

        self.J[3] = sim.send_hinge_joint(first_body_id=self.O[0],
                second_body_id=self.O[4],
                x=self.cx + self.wheel_rel_x, y=self.cy + self.wheel_rel_y,
                    z=self.cz + self.rad, speed=20,
                n1=1, n2=0, n3=0, position_control=False)

        self.J[4] = sim.send_slider_joint(first_body_id=self.O[0],
                second_body_id=self.O[5], x=0, y=0, z=1, lo=-self.rad, hi=0)

        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[6])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[7])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[8])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[9])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[10])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[11])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[12])
        sim.send_fixed_joint(first_body_id=self.O[0], second_body_id=self.O[13])

    def send_sensors(self, sim):
        self.S = {}

        PI4 = math.pi/4
        SPI4 = math.sin(PI4)
        CPI4 = math.cos(PI4)

        # a small offset for sensors due to rounding errors
        ofs = 0.001

        self.S[0] = sim.send_touch_sensor(body_id=self.O[5])

        self.S[1] = sim.send_ray_sensor(body_id=self.O[6], x=self.cx, y=self.cy
                + self.rad + ofs, z=self.cz + 2*self.hgt, r1=0, r2=1, r3=0)
        self.S[2] = sim.send_ray_sensor(body_id=self.O[7],
                x=self.cx + self.rad*math.cos(PI4) + ofs, y=self.cy +
                self.rad*math.sin(PI4) + ofs, z=self.cz + 2*self.hgt, r1=1,
                r2=1, r3=0)
        self.S[3] = sim.send_ray_sensor(body_id=self.O[8], x=self.cx + self.rad
                + ofs, y=self.cy, z=self.cz + 2*self.hgt, r1=1, r2=0, r3=0)
        self.S[4] = sim.send_ray_sensor(body_id=self.O[9], x=self.cx +
                self.rad*math.cos(PI4) + ofs, y=self.cy -
                self.rad*math.sin(PI4) - ofs, z=self.cz + 2*self.hgt, r1=1,
                r2=-1, r3=0)
        self.S[5] = sim.send_ray_sensor(body_id=self.O[10], x=self.cx,
                y=self.cy - self.rad - ofs, z=self.cz + 2*self.hgt, r1=0,
                r2=-1, r3=0)
        self.S[6] = sim.send_ray_sensor(body_id=self.O[11], x=self.cx -
                self.rad*math.cos(PI4) - ofs, y=self.cy -
                self.rad*math.sin(PI4) - ofs, z=self.cz + 2*self.hgt, r1=-1,
                r2=-1, r3=0)
        self.S[7] = sim.send_ray_sensor(body_id=self.O[12], x=self.cx -
                self.rad - ofs, y=self.cy, z=self.cz + 2*self.hgt, r1=-1, r2=0,
                r3=0)
        self.S[8] = sim.send_ray_sensor(body_id=self.O[13], x=self.cx -
                self.rad*math.cos(PI4) - ofs, y=self.cy +
                self.rad*math.sin(PI4) + ofs, z=self.cz + 2*self.hgt, r1=-1,
                r2=1, r3=0)

    def send_neurons(self, sim):
        self.SN = {}
        for s in self.S:
            self.SN[s] = sim.send_sensor_neuron(sensor_id = self.S[s])

        self.MN = {}
        for j in self.J:
            self.MN[j] = sim.send_motor_neuron(joint_id = self.J[j], tau = 1)

    def send_synapses(self, sim, wts):
        for i in self.SN:
            for j in self.MN:
                sim.send_synapse(source_neuron_id = self.SN[i],
                        target_neuron_id = self.MN[j], weight = wts[i][j])

        # sim.send_synapse(source_neuron_id = self.SN[0],
        #         target_neuron_id = self.MN[4], weight = 1)

