from abc import ABC, abstractmethod

class EnvShape(ABC):
    _ref_id = 0

    def __init__(self, x, y, z, fixed, fixed_to, sensor, ref, cg, r, g, b):
        self.x = x
        self.y = y
        self.z = z

        if ref == -1:
            ref = EnvShape._ref_id
            EnvShape._ref_id += 1

        self.ref = ref
        self.fixed = fixed
        self.fixed_to = fixed_to
        self.sensor = sensor
        self.cg = cg
        self.r = r
        self.g = g
        self.b = b

    @abstractmethod
    def is_off_ground(self):
        return

    @abstractmethod
    def get_base_area(self):
        return

class Rectangle(EnvShape):
    def __init__(self, x, y, z, l, w, h, fixed, fixed_to=-1, sensor='none',
            ref=-1, cg='env', r=1, g=0, b=0):
        super(Rectangle, self).__init__(x, y, z, fixed, fixed_to, sensor, ref, cg,
                r, g, b)

        self.l = l
        self.w = w
        self.h = h

    def is_off_ground(self):
        return self.z - self.h > 0

    def get_base_area(self):
        x1 = self.x - self.l/2
        x2 = self.x + self.l/2

        y1 = self.y - self.w/2
        y2 = self.y + self.w/2

        return [(x1, y1), (x2, y2)]
