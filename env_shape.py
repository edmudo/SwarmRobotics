from abc import ABC, abstractmethod

class EnvShape(ABC):
    _ref_id = 0

    def __init__(self, x, y, z, fixed, sensor='none', ref=-1):
        self.x = x
        self.y = y
        self.z = z

        if ref == -1:
            ref = _ref_id
            _ref_id += 1

        self.ref = ref
        self.fixed = fixed
        self.sensor = sensor

    @abstractmethod
    def is_off_ground(self):
        return

    @abstractmethod
    def get_base_area(self):
        return

class Rectangle(EnvShape):
    def __init__(self, x, y, z, l, w, h, fixed):
        super(Rectangle, self).__init__(x, y, z, fixed)

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
