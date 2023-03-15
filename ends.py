import itertools
import math

import numpy as np
from shapely.geometry import LineString

from utm_func import utm_dist


class Ends:

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def line(self):
        return LineString([self.p1, self.p2])

    def array(self):
        return np.array([[self.p1.x, self.p1.y], [self.p2.x, self.p2.y]])

    def points(self):
        return [self.p1, self.p2]

    def close_to(self, other):
        if utm_dist(self.p1, other.p1) < 0.5:
            return True
        if utm_dist(self.p1, other.p2) < 0.5:
            return True
        if utm_dist(self.p2, other.p1) < 0.5:
            return True
        if utm_dist(self.p2, other.p2) < 0.5:
            return True
        return False

    def overlaps(self, other):
        for r in itertools.product(self.points(), other.points()):
            if r[0].touches(r[1]):
                return True
        return False

    def touches(self, other):
        if self.line().intersects(other.line()):
            return True
        return False

    def collinear(self, other):
        s1 = (self.p2.y - self.p1.y) / (self.p2.x - self.p1.x)
        s2 = (other.p2.y - other.p1.y) / (other.p2.x - other.p1.x)
        return math.degrees(math.atan((s2 - s1) / (1 + (s2 * s1)))) < 5

    def is_same_line(self, other):
        if self.close_to(other) or self.overlaps(other) or self.touches(other):
            return True
        return False
