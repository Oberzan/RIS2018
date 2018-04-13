#!/usr/bin/env python

class ClusterPoint():
    def __init__(self, x, y, n, is_visited=False):
        self.x = x
        self.y = y
        self.n = n
        self.is_visited = is_visited

    def __str__(self):
        return "[x: {}, y: {}, n: {}, is_visited: {}]".format(self.x, self.y, self.n, self.is_visited)

    def __repr__(self):
        return self.__str__()

    def move_center(self, p):
        new_x = (self.n * self.x + p.x) / (self.n + 1)
        new_y = (self.n * self.y + p.y) / (self.n + 1)
        return ClusterPoint(new_x, new_y, self.n + 1, self.is_visited)