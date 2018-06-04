class ClusterPoint():
    def __init__(self, x, y, n, is_visited=False, color=None, discrete_color=None, data=None):
        self.x = x
        self.y = y
        self.n = n
        self.is_visited = is_visited
        self.discrete_color = discrete_color
        self.color = color
        self.data = data

    def __str__(self):
        return "[x: {}, y: {}, n: {}, is_visited: {}, color: {}, discrete_color: {}, data: {}]".format(self.x, self.y, self.n,
                                                                                        self.is_visited,
                                                                                        self.color, self.discrete_color, self.data)

    def __repr__(self):
        return self.__str__()

    def move_center(self, p, new_data):
        new_x = (self.n * self.x + p.x) / (self.n + 1)
        new_y = (self.n * self.y + p.y) / (self.n + 1)
        actual_data = new_data if new_data is not None else self.data
        return ClusterPoint(new_x, new_y, self.n + 1, self.is_visited, self.color, self.discrete_color, actual_data)
