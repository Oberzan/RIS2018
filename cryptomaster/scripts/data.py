
class ClusterPoint():
    def __init__(self, x, y, n, is_visited=False, color=None, discrete_colors=None, data=None):
        self.x = x
        self.y = y
        self.n = n
        self.is_visited = is_visited
        self.color = color
        self.data = data
        self.discrete_colors = discrete_colors

    def __str__(self):
        return "[x: {}, y: {}, n: {}, is_visited: {}, color: {}, discrete_color: {}, data: {}]".format(self.x, self.y, self.n,
                                                                                        self.is_visited,
                                                                                        self.color, self.get_discrete_color(), self.data)

    def __repr__(self):
        return self.__str__()


    def get_discrete_color(self):
        print(self.discrete_colors)
        try:
            return max(self.discrete_colors, key=self.discrete_colors.get)
        except ValueError:
            return None


    def move_center(self, p, new_data, new_color, new_discrete_color):
        new_x = (self.n * self.x + p.x) / (self.n + 1)
        new_y = (self.n * self.y + p.y) / (self.n + 1)
        actual_data = new_data if new_data is not None else self.data

        if new_discrete_color:

            if self.discrete_colors == None or not new_discrete_color in self.discrete_colors:
                self.discrete_colors[new_discrete_color] = 1
            else:
                self.discrete_colors[new_discrete_color] += 1

        return ClusterPoint(new_x, new_y, self.n + 1, self.is_visited, new_color, self.discrete_colors, actual_data)
