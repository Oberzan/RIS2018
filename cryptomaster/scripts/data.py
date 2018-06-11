import json

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
        return "[n: {}, is_visited: {}, discrete_color: {}, data: {}]".format(self.n,
                                                                                        self.is_visited, self.get_discrete_color(), self.data)

    def reset_cluster_point(self):
        return ClusterPoint(self.x, self.y, 1, True, self.color, self.discrete_colors, self.data)

    def __repr__(self):
        return self.__str__()

    def get_actual_data(self):
        try:
            return json.loads(max(self.data, key=self.data.get))
        except ValueError as e:
            print(e)
            return None


    def get_discrete_color(self):
        try:
            return max(self.discrete_colors, key=self.discrete_colors.get)
        except ValueError:
            return None


    def move_center(self, p, new_data, new_color, new_discrete_color):
        new_x = (self.n * self.x + p.x) / (self.n + 1)
        new_y = (self.n * self.y + p.y) / (self.n + 1)


        if new_data:
            d_data = json.loads(new_data)
            if d_data.get('k', None) != None and d_data.get('points', []) == []:
                self.data = {}

            if self.data == None:
                self.data = {}

            if new_data in self.data:
                self.data[new_data] += 1
            else:
                self.data[new_data] = 1

        if new_discrete_color:
            if self.discrete_colors == None:
                self.discrete_colors = {}

            if not new_discrete_color in self.discrete_colors:
                self.discrete_colors[new_discrete_color] = 1
            else:
                self.discrete_colors[new_discrete_color] += 1

        return ClusterPoint(new_x, new_y, self.n + 1, self.is_visited, new_color, self.discrete_colors, self.data)
