from itertools import groupby
from numpy.polynomial.polynomial import polyfit
import numpy as np


class Trader(object):
    def __init__(self):
        self.coin_values = [1, 5, 10]

    def _calculate_gains_for_line(self, k, n):
        value_on_day_5 = k * 5 + n
        value_on_day_7 = k * 7 + n
        gains = float(value_on_day_7) / float(value_on_day_5)
        if value_on_day_5 <= 0:
            return value_on_day_5, value_on_day_7, 0
        
        return value_on_day_5, value_on_day_7, gains

    def merge_data(self, points):
        merged_data = dict(points[0].get_actual_data())
        for data_point in points[1:]:
            temp_data = data_point.get_actual_data()

            if temp_data.get('k', None):
                merged_data['k'] = temp_data.get('k')

            merged_data['points'] = (merged_data.get('points', []) + temp_data.get('points', []))
        return merged_data


    def _calculate_gains(self, color, group):
        data_points = list(group)
        merged_data = self.merge_data(data_points)

        points = merged_data.get('points', [])
        x = np.array([p[0] for p in points])
        y = np.array([p[1] for p in points])
        if len(points) > 2:
            k, n = polyfit(x, y, 1)
        elif len(points) == 2:
            p0 = points[0]
            p1 = points[1]
            k = (p1[1] - p0[1]) / (p1[0] - p0[0])
            n = p1[1] - (p1[0] * k)
        else:
            k = merged_data.get('k')
            p = points[0]
            n = p[1] - (k * p[0])

        d5, d7, gains = self._calculate_gains_for_line(k, n)

        print("Color: ", color)
        print("k: ", k)
        print("n: ", n)
        print("5th day: ", d5)
        print("7th day: ", d7)
        print("Gains: ", gains)
        print("Merged data: ", merged_data)
        print("--------------\n\n")
        return gains


    def get_job_gains(self, data_points):
        print("DATA POINTS: ", data_points)
        print("\n\n--------CALCULATING JOB GAINS--------------\n\n")
        data_points = sorted(data_points, key=lambda item: item.get_discrete_color())
        groups = groupby(data_points, lambda data_point: (data_point.get_discrete_color()))
        res = {}
        for key, group in groups:
            gains = self._calculate_gains(key, group)
            res[key] = gains

        return res
