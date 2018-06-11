#!/usr/bin/env python

import cv2
import numpy as np
from geometry_msgs.msg import Point

HARDCODED_POINTS = [Point(33, 92, 0), Point(13, 98, 0), Point(45, 83, 0)]


class GoalGenerator:

    def __init__(self, map_img_path, max_num_points=None, erosion_factor=10, goal_step=10):
        self.max_num_points = max_num_points
        self.erosion_factor = erosion_factor
        self.goal_step = goal_step
        self.img = cv2.imread(map_img_path, 0)

    def generate_points(self):
        eroded_image = self.erode_image(self.img)
        goals = self.generate_goals(eroded_image, offset_x=7, offset_y=-2)
        return goals

    def erode_image(self, img):
        kernel = np.ones((self.erosion_factor, self.erosion_factor), np.uint8)
        return cv2.erode(img, kernel, iterations=1)

    def generate_goals(self, img, offset_x=0, offset_y=0):
        print("--------Staring goal generation--------")
        height, width = img.shape
        goals = []
        for y in range(self.goal_step, height, self.goal_step):
            if y < 23:
                continue
            if y > 50 and y < 58:
                continue

            for x in range(self.goal_step, width, self.goal_step):
                if self.max_num_points and len(goals) >= self.max_num_points:
                    return goals

                inner_y = y + offset_y
                inner_x = x + offset_x

                if inner_y < img.shape[0] and inner_x < img.shape[1] and img[inner_y][inner_x] > 250:
                    p = Point(inner_x, inner_y, 0)
                    goals.append(p)

        return goals + HARDCODED_POINTS
