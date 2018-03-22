#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from move_base_msgs.msg import MoveBaseGoal


def point_2_base_goal(point, frame_id="map"):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.pose.orientation.w = 1
    goal.pose.position = point
    goal.header.stamp = rospy.Time(0)
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose = goal
    return move_base_goal


def erode_image(img, erosion_factor):
    kernel = np.ones((erosion_factor, erosion_factor), np.uint8)
    return cv2.erode(img, kernel, iterations=1)


def generate_goals(img, step):
    height, width = img.shape
    goals = set()
    for y in range(step, height, step):
        for x in range(step, width, step):
            if img[y][x] > 250:
                goals.add((x, y))
    return goals


def nearest_goal(point, goals):
    max_dist = 999999999
    c_goal = None
    for p in goals:
        dist = ((point.x - p.x) ** 2 + (point.y - p.y) ** 2) ** 0.5
        if dist < max_dist:
            c_goal = p
            max_dist = dist

    return (c_goal, max_dist)
