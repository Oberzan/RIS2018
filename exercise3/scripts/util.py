#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal
from math import atan2


def flip(m, axis):
    if not hasattr(m, 'ndim'):
        m = np.asarray(m)
    indexer = [slice(None)] * m.ndim
    try:
        indexer[axis] = slice(None, None, -1)
    except IndexError:
        raise ValueError("axis=%i is invalid for the %i-dimensional input array"
                         % (axis, m.ndim))
    return m[tuple(indexer)]

def angle_to_goal(viewpoint, target):
    angle = atan2(target.y - viewpoint.y, target.x - viewpoint.x)
    print("Angle to goal: {}".format(angle))
    return angle

def normalize_value(value, max=1, min=0):
    return (value - min) / (max - min)

def normalize(data, max=1, min=0):
    data.x = normalize_value(data.x, max, min)
    data.y = normalize_value(data.y, max, min)
    data.z = normalize_value(data.z, max, min)

def orientation_vector(viewpoint, target):
    orientation = Quaternion()
    orientation.x = target.x - viewpoint.x
    orientation.y = target.y - viewpoint.y
    orientation.w = 1
    normalize(orientation)
    return orientation

def approached_target(viewpoint, orientation, approach_factor=0.7):
    p = Point()
    p.x = viewpoint.x + (orientation.x * approach_factor)
    p.y = viewpoint.y + (orientation * approach_factor)
    return p

def point_2_base_goal(point, frame_id="map", orientation=None):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.pose.orientation.w = 1
    if orientation:
        goal.pose.orientation = orientation

    goal.pose.position = point
    goal.header.stamp = rospy.Time(0)
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose = goal
    return move_base_goal


def erode_image(img, erosion_factor):
    kernel = np.ones((erosion_factor, erosion_factor), np.uint8)
    return cv2.erode(img, kernel, iterations=1)


def generate_goals(img, step, offset_x=0, offset_y=0):
    height, width = img.shape
    goals = []
    for y in range(step, height, step):
        for x in range(step, width, step):
            if len(goals) >= 2:
                return goals
            
            inner_y = y + offset_y
            inner_x = x + offset_x

            if inner_y < img.shape[0] and inner_x < img.shape[1] and img[inner_y][inner_x] > 250:
                p = Point()
                p.x = inner_x
                p.y = inner_y

                goals.append(p)

    ## Hardcoded for competition
    h1 = Point()
    h1.x = 33
    h1.y = 92
    goals.append(h1)
    h2 = Point()
    h2.x = 13
    h2.y = 98
    goals.append(h2)
    h3 = Point()
    h3.x = 45
    h3.y = 80
    goals.append(h3)
    return goals


def nearest_goal(point, goals):
    max_dist = 999999999
    c_goal = None
    for p in goals:
        dist = point_distance(point, p)
        if dist < max_dist:
            c_goal = p
            max_dist = dist

    return (c_goal, max_dist)


def point_distance(p1, p2):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5