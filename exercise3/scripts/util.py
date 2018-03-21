#!/usr/bin/env python

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
