#!/usr/bin/env python

# from __future__ import print_function

import roslib

roslib.load_manifest('exercise3')
import sys
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from constants import ACTION_CLIENT_STATUSES, ROTATE_SPEED, ROTATE_ANGLE, GOAL_RESULT_TIMEOUT, ROTATING, DEFAULT, MOVING, CIRCLE_APPROACHING, CIRCLE_APPROACHED
from moves import rotate
from util import point_2_base_goal, flip, orientation_vector, approached_target, angle_to_goal
from tf.transformations import euler_from_quaternion


class Rotator():


    def __init__(self):
        rospy.init_node('rotating_test')
        self.action_client = SimpleActionClient("move_base", MoveBaseAction)
        self.action_client.wait_for_server()
        self.cv_map = None
        self.map_transform = None
        self.map_resolution = 0
        self.circles_detected = 0
        self.circle_rotating_rate = rospy.Rate(10)
        self.circle_to_visit_jobs = []
        self.state = DEFAULT


        # Odom subscriber
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

       # Velocity publisher
        self.velocity_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)


        print("MapGoals initialized.")


        viewpoint = Point(-1, 1.7, 0)
        target = Point(-1.12, 2.525, 0)



        self.rotate_to_goal(viewpoint, target)












    def rotate_to_goal(self, viewpoint, goal):
        print("Rotating from viewpoint {}".format(viewpoint))
        print("Rotating to goal {}".format(goal))

        angle = angle_to_goal(viewpoint, goal)


        speed = Twist()
        while abs(angle - self.theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 1
            self.velocity_publisher.publish(speed)
            self.circle_rotating_rate.sleep()
            print(angle, self.theta)

        speed.angular.z = 0.0
        self.velocity_publisher.publish(speed)

        print("Orientated to goal!.")


    def odom_callback(self, odometry):
        rot_q = odometry.pose.pose.orientation
        (roll, pitch, jaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # print("self.theta={}".format(jaw))
        self.theta = jaw

    def approach(self, viewpoint, target, orientation):
        print("Approaching...")
        closer_viewpoint = approached_target(viewpoint, orientation)
        move_base_goal = point_2_base_goal(closer_viewpoint, orientation=orientation)
        self.action_client.send_goal(move_base_goal)
        self.action_client.wait_for_result(rospy.Duration(GOAL_RESULT_TIMEOUT))
        print("Aproached circle!")
        print("Action result: " + ACTION_CLIENT_STATUSES[self.action_client.get_state()])




def main(args):
    _ = Rotator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
