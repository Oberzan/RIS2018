#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist


def rotate(velocity_publisher, speed, angle, clockwise=True):
    print("Started rotating")
    vel_msg = Twist()

    # Converting from angles to radians
    angular_speed = speed * 2 * math.pi / 360
    relative_angle = angle * 2 * math.pi / 360

    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
    print("Finished rotating.")

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
