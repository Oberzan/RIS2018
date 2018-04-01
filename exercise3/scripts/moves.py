#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from constants import ROTATING, DEFAULT, OBSERVING


def rotate(velocity_publisher, speed, angle, state_publisher, step_angle=45, clockwise=True, sleep_duration=2):
    print("Started rotating")
    vel_msg = Twist()

    # Converting from angles to radians
    angular_speed = speed * 2 * math.pi / 360
    relative_angle = angle * 2 * math.pi / 360
    relative_step_angle = step_angle * 2 * math.pi / 360

    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    actual_speed = -abs(angular_speed) if clockwise else abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    num_stops = int(angle / step_angle)
    print("Angle completed in {} steps.".format(num_stops))

    relative_angle_inner = current_angle + relative_step_angle
    string_message = String()

    while current_angle < relative_angle:
        vel_msg.angular.z = actual_speed
        string_message.data = ROTATING
        state_publisher.publish(string_message)
        print("Publishing state: {}".format(string_message.data))

        while current_angle < relative_angle_inner:
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)

        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        string_message.data = OBSERVING
        print("Publishing state: {}".format(string_message.data))
        state_publisher.publish(string_message)
        rospy.sleep(sleep_duration)
        relative_angle_inner += relative_step_angle

    string_message.data = DEFAULT
    state_publisher.publish(string_message)
    print("Publishing state: {}".format(string_message.data))

    print("Finished rotating.")
