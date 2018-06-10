import math
import rospy
from geometry_msgs.msg import Twist
import states as states



def rotate(velocity_publisher, speed, angle, step_angle=45, clockwise=True, sleep_duration=0.1, state_func=None):
    print("Started rotating")
    # Converting from angles to radians
    relative_angle = angle * 2 * math.pi / 360
    current_angle = 0

    num_stops = int(angle / step_angle)
    print("Angle completed in {} steps.".format(num_stops))

    while current_angle < relative_angle:
        # Rotate for step_angle
        current_angle += rotate_inner(velocity_publisher,
                                      speed, step_angle, clockwise)
        state_func(states.OBSERVING)
        rospy.sleep(sleep_duration)
        state_func(states.READY_FOR_GOAL)


def rotate_inner(velocity_publisher, speed, angle, clockwise=True):
    vel_msg = Twist()
    angular_speed = speed * 2 * math.pi / 360
    relative_angle = angle * 2 * math.pi / 360

    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = - \
        abs(angular_speed) if clockwise else abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print("Rotated for: {}".format(angle))

    return current_angle
