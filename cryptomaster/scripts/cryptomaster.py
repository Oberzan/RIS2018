#!/usr/bin/env python
import sys

import roslib

roslib.load_manifest('cryptomaster')
import numpy as np
import rospy
from path_generator import GoalGenerator
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
from util import flip, nearest_goal, point_2_base_goal, point_distance, quaternion_between, get_approached_viewpoint, \
    rotate_quaternion
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Twist
import states as states
from constants import GOAL_RESULT_TIMEOUT, ACTION_CLIENT_STATUSES, ROTATE_ANGLE, ROTATE_SPEED, NUM_CIRCLES_TO_DETECT
from moves import rotate
from cluster import Clusterer
from std_msgs.msg import String
from std_msgs.msg import Int8
from hand import HandManipulator
from tf.transformations import quaternion_from_euler
from openservorobot.msg import ManipulatorDescriptionM


class CryptoMaster(object):

    def __init__(self):
        rospy.init_node('cryptomaster')
        self.state = states.WAITING_FOR_MAP
        self.action_client = SimpleActionClient("move_base", MoveBaseAction)
        self.action_client.wait_for_server()
        self.goal_generator = GoalGenerator(rospy.get_param('~img'), erosion_factor=rospy.get_param('~erosion'),
                                            goal_step=rospy.get_param('~step'))

        
        self.hand_manipulator = HandManipulator()
        self.clusterer = Clusterer(min_center_detections=15)
        self.cv_map = None
        self.map_transform = None
        self.map_resolution = 0
        self.viewpoints = None
        self.goals_left = None
        self.robot_location = Point(100.0, 15.0, 0.0)
        self.circles_detected = 0
        self.coins_dropped = 0

        self.state_handlers = {
            states.READY_FOR_GOAL: self.ready_for_goal_state_handler,
            states.WAITING_FOR_MAP: self.map_state_handler
        }

        _ = rospy.Subscriber(
            "map", OccupancyGrid, self.map_callback)
        self.arm_publisher = rospy.Publisher(
            "set_manipulator_position", Int8, queue_size=10)
        self.state_publisher = rospy.Publisher(
            "engine/status", String, queue_size=10)
        self.velocity_publisher = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.speaker_publisher = rospy.Publisher(
            "speaker/say", String, queue_size=10)
        self.engine_state_publisher = rospy.Publisher(
            "engine/status", String, queue_size=10)

    def map_state_handler(self):
        print("Waiting for map_callback...")

    def change_state(self, state):
        self.state = state
        string_message = String()
        string_message.data = self.state
        self.clusterer.change_state(state)
        self.engine_state_publisher.publish(string_message)

    def run_robot(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            state_handler = self.state_handlers.get(self.state)

            if state_handler:
                state_handler()
            else:
                print("UNKNOWN STATE: ", self.state)

            while self.clusterer.has_pending_jobs():
                if self.circles_detected == NUM_CIRCLES_TO_DETECT:
                    break

                circle_target = self.clusterer.get_next_job()
                self.handle_cluster_job(circle_target)

            if self.circles_detected == NUM_CIRCLES_TO_DETECT:
                self.say("Die puny humans.")
                break

            rate.sleep()

        print("Execution finished")

    def ready_for_goal_state_handler(self):
        print("--------Ready For Goal State Handler--------")
        new_goal, _ = nearest_goal(self.robot_location, self.goals_left)
        print("Got new goal: ", new_goal)
        self.goals_left.remove(new_goal)
        print(len(self.goals_left), " goals left.")

        move_status_result = self.move_to_point(new_goal)

        if move_status_result == 'SUCCEEDED':
            self.change_state(states.OBSERVING)
            rotate(self.velocity_publisher, ROTATE_SPEED, ROTATE_ANGLE)
            self.change_state(states.READY_FOR_GOAL)

    def move_to_point(self, goal, quaternion=None):
        print("--------Moving To Point--------")
        move_base_goal = point_2_base_goal(goal, orientation=quaternion)
        self.action_client.send_goal(move_base_goal)
        self.action_client.wait_for_result(
            rospy.Duration(GOAL_RESULT_TIMEOUT))

        status = ACTION_CLIENT_STATUSES[self.action_client.get_state()]
        print("Action result: ", status)
        self.robot_location = goal
        return status

    def cylinder_approached_handler(self):
        print("--------Cylinder Approached Handle--------")
        self.hand_manipulator.grab_coin(self.coins_dropped)
        self.hand_manipulator.drop_coin()
        self.say("Coin thrown in!", 1)
        self.coins_dropped += 1
        self.state = states.READY_FOR_GOAL

    def handle_cluster_job(self, target):
        print("--------Handle Cluster Job--------")
        nearest_viewpoints = self.find_nearest_viewpoints(
            target, self.robot_location)

        succeded = False
        viewpoint_ix = 0

        while not succeded and viewpoint_ix < len(nearest_viewpoints):
            print("Trying viewpoint with index: ", viewpoint_ix)
            nearest_viewpoint = nearest_viewpoints[viewpoint_ix]
            approached_target = get_approached_viewpoint(
                nearest_viewpoint, target, 0.45)

            print("Aproached target: ", approached_target)

            quaternion_ros, q = quaternion_between(
                target, approached_target)

            approach_status = self.move_to_point(approached_target, quaternion=quaternion_ros)

            if approach_status == 'SUCCEEDED':
                succeded = True
                _, cluster_ix = self.clusterer.find_nearest_cluster(target)
                self.clusterer.reset_cluster(cluster_ix)

                self.change_state(states.OBSERVING)
                rospy.sleep(5)
                self.change_state(states.CIRCLE_APPROACHED)

                improved_cluster = self.clusterer.centers[cluster_ix]
                print(improved_cluster)

                _, rotated_quat = rotate_quaternion(q, 90)

                approached_target = get_approached_viewpoint(
                    approached_target, improved_cluster, 0.305)

                self.move_to_point(approached_target, quaternion=rotated_quat)

            viewpoint_ix += 1

        print("Moved to approached target!")
        self.state = states.CIRCLE_APPROACHED

        self.say("Cylinder detected", 3)
        self.circles_detected += 1
        self.cylinder_approached_handler()

    def find_nearest_viewpoints(self, circle_target, robot_location):
        print("--------Circle Goal Viewpoint--------")
        distance_to_circle = point_distance(circle_target, robot_location)

        candidate_viewpoints = [p for p in self.viewpoints if point_distance(
            p, robot_location) <= distance_to_circle]
        print("Filtered to : ", len(candidate_viewpoints), " points")

        print("Got request for nearest point to circle: {}".format(circle_target))


        by_dist = sorted(candidate_viewpoints, key=lambda goal: point_distance(goal, circle_target))

        print(by_dist)

        return by_dist

    def map_callback(self, data):
        print("--------Map callback--------")
        size_x = data.info.width
        size_y = data.info.height
        self.cv_map = np.zeros(shape=(size_y, size_x))

        if size_x < 3 or size_y < 3:
            print("Map size is only x: {}, y: {}. Not running map to image conversion.".format(
                size_x, size_y))

        rows, columns = self.cv_map.shape

        if rows != size_y and columns != size_x:
            self.cv_map = np.array([size_y, size_x])

        self.map_resolution = data.info.resolution
        self.map_transform = data.info.origin

        grid = flip(np.reshape(data.data, (size_y, size_x)), 0)

        for i in range(size_y):
            for j in range(size_x):
                if grid[i][j] == -1:
                    self.cv_map[i][j] = 127
                elif grid[i][j] == 100:
                    self.cv_map[i][j] = 0
                elif grid[i][j] == 0:
                    self.cv_map[i][j] = 255
                else:
                    print('Error at i:' + str(grid[i][j]))

        print("Map successfully saved.")

        pixel_goals = self.goal_generator.generate_points()
        self.viewpoints = [self.transform_map_point(p) for p in pixel_goals]
        self.goals_left = self.viewpoints[:]
        print("Transformed goals to map coordinates")
        print("Viewpoints: ", self.viewpoints)
        self.state = states.READY_FOR_GOAL

    def transform_map_point(self, point):
        _, size_y = self.cv_map.shape
        transformed = Point(point.x * self.map_resolution + self.map_transform.position.x,
                            (size_y - point.y) * self.map_resolution + (
                                self.map_transform.position.y * self.map_resolution * 2), 0)
        return transformed

    def say(self, data, sleep_duration=1):
        print("Saying: ", data)
        say = String()
        say.data = data
        self.speaker_publisher.publish(say)
        rospy.sleep(sleep_duration)


def main(args):
    crypto_robot = CryptoMaster()
    crypto_robot.hand_manipulator.move_to_standby()
    crypto_robot.run_robot()
    # crypto_robot.hand_manipulator.grab_coin(0)
    # crypto_robot.hand_manipulator.drop_coin()
    # crypto_robot.hand_manipulator.grab_coin(1)
    # crypto_robot.hand_manipulator.drop_coin()
    # crypto_robot.hand_manipulator.grab_coin(2)
    # crypto_robot.hand_manipulator.drop_coin()

if __name__ == '__main__':
    main(sys.argv)