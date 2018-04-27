from geometry_msgs.msg import Point
import numpy as np
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
import rospy
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_about_axis


def point_distance(p1, p2):
    return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5


def nearest_goal(point, goals):
    max_dist = 999999999
    c_goal = None
    for p in goals:
        dist = point_distance(point, p)
        if dist < max_dist:
            c_goal = p
            max_dist = dist

    return (c_goal, max_dist)


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


def point_2_base_goal(point, frame_id="map", orientation=None):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    if orientation is not None:
        goal.pose.orientation = orientation
    else:
        goal.pose.orientation.w = 1
    goal.pose.position = point
    goal.header.stamp = rospy.Time(0)
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose = goal
    return move_base_goal


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def quaternion_between(target, viewpoint):
    angle = angle_between((
        target.x-viewpoint.x,
        target.y-viewpoint.y, 0), (1, 0, 0))
    if target.y < viewpoint.y:
        angle = -angle

    quaternion = quaternion_about_axis(angle, (0, 0, 1))
    quaternion_ros, quaternion = Quaternion(
        quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    return quaternion_ros, quaternion


def get_approached_viewpoint(viewpoint, target, threshold=0.2):
    vector_to_target = Point(target.x - viewpoint.x, target.y - viewpoint.y, 0)
    dist = point_distance(viewpoint, target)
    k = dist / threshold
    return Point(target.x - vector_to_target.x / k, target.y - vector_to_target.y / k, 0)
