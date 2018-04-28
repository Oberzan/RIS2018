import rospy
from std_msgs.msg import Float32MultiArray


class HandManipulator(object):

    def __init__(self):
        ## Grip values
        self.GRIP_OPEN_VALUE = 0.4
        self.GRIP_CLOSED_VALUE = 1.0

        ## Coin join value
        self.COIN_1_JOINT_POSITION = -1.7

        ## Hand positions
        # Standby
        self.STANDBY_POSITION = [-0.2, 2, -2, -1.5, -0.1, self.GRIP_CLOSED_VALUE]
        self.STANDBY_POSITION_UPPER = [-0.2, 2, -1.5, -1.5, -0.1, self.GRIP_CLOSED_VALUE]

        # Grabing positions
        self.GRAB_POSITION_UPPER = [-1.7, 2, -1.5, -1.5, -0.1, self.GRIP_OPEN_VALUE]
        self.GRAB_POSITION = [-1.7, 1.5, -1.8, -1.2, -0.1, self.GRIP_OPEN_VALUE]
        self.COIN_GRABBED_POSITION = [-1.7, 1.5, -1.8, -1.2, -0.1, self.GRIP_CLOSED_VALUE]

        # Dropping positions
        self.DROP_POSITOION_CLOSED = [1.5, 0.5, -0.5, 0, 1.4, self.GRIP_CLOSED_VALUE]
        self.DROP_POSITOION_OPEN = [1.5, 0.5, -0.5, 0, 1.4, self.GRIP_OPEN_VALUE]

        ## Position publisher
        self.position_publisher = rospy.Publisher("set_manipulator", Float32MultiArray, queue_size=10)

    def drop_coin(self):
        self.move_arm_to(self.DROP_POSITOION_CLOSED)
        self.move_arm_to(self.DROP_POSITOION_OPEN)
        self.move_to_standby()

    def move_to_standby(self):
        self.move_arm_to(self.STANDBY_POSITION)

    def grab_coin(self, index):
        coin_joint_positions = {
            0: self.COIN_1_JOINT_POSITION,
            1: self.COIN_1_JOINT_POSITION + 0.25,
            2: self.COIN_1_JOINT_POSITION + 0.5
        }

        joint_position_value = coin_joint_positions.get(index)

        print("Moving to grab coin position")
        self.move_arm_to(self.STANDBY_POSITION_UPPER)
        self.move_arm_to(self.with_coin_value(self.GRAB_POSITION_UPPER, joint_position_value))
        self.move_arm_to(self.with_coin_value(self.GRAB_POSITION, joint_position_value))
        self.move_arm_to(self.with_coin_value(self.COIN_GRABBED_POSITION, joint_position_value))
        self.move_arm_to(self.with_coin_value(self.GRAB_POSITION_UPPER, joint_position_value))
        print("Coin grabbed")

    def move_arm_to(self, data, sleep_duration=2):
        print("----------Hand Manipulator Publish Data----------")
        print("Posting data: ", data)
        arr = Float32MultiArray()
        arr.data = data
        self.position_publisher.publish(arr)
        rospy.sleep(sleep_duration)

    def with_coin_value(self, position, joint_value):
        l = position[:]
        l[0] = joint_value
        return l

    def _with_open_hand(self, position):
        return self._with_hand_grip(position, self.GRIP_OPEN_VALUE)

    def with_closed_hand(self, position):
        return self._with_hand_grip(position, self.GRIP_CLOSED_VALUE)

    def _with_hand_grip(self, position, grip_value):
        l = position[:]
        l[5] = grip_value
        return l
