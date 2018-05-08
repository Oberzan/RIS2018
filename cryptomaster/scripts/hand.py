import rospy
from std_msgs.msg import Float32MultiArray


class HandManipulator(object):

    def __init__(self,joint_minimums=(0,0,0,0,0,0)):
        self.mins = joint_minimums
        ## Grip values
        self.GRIP_OPEN_VALUE = 0.8
        self.GRIP_CLOSED_VALUE = 1.5

        ## Coin join value
        self.COIN_1_JOINT_POSITION = 0.24

        ## Hand positions
        # Standby
        self.STANDBY_POSITION =         [1.74, 2.25, 0.5, 0.13, 1.68, self.GRIP_CLOSED_VALUE]

        # Grabing positions
        self.ABOVE_COIN_OPEN =          [0.24, 2.25, 1.3, 0.13, 1.68, self.GRIP_OPEN_VALUE]
        self.GRAB_POSITION =            [0.24, 1.85, 0.7, -1.1, 1.68, self.GRIP_OPEN_VALUE]
        self.GRABBED_POSITION =         [0.24, 1.85, 0.7, -1.1, 1.68, self.GRIP_CLOSED_VALUE]
        self.ABOVE_COIN_CLOSED =        [0.24, 2.25, 1.3, 0.13, 1.68, self.GRIP_CLOSED_VALUE]

        # Dropping positions
        self.DROP_POSITION_CLOSED =     [3.44, 0.75, 2, 0, 1.68, self.GRIP_CLOSED_VALUE]
        self.DROP_POSITION_OPEN =       [3.44, 0.75, 2, 0, 1.68, self.GRIP_OPEN_VALUE]

        ## Position publisher
        self.position_publisher = rospy.Publisher("set_manipulator", Float32MultiArray, queue_size=10)
    
    def sum(self,positions):
        return [x + y for x,y in zip(self.mins,positions)]
        

    def drop_coin(self):
        self.move_arm_to(self.DROP_POSITION_CLOSED)
        self.move_arm_to(self.DROP_POSITION_OPEN)
        self.move_to_standby()

    def move_to_standby(self):
        self.move_arm_to(self.STANDBY_POSITION)

    def grab_coin(self, index):
        coin_joint_positions = {
            0: self.COIN_1_JOINT_POSITION + 1.0,
            1: self.COIN_1_JOINT_POSITION + 0.55,
            2: self.COIN_1_JOINT_POSITION
        }

        joint_position_value = coin_joint_positions.get(index)

        print("Moving to grab coin position")
        self.move_arm_to(self.with_coin_value(self.ABOVE_COIN_OPEN, joint_position_value))
        self.move_arm_to(self.with_coin_value(self.GRAB_POSITION, joint_position_value))
        self.move_arm_to(self.with_coin_value(self.GRABBED_POSITION,        joint_position_value))
        self.move_arm_to(self.with_coin_value(self.ABOVE_COIN_CLOSED, joint_position_value))
        print("Coin grabbed")

    def move_arm_to(self, data, sleep_duration=2):
        print("----------Hand Manipulator Publish Data----------")
        print("Posting data: ", self.sum(data))
        arr = Float32MultiArray()
        arr.data = self.sum(data)
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
