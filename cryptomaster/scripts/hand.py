import rospy
from std_msgs.msg import Float32MultiArray
from openservorobot.msg import ManipulatorDescriptionM



class HandManipulator(object):

    def __init__(self,joint_minimums=(0,0,0,0,0,0)):


        desc = rospy.wait_for_message("openservorobot/manipulator_description",ManipulatorDescriptionM)


        self.mappings = {}
        for joint_index, joint in enumerate(desc.joints):
            print(joint)
            self.mappings[joint_index]['min'] = joint.dh_min
            self.mappings[joint_index]['max'] = joint.dh_max

        print(self.mappings)



        self.mins = joint_minimums
        ## Grip values
        self.GRIP_OPEN_VALUE = 0
        self.GRIP_CLOSED_VALUE = 1

        ## Coin join value
        self.COIN_1_JOINT_POSITION = 0.24

        ## Hand positions
        # Standby
        self.STANDBY_POSITION =         [0.2, 0.8, 0.5, 0.13, 0.8, self.GRIP_CLOSED_VALUE]

        # Grabing positions
        self.ABOVE_COIN_OPEN =          [0.24, 0.8, 0.7, 0.13, 0.8, self.GRIP_OPEN_VALUE]
        self.GRAB_POSITION =            [0.24, 0.7, 0.7, 0.2, 0.8, self.GRIP_OPEN_VALUE]
        self.GRABBED_POSITION =         [0.24, 0.7, 0.7, 0.2, 0.8, self.GRIP_CLOSED_VALUE]
        self.ABOVE_COIN_CLOSED =        [0.24, 0.8, 0.7, 0.13, 0.8, self.GRIP_CLOSED_VALUE]

        # Dropping positions
        self.DROP_POSITION_CLOSED =     [0.99, 0.75, 0.8, 0, 0.8, self.GRIP_CLOSED_VALUE]
        self.DROP_POSITION_OPEN =       [0.99, 0.75, 0.8, 0, 0.8, self.GRIP_OPEN_VALUE]

        ## Position publisher
        self.position_publisher = rospy.Publisher("set_manipulator", Float32MultiArray, queue_size=10)



    def map_to_bounds(self, joint_index, value):

        joint_values = self.mappings[joint_index]

        min = joint_values['min']
        max = joint_values['max']

        interval_range = max - min

        return (value * interval_range) + min


    
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
        new_data = []

        for value, joint_index in data:
            new_data.append(self.map_to_bounds(joint_index, value))

        print("----------Hand Manipulator Publish Data----------")
        print("Posting data: ", new_data)
        arr = Float32MultiArray()
        arr.data = new_data
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
