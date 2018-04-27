import rospy
from std_msgs.msg import Float32MultiArray

class HandManipulator(object):


    def __init__(self):
        self.ARM_POSITION_1 = [-1.7, 2.5, -2.5, -1.5, -0.1, 0.4]
        self.ARM_POSITION_2 = [1, 0.5, 0.1, 0.5, 1.4, 0]
        self.ARM_POSITION_3 = [0.2, 1.5, 0.4, 1.5, 0.4, 1]
        self.positon_publisher = rospy.Publisher("set_manipulator", Float32MultiArray, queue_size=10)


    def move_to_position1(self):
        print("Moving to position1")
        self.publish_data(self.ARM_POSITION_1)

    def move_to_position2(self):
        print("Moving to position2")

        self.publish_data(self.ARM_POSITION_1)

    def move_to_position3(self):
        print("Moving to position3")
        self.publish_data(self.ARM_POSITION_1)


    def publish_data(self, data):
        print("----------Hand Manipulator Publish Data----------")
        print("Posting data: ", data)
        arr = Float32MultiArray()
        arr.data = data
        self.positon_publisher.publish(arr)

