import rospy
from std_msgs.msg import Float32MultiArray

class HandManipulator(object):


    def __init__(self):
        self.STANDBY_POSITION = [-0.2, 2, -2, -1.5, -0.1, 1]
        self.STANDBY_POSITION_2 = [-0.2, 2, -1.5, -1.5, -0.1, 1]
        self.STANDBY_POSITION_3 = [-1.7, 2, -1.5, -1.5, -0.1, 1]
        self.STANDBY_POSITION_4 = [-1.7, 1.5, -1.8, -1.2, -0.1, 0.4]
        self.STANDBY_POSITION_4_GRIPPED = [-1.7, 1.5, -1.8, -1.2, -0.1, 1]


        self.DROP_POSITOION_CLOSED = [1.5, 0.5, -0.5, 0, 1.4, 1]
        self.DROP_POSITOION_OPEN = [1.5, 0.5, -0.5, 0, 1.4, 0.4]
        self.ARM_POSITION_3 = [0.2, 1.5, 0.4, 1.5, 0.4, 1]
        self.positon_publisher = rospy.Publisher("set_manipulator", Float32MultiArray, queue_size=10)



    def drop_coin(self):
        self.publish_data(self.DROP_POSITOION_CLOSED)
        self.publish_data(self.DROP_POSITOION_OPEN)

    def move_to_standby(self):
        print("Moving to position1")
        self.publish_data(self.STANDBY_POSITION)





    def grab_coin(self, index):
        coin_joint_positions = {
            0: -1.7,
            1: -1.45,
            2: -1.3
        }
        join_position = coin_joint_positions.get(index)


        print("Moving to grab_cpin position")
        self.publish_data(self.STANDBY_POSITION_2)
        self.publish_data([join_position, 2, -1.5, -1.5, -0.1, 0.4])
        self.publish_data([join_position, 1.5, -1.8, -1.2, -0.1, 0.4])
        self.publish_data([join_position, 1.5, -1.8, -1.2, -0.1, 1])
        self.publish_data([join_position, 2, -1.5, -1.5, -0.1, 1])






    def publish_data(self, data):
        print("----------Hand Manipulator Publish Data----------")
        print("Posting data: ", data)
        arr = Float32MultiArray()
        arr.data = data
        self.positon_publisher.publish(arr)
        rospy.sleep(2)

