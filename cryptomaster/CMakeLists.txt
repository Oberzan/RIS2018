cmake_minimum_required(VERSION 2.8.3)
project(cryptomaster)

find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	std_msgs
	geometry_msgs
	tf
	rospy
)

find_package( OpenCV REQUIRED )

catkin_package()

catkin_install_python(PROGRAMS scripts/cryptomaster.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
