cmake_minimum_required(VERSION 2.8.3)
project(exercise3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf
	rospy
)

find_package( OpenCV REQUIRED )

catkin_package()

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# add_executable(map_goals_cpp src/map_goals_cpp.cpp)
#target_link_libraries(map_goals_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS scripts/map_goals DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/rotating_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
