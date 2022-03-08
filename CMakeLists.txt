cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############
catkin_package(
CATKIN_DEPENDS
	geometry_msgs
	rospy
	nav_msgs
	tf)



catkin_install_python(PROGRAMS
   src/controller.py

   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


