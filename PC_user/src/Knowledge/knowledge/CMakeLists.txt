cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
 	visualization_msgs
	interactive_markers
	tf
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  	DEPENDENCIES
  	std_msgs
	geometry_msgs
)

catkin_package(
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

