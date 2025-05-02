cmake_minimum_required(VERSION 3.0.2)
project(vision_pca)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# Add standalone PCL dependency
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_pca
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # Added PCL includes
)

add_executable(grasping_node src/grasping_node.cpp)  # Fixed typo

target_link_libraries(grasping_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}  # Added PCL libraries
)

add_dependencies(grasping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})