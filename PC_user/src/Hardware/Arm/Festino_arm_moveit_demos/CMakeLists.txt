cmake_minimum_required(VERSION 3.0.2)
project(Festino_arm_moveit_demos)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  rospy
  tf2_geometry_msgs
  std_msgs
  message_generation
  geometry_msgs
)

add_service_files(
  FILES
  arm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    tf2_geometry_msgs
    message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(pick_place src/pick_place.cpp)
add_executable(prueba1 src/prueba1.cpp)
add_executable(pruebaSrv src/pruebaSrv.cpp)

target_link_libraries(pick_place ${catkin_LIBRARIES})
target_link_libraries(prueba1 ${catkin_LIBRARIES})
target_link_libraries(pruebaSrv ${catkin_LIBRARIES})
