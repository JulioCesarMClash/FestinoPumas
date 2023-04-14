cmake_minimum_required(VERSION 2.8.3)
project(tug_cfg_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tug_cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################
## Declare tug_cfg parameters ##
################################

tug_cfg_generate_cpp(
  cfg/ExampleConfig.yaml
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tug_cfg_example
#  CATKIN_DEPENDS roscpp tug_cfg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional compile options for this package
add_definitions(-std=c++11 -Wformat -Wall -Wextra -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/example_node.cpp include/${PROJECT_NAME}/example_node.h
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_tug_cfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_test_example_config
    test/test_example_config.cpp
    test/test_helpers.cpp include/${PROJECT_NAME}/test_helpers.h
  )

  add_dependencies(${PROJECT_NAME}_test_example_config
    ${PROJECT_NAME}_tug_cfg
  )

  target_link_libraries(${PROJECT_NAME}_test_example_config
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(${PROJECT_NAME}_test_ros_param_reader
    test/test_ros_param_reader.test
    test/test_ros_param_reader.cpp
    test/test_helpers.cpp include/${PROJECT_NAME}/test_helpers.h
  )

  add_dependencies(${PROJECT_NAME}_test_ros_param_reader
    ${PROJECT_NAME}_tug_cfg
  )

  target_link_libraries(${PROJECT_NAME}_test_ros_param_reader
    ${catkin_LIBRARIES}
  )

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
  roslint_cpp()
  roslint_add_test()
endif()
