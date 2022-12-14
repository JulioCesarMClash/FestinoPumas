# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotino_msgs: 5 messages, 2 services")

set(MSG_I_FLAGS "-Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotino_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" ""
)

get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_custom_target(_robotino_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotino_msgs" "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)

### Generating Services
_generate_srv_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)
_generate_srv_cpp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
)

### Generating Module File
_generate_module_cpp(robotino_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotino_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotino_msgs_generate_messages robotino_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_cpp _robotino_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotino_msgs_gencpp)
add_dependencies(robotino_msgs_gencpp robotino_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotino_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)
_generate_msg_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)
_generate_msg_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)
_generate_msg_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)
_generate_msg_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)

### Generating Services
_generate_srv_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)
_generate_srv_eus(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
)

### Generating Module File
_generate_module_eus(robotino_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotino_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotino_msgs_generate_messages robotino_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_eus _robotino_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotino_msgs_geneus)
add_dependencies(robotino_msgs_geneus robotino_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotino_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)
_generate_msg_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)

### Generating Services
_generate_srv_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)
_generate_srv_lisp(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
)

### Generating Module File
_generate_module_lisp(robotino_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotino_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotino_msgs_generate_messages robotino_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_lisp _robotino_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotino_msgs_genlisp)
add_dependencies(robotino_msgs_genlisp robotino_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotino_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)
_generate_msg_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)
_generate_msg_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)
_generate_msg_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)
_generate_msg_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)

### Generating Services
_generate_srv_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)
_generate_srv_nodejs(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
)

### Generating Module File
_generate_module_nodejs(robotino_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotino_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotino_msgs_generate_messages robotino_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_nodejs _robotino_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotino_msgs_gennodejs)
add_dependencies(robotino_msgs_gennodejs robotino_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotino_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)
_generate_msg_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)
_generate_msg_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)
_generate_msg_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)
_generate_msg_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)

### Generating Services
_generate_srv_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)
_generate_srv_py(robotino_msgs
  "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
)

### Generating Module File
_generate_module_py(robotino_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotino_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotino_msgs_generate_messages robotino_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv" NAME_WE)
add_dependencies(robotino_msgs_generate_messages_py _robotino_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotino_msgs_genpy)
add_dependencies(robotino_msgs_genpy robotino_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotino_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotino_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotino_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotino_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotino_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotino_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotino_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotino_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotino_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotino_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotino_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
