# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/juliobotic/FestinoPumas/PC_user/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juliobotic/FestinoPumas/PC_user/build

# Utility rule file for robotino_msgs_generate_messages_cpp.

# Include the progress variables for this target.
include Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/progress.make

Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/AnalogReadings.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/DigitalReadings.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/MotorReadings.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/EncoderReadings.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/PowerReadings.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h


/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/AnalogReadings.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/AnalogReadings.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/AnalogReadings.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from robotino_msgs/AnalogReadings.msg"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/AnalogReadings.msg -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/DigitalReadings.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/DigitalReadings.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/DigitalReadings.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from robotino_msgs/DigitalReadings.msg"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/DigitalReadings.msg -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/MotorReadings.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/MotorReadings.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/MotorReadings.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from robotino_msgs/MotorReadings.msg"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/MotorReadings.msg -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/EncoderReadings.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/EncoderReadings.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/EncoderReadings.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from robotino_msgs/EncoderReadings.msg"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/EncoderReadings.msg -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/PowerReadings.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/PowerReadings.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/PowerReadings.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating C++ code from robotino_msgs/PowerReadings.msg"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg/PowerReadings.msg -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h: /opt/ros/melodic/share/gencpp/msg.h.template
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating C++ code from robotino_msgs/SetEncoderPosition.srv"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/SetEncoderPosition.srv -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h: /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h: /opt/ros/melodic/share/gencpp/msg.h.template
/home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/juliobotic/FestinoPumas/PC_user/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating C++ code from robotino_msgs/ResetOdometry.srv"
	cd /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs && /home/juliobotic/FestinoPumas/PC_user/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/srv/ResetOdometry.srv -Irobotino_msgs:/home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p robotino_msgs -o /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

robotino_msgs_generate_messages_cpp: Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/AnalogReadings.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/DigitalReadings.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/MotorReadings.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/EncoderReadings.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/PowerReadings.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/SetEncoderPosition.h
robotino_msgs_generate_messages_cpp: /home/juliobotic/FestinoPumas/PC_user/devel/include/robotino_msgs/ResetOdometry.h
robotino_msgs_generate_messages_cpp: Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/build.make

.PHONY : robotino_msgs_generate_messages_cpp

# Rule to build all files generated by this target.
Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/build: robotino_msgs_generate_messages_cpp

.PHONY : Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/build

Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/clean:
	cd /home/juliobotic/FestinoPumas/PC_user/build/Robotino_pkgs/robotino_msgs && $(CMAKE_COMMAND) -P CMakeFiles/robotino_msgs_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/clean

Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/depend:
	cd /home/juliobotic/FestinoPumas/PC_user/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juliobotic/FestinoPumas/PC_user/src /home/juliobotic/FestinoPumas/PC_user/src/Robotino_pkgs/robotino_msgs /home/juliobotic/FestinoPumas/PC_user/build /home/juliobotic/FestinoPumas/PC_user/build/Robotino_pkgs/robotino_msgs /home/juliobotic/FestinoPumas/PC_user/build/Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Robotino_pkgs/robotino_msgs/CMakeFiles/robotino_msgs_generate_messages_cpp.dir/depend

