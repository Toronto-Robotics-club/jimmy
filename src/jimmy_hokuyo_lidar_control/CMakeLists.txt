# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package (must match package.xml).
# from here on "${PROJECT_NAME}" == "jimmy_lidar_motor_control"
project(jimmy_hokuyo_lidar_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin 
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages. 
# I added: roscpp, rospy, std_msgs, message_generation, and jimmycpp. All this
# is out of the manual_operation package (used to initially test jimmycpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  jimmycpp
  message_generation
)

#declare any custom files that I have created
add_message_files(
  FILES
  lidar_message.msg
)

#the custom message I am declaring relies on data types found here
generate_messages(DEPENDENCIES std_msgs )

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
  INCLUDE_DIRS include
#  LIBRARIES my_robot_base
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files. If there are multiple
# executables, create multiples of this line. NOTE: executable name and source
# file name should be the same!!!!
add_executable(lidar_system_control src/lidar_system_control.cpp)
# used to be: add_executable(${PROJECT_NAME}_node src/lidar_motor_control.cpp) #but caused an issue


# Specify libraries against which to link. This line should be copied for each
# distinct executable in the package (where executable names should be the same
# as source names). The library it needs is jimmycpp.
target_link_libraries(lidar_system_control ${catkin_LIBRARIES})
# used to be: target_link_libraries(${PROJECT_NAME}_node serial ${catkin_LIBRARIES}) #but caused an issue

# this line is to be used anytime I am using custom messages. It prevents build 
# errors when I run `catkin_make`. The first argument is the same name as the 
# name of the source file. The second argument is the name of the package plus 
# add “_generate_messages_cpp”. This solution to my problem is described on 
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B) in section
#topline used to be: add_dependencies(${PROJECT_NAME}_node #but caused an issue.
add_dependencies(lidar_system_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  jimmy_hokuyo_lidar_control_generate_messages_cpp
)

