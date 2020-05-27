# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package (must match package.xml).
project(jimmy_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find the catkin build system, and any other packages on which we depend. In
# this case we only need the roscpp library.
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
)

# Declare our catkin package.
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp 
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files. If there are multiple
# executables, create multiples of this line. NOTE: executable name and source
# file name should be the same (not a must but a good practice)!!!!
add_executable(gazebo_lidar_motion src/gazebo_lidar_motion.cpp)

# Specify libraries against which to link. This line should be copied for each
# distinct executable in the package (where executable names should be the same
# as source names).
target_link_libraries(gazebo_lidar_motion ${catkin_LIBRARIES})
