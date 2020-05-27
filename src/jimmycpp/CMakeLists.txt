# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package (must match name in package.xml).
project(jimmycpp)

#use the C++11 standard
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# Declare our catkin package.
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
  LIBRARIES
	jimmycpp
)

# Specify locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(jimmycpp
	include/${PROJECT_NAME}/arduinomega.h
    include/${PROJECT_NAME}/ampsteppers.h
    include/${PROJECT_NAME}/trc_logging.h
    include/${PROJECT_NAME}/lidar_magnetic_encoder.h
    include/${PROJECT_NAME}/lidar_stepper_motion.h
   src/arduinomega.cpp
   src/ampsteppers.cpp
   src/trc_logging.cpp
   src/lidar_magnetic_encoder.cpp
   src/lidar_stepper_motion.cpp
)

#wow, this was not a fun one. Here goes:
#lidar_rotation.cpp & .h were written with libserial.so version 0.6; and 
#lidar_magnetic_encoder.cpp&.h and lidar_stepper_motion.cpp&.h were written with
#libserial.so version 1.0. It so happens that those versions are quite different.
#Ver 0.6 is in /usr/lib/x86_64-linux-gnu/libserial.so.0.0.0; and version 1.0 is
#in /usr/local/lib/libserial.so.1.0.0. the line below means the following:
# jimmycpp - this is called the 'target' (executable code), and this target 
# relies on libraries (that it called).
# /usr/local/lib/libserial.so - this is the libserial library ver 1.0. and it is
# needed by lidar_magnetic_encoder.cpp &.h and lidar_stepper_motion.cpp&.h; I am 
# not sure why I had to specify the path, but I had to if I wanted it to work.
# serial - this is needed by lidar_rotation.cpp and this is the libserial ver 0.6.
# In this case I did not need to specify the path to the library.
#labjackusb - is the exodriver for the U3, and is needed by lidar_encoder.cpp&.h 
#see: https://answers.ros.org/question/274451/how-to-utilize-so-file-in-cmakeliststxt/
#see: https://cmake.org/pipermail/cmake/2011-May/044295.html
#see: https://cmake.org/cmake/help/v3.16/command/target_link_libraries.html
#commented out during melodic adoption in new PC (W520)
#target_link_libraries(jimmycpp /usr/local/lib/libserial.so serial labjackusb ${catkin_LIBRARIES})
target_link_libraries(jimmycpp serial ${catkin_LIBRARIES})
