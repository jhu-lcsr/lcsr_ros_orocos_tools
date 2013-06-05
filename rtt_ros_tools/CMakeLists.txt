# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tools)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp rtt rtt_rosnode)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})#

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

find_package(catkin COMPONENTS xenomai_ros)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Additional headers:
# Installs in the include/orocos/rtt_ros_tools/ directory
orocos_install_headers(
  include/rtt_ros_tools/tools.h 
  include/rtt_ros_tools/time.h 
  include/rtt_ros_tools/params.h 
  include/rtt_ros_tools/throttles.h 
  ) 

# Building a normal library (optional):
# Creates a library libsupport-<target>.so and installs it in
# lib/
orocos_library(rtt_ros_tools
  src/time.cpp
  src/params.cpp
  src/throttles.cpp) 
if($ENV{OROCOS_TARGET} STREQUAL "xenomai") 
  add_xenomai_posix_flags()
endif()

orocos_generate_package()

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rtt rtt_rosnode
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
