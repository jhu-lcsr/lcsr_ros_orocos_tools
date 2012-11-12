#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(rtt_ros_tools)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Find the rosbuild_xenomai package
rosbuild_find_ros_package(rosbuild_xenomai)
include(${rosbuild_xenomai_PACKAGE_PATH}/rosbuild_xenomai.cmake)

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
  rosbuild_add_xenomai_posix_flags()
endif()

orocos_generate_package()
