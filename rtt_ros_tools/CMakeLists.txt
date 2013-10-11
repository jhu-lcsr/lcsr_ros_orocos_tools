cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tools)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(catkin REQUIRED 
  COMPONENTS roscpp rtt_rosnode)

catkin_package(
  DEPENDS  rtt  
  CATKIN_DEPENDS roscpp rtt_rosnode
  INCLUDE_DIRS  include
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})#

# Building a normal library (optional):
# Creates a library libsupport-<target>.so and installs it in
# lib/
orocos_library(${PROJECT_NAME}
  src/time.cpp
  src/params.cpp
  src/throttles.cpp) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Additional headers:
# Installs in the include/orocos/rtt_ros_tools/ directory
orocos_install_headers(
  include/rtt_ros_tools/tools.h 
  include/rtt_ros_tools/time.h 
  include/rtt_ros_tools/params.h 
  include/rtt_ros_tools/throttles.h 
  ) 

orocos_generate_package()

