cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tools)

find_package(catkin REQUIRED 
  COMPONENTS roscpp rtt_ros cmake_modules)

catkin_package(
  DEPENDS  rtt  
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS  include
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})#

if(OROCOS_TARGET STREQUAL "xenomai")
  find_package(Xenomai REQUIRED)
  add_definitions(${Xenomai_POSIX_DEFINITIONS})
  include_directories(${Xenomai_POSIX_INCLUDE_DIRS})
  link_directories(${Xenomai_POSIX_LIBRARY_DIRS})
endif()

# Building a normal library (optional):
# Creates a library libsupport-<target>.so and installs it in
# lib/
orocos_library(${PROJECT_NAME}
  src/time.cpp
  src/params.cpp
  src/throttles.cpp) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


if(OROCOS_TARGET STREQUAL "xenomai")
  set_target_properties(${PROJECT_NAME}
    PROPERTIES LINK_FLAGS ${Xenomai_POSIX_LDFLAGS})
  target_link_libraries(${PROJECT_NAME}
    ${Xenomai_POSIX_LIBRARIES})
endif()

# Additional headers:
# Installs in the include/orocos/rtt_ros_tools/ directory
orocos_install_headers(
  include/rtt_ros_tools/tools.h 
  include/rtt_ros_tools/time.h 
  include/rtt_ros_tools/params.h 
  include/rtt_ros_tools/throttles.h 
  ) 

orocos_generate_package()

