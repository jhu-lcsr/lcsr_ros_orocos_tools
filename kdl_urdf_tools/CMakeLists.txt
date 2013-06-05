# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(kdl_urdf_tools)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp orocos_kdl kdl_parser std_msgs sensor_msgs geometry_msgs trajectory_msgs tf tf_conversions angles)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})#

add_library(kdl_urdf_tools src/tools.cpp)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp orocos_kdl kdl_parser std_msgs sensor_msgs geometry_msgs trajectory_msgs tf tf_conversions angles
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES kdl_urdf_tools
)
