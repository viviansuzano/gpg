cmake_minimum_required(VERSION 2.8.3)
project(gpg_gazebo)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    gazebo_ros
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
