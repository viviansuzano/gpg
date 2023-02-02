cmake_minimum_required(VERSION 2.8.3)
project(gpg_localization)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
    nav_msgs
    geometry_msgs
    tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_robot_pose
  src/publish_robot_pose.cpp
)
target_link_libraries(publish_robot_pose
  ${catkin_LIBRARIES}
)
add_dependencies(publish_robot_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

