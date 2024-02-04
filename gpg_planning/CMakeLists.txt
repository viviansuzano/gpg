cmake_minimum_required(VERSION 2.8.3)
project(gpg_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  grid_map_core
  grid_map_msgs
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf grid_map_core grid_map_msgs grid_map_ros
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gpg_goal_publisher src/gpg_goal_publisher.cpp)
add_dependencies(gpg_goal_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpg_goal_publisher ${catkin_LIBRARIES})

add_executable(planning_node src/planning_node.cpp src/planning.cpp)
target_link_libraries(planning_node ${catkin_LIBRARIES})


################################################################################
# Install
################################################################################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
