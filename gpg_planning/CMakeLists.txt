cmake_minimum_required(VERSION 2.8.3)
project(gpg_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Build
################################################################################
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpg_goal_publisher src/gpg_goal_publisher.cpp)
add_dependencies(gpg_goal_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpg_goal_publisher ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS gpg_goal_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
