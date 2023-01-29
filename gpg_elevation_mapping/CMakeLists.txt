cmake_minimum_required(VERSION 2.8.3)
project(gpg_elevation_mapping)

find_package(catkin REQUIRED
  COMPONENTS
    elevation_mapping
		grid_map_filters
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS elevation_mapping
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_map_filters
  src/grid_map_filters_node.cpp
  src/GridMapFilters.cpp
)
target_link_libraries(
  grid_map_filters
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
