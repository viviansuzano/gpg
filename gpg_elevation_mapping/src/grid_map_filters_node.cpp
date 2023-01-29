#include "gpg_elevation_mapping/GridMapFilters.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_filters_node");
  ros::NodeHandle nodeHandle("~");
  
  bool success;
  grid_map_filters::GridMapFilters gridMapFilters(nodeHandle, success);
  
  if (success) 
		ros::spin();
  
  return 0;
}
