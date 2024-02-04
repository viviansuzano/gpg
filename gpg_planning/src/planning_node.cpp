#include <ros/ros.h>
#include "gpg_planning/Planning.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh("~");

    bool success;
    footstep_planning::Planner footStepsPlanner(nh, success);

    if (success)
        ros::spin();

    return 0;
}
