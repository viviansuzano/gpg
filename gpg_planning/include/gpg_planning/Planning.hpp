#pragma once

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "gpg_planning/FootPositionVisualizer.h"
#include <vector>
#include <array>
#include <Eigen/Dense>

namespace footstep_planning {

class Planner{
    public:
        // Constructor
        Planner(ros::NodeHandle& nodeHandle, bool& success);

        // Destructor
        virtual ~Planner();

        // Lê e verifica os parâmetros do ROS
        bool readParameters();

        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void cmdCallback(const geometry_msgs::Twist& msg);
        void mapCallback(const grid_map_msgs::GridMap& msg);

        bool updateFootMarkers();
        bool updateNextFootMarkers();


    private:

        ros::NodeHandle& node_handle;

        ros::Subscriber pose_subscriber;

        ros::Subscriber cmd_subscriber;

        ros::Subscriber map_subscriber;

        std::string pose_topic;
        std::string cmd_topic;
        std::string map_topic;
        double base_to_hip_x;
        double base_to_hip_y;
        double robot_height;
        double t_stance;
        double k_velocity;
        double gravity;
        double threshold_traversability;

        enum FootIDs {FL=0, FR, RL, RR, nextFL, nextFR, nextRL, nextRR};

        footstep_marker::FootPositionVisualizer footPosVisualizer;
        footstep_marker::FootPositionVisualizer nextFootPosVisualizer;
        std::vector<Eigen::Vector3d> footPositions;
        std::vector<Eigen::Vector3d> nextFootPositions;

        tf::Transform w_transformFootPos_b;

        tf::Vector3 b_baseTwist_linear;
        tf::Vector3 b_baseTwist_angular;

        tf::Vector3 b_cmdVel_linear;
        tf::Vector3 b_cmdVel_angular;

        std::array<tf::Vector3, 4> b_pos;
        std::array<tf::Vector3, 8> w_pos;

        std::array<double, 8> elevation;
        std::array<double, 8> traversability;
        std::array<u_int, 4> binary_t;
        std::array<u_int, 4> nextBinary_t;
};

} /* namespace*/