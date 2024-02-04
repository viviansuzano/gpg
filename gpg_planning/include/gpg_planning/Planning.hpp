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
        void footCallback(const visualization_msgs::Marker& msg);
        void mapCallback(const grid_map_msgs::GridMap& msg);

        bool updateFootMarkers();

        bool updateNextFootMarkers();


    private:

        ros::NodeHandle& node_handle;

        ros::Subscriber pose_subscriber;

        ros::Subscriber cmd_subscriber;

        ros::Subscriber footFL_subscriber;
        ros::Subscriber footFR_subscriber;
        ros::Subscriber footRL_subscriber;
        ros::Subscriber footRR_subscriber;

        ros::Subscriber map_subscriber;


        std::string pose_topic;
        std::string cmd_topic;
        std::string map_topic;
        std::string foot_topic;
        std::string next_foot_topic;
        double base_to_hip_x;
        double base_to_hip_y;
        double robot_height;
        double t_stance;
        double k_velocity;
        double gravity;

        tf::Vector3 w_posFL;
        tf::Vector3 w_posFR;
        tf::Vector3 w_posRL;
        tf::Vector3 w_posRR;

        tf::Vector3 w_nextPosFL;
        tf::Vector3 w_nextPosFR;
        tf::Vector3 w_nextPosRL;
        tf::Vector3 w_nextPosRR;

        footstep_marker::FootPositionVisualizer footPosVisualizer;
        footstep_marker::FootPositionVisualizer nextFootPosVisualizer;
        std::vector<Eigen::Vector3d> footPositions;
        std::vector<Eigen::Vector3d> nextFootPositions;

        tf::Transform w_transformTwist_b;
        tf::Transform w_transformFootPos_b;

        struct robotState
        {
            float pos_x, pos_y, pos_z;
            float vel_x, vel_y, vel_z, vela_x, vela_y, vela_z;
        };

        tf::Vector3 w_basePos;
        tf::Vector3 w_baseTwist_linear;
        tf::Vector3 w_baseTwist_angular;
        tf::Vector3 b_baseTwist_linear;
        tf::Vector3 b_baseTwist_angular;

        tf::Vector3 w_cmdVel_linear;
        tf::Vector3 w_cmdVel_angular;
        tf::Vector3 b_cmdVel_linear;
        tf::Vector3 b_cmdVel_angular;

        double elevation_FL;
        double elevation_FR;
        double elevation_RL;
        double elevation_RR;

        double traversability_FL;
        double traversability_FR;
        double traversability_RL;
        double traversability_RR;

        tf::Vector3 r_cmd_FL;
        tf::Vector3 r_cmd_FR;
        tf::Vector3 r_cmd_RL;
        tf::Vector3 r_cmd_RR;

};

} /* namespace*/