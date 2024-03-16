#pragma once

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace footstep_marker {

class FootPositionVisualizer {

public:

	FootPositionVisualizer (ros::NodeHandle& nodeHandle, const std::string& topic, const int num) {
		nodeHandle_ = nodeHandle;
		marker_type = num;
		initPublishers(topic);
		initMarkers(num);

	} 

	bool initPublishers(const std::string& topic) {
		footPosFL_.first = nodeHandle_.advertise<visualization_msgs::Marker>(topic + "_FL", 1);
		footPosFR_.first = nodeHandle_.advertise<visualization_msgs::Marker>(topic + "_FR", 1);
		footPosRL_.first = nodeHandle_.advertise<visualization_msgs::Marker>(topic + "_RL", 1);
		footPosRR_.first = nodeHandle_.advertise<visualization_msgs::Marker>(topic + "_RR", 1);
		return true;
	}

	bool initMarkers(int num) {
		// num = 0 -> current foot markers | num = 1 -> next foot markers
		int id[4] = {0+num*4, 1+num*4, 2+num*4, 3+num*4};
		
		std::vector<float> rgba;
		int type;
		double scale;

		if (num == 0){
			rgba = {0.13, 0.16, 0.85, 1.0};
			type = visualization_msgs::Marker::SPHERE;
			scale = 0.04;
		} else if (num == 1){
			rgba = {0.14, 0.83, 0.3, 0.8};
			type = visualization_msgs::Marker::SPHERE;
			scale = 0.04;
		} else if (num == 2){
			rgba = {0.45, 0.45, 0.5, 0.8};
			type = visualization_msgs::Marker::CUBE;
			scale = 0.03;
		} else{
			rgba = {1.0, 1.0, 1.0, 1.0};
			type = visualization_msgs::Marker::SPHERE;
			scale = 0.04;
		}
	
		visualization_msgs::Marker pointMarker = getSphereMarker("world","foot_pos_FL", id[0], type, scale, rgba[0], rgba[1], rgba[2], rgba[3]);
		footPosFL_.second = pointMarker;

		pointMarker = getSphereMarker("world","foot_pos_FR", id[1], type, scale, rgba[0], rgba[1], rgba[2], rgba[3]);
		footPosFR_.second = pointMarker;

		pointMarker = getSphereMarker("world","foot_pos_RL", id[2], type, scale, rgba[0], rgba[1], rgba[2], rgba[3]);
		footPosRL_.second = pointMarker;

		pointMarker = getSphereMarker("world","foot_pos_RR", id[3], type, scale, rgba[0], rgba[1], rgba[2], rgba[3]);
		footPosRR_.second = pointMarker;

		return true;
	}

	visualization_msgs::Marker getSphereMarker (const std::string& frame_id,
												const std::string& ns,
												const int id,
												const int type,
												const double scale,
												const float red,
												const float green,
												const float blue,
												const float a) {
		visualization_msgs::Marker marker;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = type;
		marker.scale.x = scale;
		marker.scale.y = scale;
		marker.scale.z = scale;
		marker.color.a = a;
		marker.color.r = red;
		marker.color.g = green;
		marker.color.b = blue;
		marker.id = id;
		marker.ns = ns;
		marker.header.stamp = ros::Time::now();
		marker.header.frame_id = frame_id;

		return marker;
	}

	void updateMarkers(const std::vector<Eigen::Vector3d>& foot_positions) {
		footPosFL_.second.pose.position.x = foot_positions.at(FL).x();
		footPosFL_.second.pose.position.y = foot_positions.at(FL).y();
		footPosFL_.second.pose.position.z = foot_positions.at(FL).z();

		footPosFR_.second.pose.position.x = foot_positions.at(FR).x();
		footPosFR_.second.pose.position.y = foot_positions.at(FR).y();
		footPosFR_.second.pose.position.z = foot_positions.at(FR).z();

		footPosRL_.second.pose.position.x = foot_positions.at(RL).x();
		footPosRL_.second.pose.position.y = foot_positions.at(RL).y();
		footPosRL_.second.pose.position.z = foot_positions.at(RL).z();

		footPosRR_.second.pose.position.x = foot_positions.at(RR).x();
		footPosRR_.second.pose.position.y = foot_positions.at(RR).y();
		footPosRR_.second.pose.position.z = foot_positions.at(RR).z();

		footPosFL_.second.header.stamp = ros::Time::now();
		footPosFR_.second.header.stamp = ros::Time::now();
		footPosRL_.second.header.stamp = ros::Time::now();
		footPosRR_.second.header.stamp = ros::Time::now();
	}

	void changeColor(const std::array<u_int,4>& binary_traversability) {
		// Altera cor ou canal alfa de acordo com a atravessabilidade binária
		if (binary_traversability.at(FL) == 0){
			if (marker_type == 0)
				footPosFL_.second.color.a = 0.8;
			else if (marker_type == 1){
				footPosFL_.second.color.r = 0.83;
				footPosFL_.second.color.g = 0.16;
				footPosFL_.second.color.b = 0.14;
			} else if (marker_type == 2){
				footPosFL_.second.color.a = 0.8;
			}
		} else {
			if (marker_type == 0)
				footPosFL_.second.color.a = 1.0;
			else if (marker_type == 1){
				footPosFL_.second.color.r = 0.14;
				footPosFL_.second.color.g = 0.83;
				footPosFL_.second.color.b = 0.3;
			} else if (marker_type == 2){
				footPosFL_.second.color.a = 0.0;
			}
		}
	
		if (binary_traversability.at(FR) == 0){
			if (marker_type == 0)
				footPosFR_.second.color.a = 0.8;
			else if (marker_type == 1){
				footPosFR_.second.color.r = 0.83;
				footPosFR_.second.color.g = 0.16;
				footPosFR_.second.color.b = 0.14;
			} else if (marker_type == 2){
				footPosFR_.second.color.a = 0.8;
			}
		} else {
			if (marker_type == 0)
				footPosFR_.second.color.a = 1.0;
			else if (marker_type == 1){
				footPosFR_.second.color.r = 0.14;
				footPosFR_.second.color.g = 0.83;
				footPosFR_.second.color.b = 0.3;
			} else if (marker_type == 2){
				footPosFR_.second.color.a = 0.0;
			}
		}

		if (binary_traversability.at(RL) == 0){
			if (marker_type == 0)
				footPosRL_.second.color.a = 0.8;
			else if (marker_type == 1){
				footPosRL_.second.color.r = 0.83;
				footPosRL_.second.color.g = 0.16;
				footPosRL_.second.color.b = 0.14;
			} else if (marker_type == 2){
				footPosRL_.second.color.a = 0.8;
			}
		} else {
			if (marker_type == 0)
				footPosRL_.second.color.a = 1.0;
			else if (marker_type == 1){
				footPosRL_.second.color.r = 0.14;
				footPosRL_.second.color.g = 0.83;
				footPosRL_.second.color.b = 0.3;
			} else if (marker_type == 2){
				footPosRL_.second.color.a = 0.0;
			}
		}

		if (binary_traversability.at(RR) == 0){
			if (marker_type == 0)
				footPosRR_.second.color.a = 0.8;
			else if (marker_type == 1){
				footPosRR_.second.color.r = 0.83;
				footPosRR_.second.color.g = 0.16;
				footPosRR_.second.color.b = 0.14;
			} else if (marker_type == 2){
				footPosRR_.second.color.a = 0.8;
			}
		} else {
			if (marker_type == 0)
				footPosRR_.second.color.a = 1.0;
			else if (marker_type == 1){
				footPosRR_.second.color.r = 0.14;
				footPosRR_.second.color.g = 0.83;
				footPosRR_.second.color.b = 0.3;
			} else if (marker_type == 2){
				footPosRR_.second.color.a = 0.0;
			}
		}

	}

	bool publish() {
		footPosFL_.first.publish(footPosFL_.second);
		footPosFR_.first.publish(footPosFR_.second);
		footPosRL_.first.publish(footPosRL_.second);
		footPosRR_.first.publish(footPosRR_.second);

	    return true;
	}

	int getTypeMarker(){
		return marker_type;
	}


protected:
	ros::NodeHandle nodeHandle_;
	int marker_type;
	int n_legs_;

	std::pair<ros::Publisher, visualization_msgs::Marker> footPosFL_;
	std::pair<ros::Publisher, visualization_msgs::Marker> footPosFR_;
	std::pair<ros::Publisher, visualization_msgs::Marker> footPosRL_;
	std::pair<ros::Publisher, visualization_msgs::Marker> footPosRR_;

	enum FootIDs { FL=0, FR, RL, RR };

};


}