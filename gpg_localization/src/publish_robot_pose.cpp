#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

ros::Publisher posePub_;

void groundTruthCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;
  tf::Transform w_transform_b;

  // orientation
  tf::Quaternion baseOrientation;
  baseOrientation.setX(msg->pose.pose.orientation.x);
  baseOrientation.setY(msg->pose.pose.orientation.y);
  baseOrientation.setZ(msg->pose.pose.orientation.z);
  baseOrientation.setW(msg->pose.pose.orientation.w);

  // translation
  tf::Vector3 translationWorldToBaseFootprint(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

  // transform from world -> base_footprint
  w_transform_b.setRotation(baseOrientation);
  w_transform_b.setOrigin(translationWorldToBaseFootprint);
  br.sendTransform(tf::StampedTransform(w_transform_b, msg->header.stamp, "world", "base_footprint"));

	// publish pose for elevation_map
	geometry_msgs::PoseWithCovarianceStamped poseMsg;
	poseMsg.header.stamp = ros::Time::now();
	poseMsg.header.frame_id = "world";
	poseMsg.pose.pose.position.x = msg->pose.pose.position.x;
	poseMsg.pose.pose.position.y = msg->pose.pose.position.y;
	poseMsg.pose.pose.position.z = 0.0;
	poseMsg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
	poseMsg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
	poseMsg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
	poseMsg.pose.pose.orientation.w = msg->pose.pose.orientation.w;

	posePub_.publish(poseMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_robot_pose");
  ros::NodeHandle nodeHandle("~");
  
  ros::Subscriber odomSub_ = nodeHandle.subscribe("/ground_truth", 1, groundTruthCallback);
  posePub_ = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1);
  
	ros::spin();
  
  return 0;
}
