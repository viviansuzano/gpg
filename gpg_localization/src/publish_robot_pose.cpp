#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

void groundTruthCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;
  tf::Transform o_transform_b;

  // orientation
  tf::Quaternion baseOrientation;
  baseOrientation.setX(msg->pose.pose.orientation.x);
  baseOrientation.setY(msg->pose.pose.orientation.y);
  baseOrientation.setZ(msg->pose.pose.orientation.z);
  baseOrientation.setW(msg->pose.pose.orientation.w);

  // translation
  tf::Vector3 translationWorldToBaseFootprint(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

  // transform from world -> base_footprint
  o_transform_b.setRotation(baseOrientation);
  o_transform_b.setOrigin(translationWorldToBaseFootprint);
  br.sendTransform(tf::StampedTransform(o_transform_b, msg->header.stamp, "odom", "base_footprint"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_robot_pose");
  ros::NodeHandle nodeHandle("~");
  
  ros::Subscriber odomSub_ = nodeHandle.subscribe("/ground_truth", 1, groundTruthCallback);
  
	ros::spin();
  
  return 0;
}
