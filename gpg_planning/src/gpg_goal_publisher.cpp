#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gpg_goal_publisher");
  ros::NodeHandle nh;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/gpg_velocity_controller/cmd_vel", 100);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0.3;
		cmd_vel.angular.z = 0.2;

    ROS_INFO("Linear velocity: %.2f, Angular velocity: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);

    cmd_vel_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
