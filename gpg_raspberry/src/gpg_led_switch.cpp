#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpg_led_switch");
  ros::NodeHandle nh;

  wiringPiSetup();
  pinMode(LED_PIN, OUTPUT);

  // loop frequency 1Hz
  ros::Rate loop_rate(2);
  int led_state = 0;
  while (ros::ok())
  {
    led_state = !led_state;
    ROS_INFO("LED STATE: %d", led_state);
    if (led_state)
        digitalWrite(LED_PIN, HIGH);
    else
        digitalWrite(LED_PIN, LOW);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
