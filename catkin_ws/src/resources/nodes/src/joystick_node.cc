#include "ros/ros.h"
#include "../include/joystick.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joystick_node");

  ros::NodeHandle nh(""), nh_param("~");
  joystick::Joystick joy(&nh, &nh_param);

  ros::spin();
}
