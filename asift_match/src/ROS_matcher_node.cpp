#include "ROS_matcher.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ROS_matcher");

  ROS_matcher ros_matcher;

  ros::spin();

  return 0;
}