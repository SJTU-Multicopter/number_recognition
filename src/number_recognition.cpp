#include "ros/ros.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "number_recognition");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
