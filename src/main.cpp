#include <stdio.h>
#include <vector>
#include <string>

#include <ros/ros.h>

#include "evaluate_sensor/evaluate.hpp"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"evaluate_sensor");

  Evaluator fm;

  ros::Rate rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();

  return 0;
}
