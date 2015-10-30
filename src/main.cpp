
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

//#include <boost/program_options.hpp>

#include <evaluate_sensor/evaluate.hpp>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"evaluate_sensor");

  Evaluator fm;
  //Evaluator *fm = new Evaluator();

  /*ros::AsyncSpinner spinner(1);
  spinner.start();*/

  ros::Rate rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();

  return 0;
}
