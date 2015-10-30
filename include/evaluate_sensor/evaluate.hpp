#ifndef EVALUATE_HPP
#define EVALUATE_HPP

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

class Evaluator
{
public:
  Evaluator();
  void test(const sensor_msgs::ImageConstPtr& msg);

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_img_;

private:

  float compute_hist(cv_bridge::CvImagePtr cv_ptr, const std::vector<float> &mean_val, const int i_min, const int i_max, const int j_min, const int j_max, bool print=false);
  float compute_stat(cv_bridge::CvImagePtr cv_ptr, int w, int h);

};

#endif // EVALUATE_HPP
