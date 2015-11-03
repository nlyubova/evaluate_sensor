#ifndef EVALUATE_HPP
#define EVALUATE_HPP

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

class Evaluator
{
public:
  Evaluator();
  void test(const sensor_msgs::ImageConstPtr& msg);
  void test_pc(const sensor_msgs::PointCloud2& msg);

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_img_;
  ros::Subscriber sub_pc_;

private:

  float compute_hist(cv_bridge::CvImagePtr cv_ptr,
                     const std::vector<float> &mean_val,
                     const int x_min, const int x_max,
                     const int y_min, const int y_max,
                     bool print=false);
  float compute_hist(sensor_msgs::PointCloud2 &msg,
                     const std::vector<float> &mean_val,
                     const int x_min, const int x_max,
                     const int y_min, const int y_max,
                     const int points_nbr=0,
                     bool print=false);

  float compute_stat(cv_bridge::CvImagePtr cv_ptr,
                     const int &w, const int &h);
  float compute_stat(sensor_msgs::PointCloud2 &msg,
                     const int &w, const int &h,
                     int &nbr);

};

#endif // EVALUATE_HPP
