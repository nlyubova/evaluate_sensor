#ifndef EVALUATE_DEPTH_HPP
#define EVALUATE_DEPTH_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include "evaluate_sensor/plane_fitting.hpp"

class Evaluator_depth
{
public:
  Evaluator_depth();

  float compute_stat(cv_bridge::CvImagePtr cv_ptr,
                     const int &w,
                     const int &h);

  void test_corners(cv_bridge::CvImagePtr cv_ptr,
                    const int &w,
                    const int &h,
                    const int &max);

  void test_hist(cv_bridge::CvImagePtr cv_ptr,
                 const int &w,
                 const int &h,
                 const int &bins);

  void compute_plane(cv_bridge::CvImagePtr img,
                     sensor_msgs::PointCloud2::Ptr &final_cloud,
                     geometry_msgs::PoseStamped &pose1);

  void test_temp(cv_bridge::CvImagePtr cv_ptr,
                 const int &w,
                 const int &h);

private:

  Plane_fitting plane_fitting;

  float compute_hist(cv_bridge::CvImagePtr cv_ptr,
                     const std::vector<float> &mean_val,
                     const int x_min,
                     const int x_max,
                     const int y_min,
                     const int y_max,
                     bool print=false);

  //accumulated image difference
  cv::Mat tempNoise;

  //previous image used for computing temporal noise
  cv::Mat imgPrev;

  //average noise per pixel over time
  std::vector <float> noiseTemp_;
};

#endif // EVALUATE_DEPTH_HPP
