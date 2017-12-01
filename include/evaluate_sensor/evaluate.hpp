#ifndef EVALUATE_HPP
#define EVALUATE_HPP

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include "evaluate_sensor/evaluate_depth.hpp"
#include "evaluate_sensor/evaluate_pc.hpp"

class Evaluator
{
public:
  Evaluator();
  void process_rgb(const sensor_msgs::ImageConstPtr& msg);
  void process_depth(const sensor_msgs::ImageConstPtr& msg);
  void process_pc(const sensor_msgs::PointCloud2& msg);
  void process_cameraInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg);

protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  ros::Subscriber sub_pc_, sub_camera_info_;
  image_transport::Subscriber sub_img_depth_, sub_img_rgb_;

  ros::Publisher pub, pub_plane_norm, poses_left_pub, poses_right_pub,
  pose_left_pub, pose_right_pub;

  Evaluator_depth evaluator_depth_;
  Evaluator_pc evaluator_pc_;

  int processedPc_;
  int processedDepth_;
  int processMax_;

private:
};

#endif // EVALUATE_HPP
