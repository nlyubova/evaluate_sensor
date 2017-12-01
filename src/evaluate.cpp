#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/highgui/highgui.hpp>

#include "evaluate_sensor/evaluate.hpp"

void Evaluator::process_cameraInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
  cv::Mat camMatrix = (cv::Mat_<double>(3, 3) <<
                       infoMsg->K[0], infoMsg->K[1], infoMsg->K[2],
                       infoMsg->K[3], infoMsg->K[4], infoMsg->K[5],
                       infoMsg->K[6], infoMsg->K[7], infoMsg->K[8]);

  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
                        infoMsg->D[0],
                        infoMsg->D[1],
                        infoMsg->D[2],
                        infoMsg->D[3],
                        infoMsg->D[4]);

  //unsubscribe from the topic
  sub_camera_info_.shutdown();
}

Evaluator::Evaluator():
  nh_("~"),
  it_(nh_),
  processedDepth_(0),
  processedPc_(0),
  processMax_(10)
{

  // Create a ROS publisher for the output model coefficients
  pub_plane_norm = nh_.advertise<geometry_msgs::PoseStamped> ("plane_normal", 1);
  // Create a ROS publisher for the output point cloud
  pub = nh_.advertise<sensor_msgs::PointCloud2> ("fitted_plane", 100);

  poses_left_pub = nh_.advertise<geometry_msgs::PoseArray> ("poses_left", 100);
  poses_right_pub = nh_.advertise<geometry_msgs::PoseArray> ("poses_right", 100);

  pose_left_pub = nh_.advertise<geometry_msgs::PoseStamped> ("pose_left", 100);
  pose_right_pub = nh_.advertise<geometry_msgs::PoseStamped> ("pose_right", 100);


  std::string detector_params_file = "/config/detector_params.yml";
  nh_.getParam("detector_params_file", detector_params_file);
  ROS_INFO_STREAM("detector_params_file: " << detector_params_file);

  std::string topic_depth_img = "/camera/depth/image_raw";
  nh_.getParam("depth_img_topic", topic_depth_img);
  ROS_INFO_STREAM("topic_depth_img: " << topic_depth_img);
  sub_img_depth_ = it_.subscribe(topic_depth_img.c_str(), 10, &Evaluator::process_depth, this);

  std::string topic_rgb_img = "/camera/rgb/image_raw";
  nh_.getParam("rgb_img_topic", topic_rgb_img);
  ROS_INFO_STREAM("topic_rgb_img: " << topic_rgb_img);
  sub_img_rgb_ = it_.subscribe(topic_rgb_img.c_str(), 10, &Evaluator::process_rgb, this);

/*  std::string topic_pointcloud = "/camera/depth/points";
  nh_.getParam("pointcloud_topic", topic_pointcloud);
  ROS_INFO_STREAM("pointcloud_topic: " << topic_pointcloud);
  sub_pc_ = nh_.subscribe(topic_pointcloud.c_str(), 10, &Evaluator::process_pc, this);
*/
  std::string topic_camera_info = "/camera/depth/camera_info";
  nh_.getParam("camera_info_topic", topic_camera_info);
  ROS_INFO_STREAM("camera_info_topic: " << topic_camera_info << "\n");
  sub_camera_info_ = nh_.subscribe(topic_camera_info.c_str(), 1, &Evaluator::process_cameraInfo, this);
}

void Evaluator::process_rgb(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
  }
}

void Evaluator::process_pc(const sensor_msgs::PointCloud2& msg)
{
  /*if (processedPc_ < processMax_)
    ++processedPc_;
  else
    sub_pc_.shutdown();*/

  sensor_msgs::PointCloud2 msg2 = msg;

  int w = msg.width;
  int h = msg.height;
  int points_nbr = 0;

  //1) compute basic statistics: mean, min, max
  float max = evaluator_pc_.compute_stat(msg2, w, h, points_nbr);
  std::cout << std::endl;

  //2) compute mean at image corners
  evaluator_pc_.test_corners(msg2, w, h, max);
  std::cout << std::endl;

  //3) compute the histogram for the whole image
  evaluator_pc_.test_hist(msg2, w, h, 0.02f, points_nbr);
  std::cout << std::endl << std::endl;

  //4) fitting a plane
  /*sensor_msgs::PointCloud2::Ptr final_cloud;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = msg.header.frame_id;
  evaluator_pc_.compute_plane(msg2, final_cloud, pose);
  // Publish the data
  if (final_cloud->data.size() > 0)
    pub.publish(final_cloud); // publish the new pcl
  pub_plane_norm.publish(pose);*/
}

void Evaluator::process_depth(const sensor_msgs::ImageConstPtr& msg)
{
  if (processedDepth_ < processMax_)
    ++processedDepth_;
  else
    sub_img_depth_.shutdown();

  std::cout << "Depth image type, size: " << msg->encoding
            << " " << msg->width << "x" << msg->height << std::endl;

  cv::Mat out;
  out.create(cv::Size(msg->height, msg->width), CV_32FC1);

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
  }

  try
  {
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      cv_ptr->image.convertTo(out, CV_32FC1);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      //convert to float so that it is in meters
      cv_ptr->image.convertTo(out, CV_32FC1, 1 / 1000.0);
      // Should we do std::numeric_limits<uint16_t>::max()?
      cv::Mat valid_mask = cv_ptr->image == std::numeric_limits<uint16_t>::min();
      out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask);
    }
    else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      //convert to float so that it is in meters
      cv_ptr->image.convertTo(out, CV_32FC1, 1 / 1000.0);
      cv::Mat valid_mask = cv_ptr->image == std::numeric_limits<uint8_t>::min();
      out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask);
    }
    else
    {
      ROS_ERROR("Unknown depth image format");
      return;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int w = msg->width;
  int h = msg->height;

  //1) compute basic statistics: mean, min, max
  float max = evaluator_depth_.compute_stat(cv_ptr, w, h);
  std::cout << std::endl;

  //2) compute mean at image corners
  evaluator_depth_.test_corners(cv_ptr, w, h, max);
  std::cout << std::endl;

  //3) compute the histogram for the whole image
  evaluator_depth_.test_hist(cv_ptr, w, h, 45);
  std::cout << std::endl;

  //4) fitting a plane
  //sensor_msgs::PointCloud2::Ptr final_cloud;
  //geometry_msgs::PoseStamped pose;
  //pose.header.frame_id = msg->header.frame_id;
  //evaluator_depth_.compute_plane(cv_ptr, final_cloud, pose);

  //5) compute temporal noise
  evaluator_depth_.test_temp(cv_ptr, w, h);
  std::cout << std::endl << std::endl;
}
