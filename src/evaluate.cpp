#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <geometry_msgs/PoseStamped.h>

#include "evaluate_sensor/evaluate.hpp"

Evaluator::Evaluator():
  nh_("~"),
  it_(nh_)
{
  // Create a ROS publisher for the output model coefficients
  pub_plane_norm = nh_.advertise<geometry_msgs::PoseStamped> ("plane_normal", 1);
  // Create a ROS publisher for the output point cloud
  pub = nh_.advertise<sensor_msgs::PointCloud2> ("output/points", 100);

  /*std::string topic_depth_img = "/camera/depth_registered/image_raw";
  nh_.getParam("depth_img_topic", topic_depth_img);
  ROS_INFO_STREAM("topic_depth_img: " << topic_depth_img);
  sub_img_ = it_.subscribe(topic_depth_img.c_str(), 10, &Evaluator::test, this);*/

  tiltingFrameId_ = "CameraDepth_frame";
  std::string topic_pointcloud = "/camera/depth_registered/points";
  nh_.getParam("pointcloud_topic", topic_pointcloud);
  ROS_INFO_STREAM("pointcloud_topic: " << topic_pointcloud);
  sub_pc_ = nh_.subscribe(topic_pointcloud.c_str(), 10, &Evaluator::test_pc, this);
}

void Evaluator::compute_plane(cv_bridge::CvImagePtr img)
{
  // Compute Plan
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cv::MatIterator_<cv::Vec3f> it = img->image.begin<cv::Vec3f>();
  cv::MatIterator_<cv::Vec3f> it_end = img->image.end<cv::Vec3f>();
  for (; it != it_end; ++it) {
    cv::Vec3f px = *it;
    cloud.push_back(pcl::PointXYZ(px[0], px[1], px[2]));
  }

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud(cloud.makeShared());
  pcl::ModelCoefficients plan_coeff;
  pcl::PointIndices inliers;
  seg.segment(inliers, plan_coeff);
  if (inliers.indices.size () == 0)
   {
     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     return;
   }

  double a, b, c, d;
  a = plan_coeff.values[0];
  b = plan_coeff.values[1];
  c = plan_coeff.values[2];
  d = plan_coeff.values[3];

  // Publish the model coefficients
  /*pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(plan_coeff, ros_coefficients);
  pub_coeff.publish (ros_coefficients);*/

  // publish the plane normal
  /*geometry_msgs::PoseStamped msg_obj_pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;*/

  /*Eigen::Affine3f t=Eigen::Affine3f();
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  pcl::getEulerAngles();*/

  double depth_mean = 0.0;

  double dst_inliers_mean = 0.0;
  double dst_inliers_deviation = 0.0;
  { // Compute distance means from plan for inliers
    std::vector<double> distances;
    for (size_t i = 0; i < inliers.indices.size(); ++i) {
      pcl::PointXYZ pt = cloud.points[inliers.indices[i]];
      distances.push_back(pcl::pointToPlaneDistance<pcl::PointXYZ>(pt, a, b, c, d));
      dst_inliers_mean += distances.back();
      depth_mean += pt.z;
    }
    dst_inliers_mean = dst_inliers_mean / distances.size();
    depth_mean = depth_mean / distances.size();
    // Compute standard deviation from plan for inliers
    for (std::vector<double>::const_iterator it = distances.begin(); it != distances.end(); ++it) {
      dst_inliers_deviation += std::pow((*it - dst_inliers_mean), 2);
    }
    dst_inliers_deviation = std::sqrt(dst_inliers_deviation / (float)distances.size());
  }

  double dst_full_mean = 0.0;
  double dst_full_deviation = 0.0;
  { // Compute distance means from plan for all pixels
    std::vector<double> distances;
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      pcl::PointXYZ pt = cloud.points[i];
      if (!std::isnan(pt.z)) {
        distances.push_back(pcl::pointToPlaneDistance<pcl::PointXYZ>(pt, a, b, c, d));
        dst_full_mean += distances.back();
      }
    }
    dst_full_mean = dst_full_mean / distances.size();
    // Compute standard deviation from plan for all pixels
    for (std::vector<double>::const_iterator it = distances.begin(); it != distances.end(); ++it) {
      dst_full_deviation += std::pow((*it - dst_full_mean), 2);
    }
    dst_full_deviation = std::sqrt(dst_full_deviation / (float)distances.size());
  }

    std::cout <<  "Cloud size: "	<< img->image.cols << "x" << img->image.rows << std::endl
                << "Depth mean: " << depth_mean << std::endl
                << "Inliers nb: " << inliers.indices.size() << " Plan: "
                << plan_coeff.values[0] << " "
                << plan_coeff.values[1] << " "
                << plan_coeff.values[2] << " "
                << plan_coeff.values[3] << std::endl
                << "Inliers DistanceToPlan Mean: " << dst_inliers_mean << std::endl
                << "Inliers DistanceToPlan Std Deviation: " << dst_inliers_deviation << std::endl
                << "All pixels DistanceToPlan Mean: " << dst_full_mean << std::endl
                << "All pixels DistanceToPlan Std Deviation: " << dst_full_deviation << std::endl << std::endl;
}

float Evaluator::compute_hist(cv_bridge::CvImagePtr img,
                              const std::vector<float> &mean_val,
                              const int x_min, const int x_max,
                              const int y_min, const int y_max,
                              bool print)
{
  std::vector<int> mean_hist(mean_val.size()-1, 0);

  //compute hist
  for(int k=0; k<mean_val.size()-1; ++k)
    for(int i=x_min; i<x_max; ++i)
      for(int j=y_min; j<y_max; ++j)
      {
        int ii = i*j;
        float tmp = 0.0;
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
          tmp = img->image.at<float>(ii);
        else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
          int tmp_int = img->image.at<short int>(ii);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }
        else if (img->encoding == sensor_msgs::image_encodings::MONO8)
        {
          uint8_t tmp_int = img->image.at<uint8_t>(ii);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }

        if ((tmp >= mean_val[k]) && (tmp < mean_val[k+1]))
          ++mean_hist[k];
      }

  if (print)
  {
    std::cout << "Hist counts: ";
    for (std::vector<int>::iterator it=mean_hist.begin(); it!=mean_hist.end(); ++it)
      std::cout << *it << " ";
    std::cout << std::endl;
  }

  //find a bin with most of data
  int max_count = 0;
  int max_id = 0;
  for (int i=0; i<=mean_hist.size(); ++i)
    if ((mean_hist[i] > 0) && (max_count < mean_hist[i]))
    {
      max_id = i;
      max_count = mean_hist[i];
    }

  //count mean at image corners
  float mean_temp = 0.0f;
  if (max_count > 0)
  {
    for(int i=x_min; i<x_max; i++)
      for(int j=y_min; j<y_max; j++)
      {
        int ii = i*j;
        float tmp = 0.0;
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
          tmp = img->image.at<float>(ii);
        else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
          int tmp_int = img->image.at<short int>(ii);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }
        else if (img->encoding == sensor_msgs::image_encodings::MONO8)
        {
          uint8_t tmp_int = img->image.at<uint8_t>(ii);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }

        if (tmp >= mean_val[max_id])
          mean_temp += tmp;
      }
    mean_temp /= static_cast<float>(max_count);
  }

  return mean_temp;
}

float Evaluator::compute_stat(cv_bridge::CvImagePtr img,
                              const int &w, const int &h)
{
  int nbr = 0;
  float mean = 0.0f;
  float min = std::numeric_limits<float>::max();
  float max = 0.0f;
  int pixel_nbr = w*h;

  for (int count = 0; count < pixel_nbr; ++count)
  {
    float tmp = 0.0f;

    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      tmp = img->image.at<float>(count);
    else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      int tmp_int = img->image.at<short int>(count);
      tmp = static_cast<float>(tmp_int)/1000.0f;
    }
    else if (img->encoding == sensor_msgs::image_encodings::MONO8)
    {
      short int tmp_int = img->image.at<short int>(count);
      tmp = static_cast<float>(tmp_int)/1000.0f;
    }

    if (tmp > 0.0f)
    {
      mean += tmp;
      if (tmp > max)
        max = tmp;
      if (tmp < min)
        min = tmp;
      ++nbr;
    }
  }

  //finalize basic statistics: mean, min, max
  if (nbr > 0)
    mean /= static_cast<float>(nbr);

  float var = 0.0f;
  for (int count = 0; count < pixel_nbr; ++count)
  {
    float tmp = 0.0f;

    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      tmp = img->image.at<float>(count);
    else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      int tmp_int = img->image.at<short int>(count);
      tmp = static_cast<float>(tmp_int)/1000.0f;
    }
    else if (img->encoding == sensor_msgs::image_encodings::MONO8)
    {
      short int tmp_int = img->image.at<short int>(count);
      tmp = static_cast<float>(tmp_int)/1000.0f;
    }

    if (tmp > 0.0f)
      var += (tmp-mean) * (tmp-mean);
  }
  var /= static_cast<float>(nbr);

  //std::cout << "- - - - - time mean var nbr min max " << std::endl;
  std::cout << "stat from_depthimg: " << mean << " " << var << " " << nbr << " " << min << " " << max << std::endl;
  return max;
}

void Evaluator::test(const sensor_msgs::ImageConstPtr& img)
{
  //std::cout << " ** img->encoding = " << img->encoding << std::endl;

  cv_bridge::CvImagePtr cv_ptr;
  //cv_bridge::CvImagePtr cv_ptr2;
  //cv::Mat cv2;
  cv::Mat out;
  out.create(cv::Size(img->height, img->width), CV_32FC1);

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img); //sensor_msgs::image_encodings::TYPE_32FC1 //, img->encoding
    //cv_ptr2 = cv_bridge::cvtColor(cv_ptr, sensor_msgs::image_encodings::TYPE_32FC1);
    //cv_ptr->image.convertTo(cv_ptr2->image, CV_32FC1);


    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      cv_ptr->image.convertTo(out, CV_32FC1);
    }
    else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      cv_ptr->image.convertTo(out, CV_32FC1, 1 / 1000.0); //convert to float so that it is in meters
      cv::Mat valid_mask = cv_ptr->image == std::numeric_limits<uint16_t>::min(); // Should we do std::numeric_limits<uint16_t>::max() too ?
      out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask); //set a$
    }
    else if (img->encoding == sensor_msgs::image_encodings::MONO8)
    {
      cv_ptr->image.convertTo(out, CV_32FC1, 1 / 1000.0); //convert to float so that it is in meters
      cv::Mat valid_mask = cv_ptr->image == std::numeric_limits<uint8_t>::min();
      out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  int w = img->width;
  int h = img->height;

  //1) compute basic statistics: mean, min, max
  float max = compute_stat(cv_ptr, w, h);

  //2) compute mean at image corners
  std::vector<float> mean_val;
  mean_val.push_back(0.12f);
  mean_val.push_back(max /4.0f);
  mean_val.push_back(max /2.0f);
  mean_val.push_back(max /5.0f*3.0f);
  mean_val.push_back(max);
  /*std::cout << "Histogram bins: ";
  for (std::vector<float>::iterator it=mean_val.begin(); it!=mean_val.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;*/

  std::vector<float> mean_corners(mean_val.size()-1, 0.0f);

  //compute the histogram for the corners
  mean_corners[0] = compute_hist(cv_ptr, mean_val, 0, w/3, h/3*2, h);
  mean_corners[1] = compute_hist(cv_ptr, mean_val, w/3*2, w, h/3*2, h);
  mean_corners[2] = compute_hist(cv_ptr, mean_val, 0, w/3, 0, h/3);
  mean_corners[3] = compute_hist(cv_ptr, mean_val, w/3*2, w, 0, h/3);

  std::cout << "Corners values: ";
  for (std::vector<float>::iterator it=mean_corners.begin(); it!=mean_corners.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;

  //compute the histogram for the whole image
  std::vector<float> test_val;
  test_val.resize(45);
  float val = 0.1f;
  for (int i=0; i<test_val.size();++i)
    test_val[i] = val + (1.0f-val) * static_cast<float>(i)/static_cast<float>(test_val.size());
  std::cout << "Histogram bins: ";
  for (std::vector<float>::iterator it=test_val.begin(); it!=test_val.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;

  //compute the histogram for the whole image
  compute_hist(cv_ptr, test_val, 0, w, 0, h, true);
}

