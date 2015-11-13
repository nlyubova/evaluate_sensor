#include <sensor_msgs/point_cloud2_iterator.h>
//#include <geometry_msgs/PoseStamped.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
/*#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>*/
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "evaluate_sensor/evaluate.hpp"

static void convertFromRosTf(const tf::Transform& tfTransform, cv::Mat &rotation, cv::Vec3d &up) //, HomogeneousTransformation<PrimType_, Position_, Rotation_>& pose
{
  const tf::Vector3& rowX = tfTransform.getBasis().getRow(0);
  const tf::Vector3& rowY = tfTransform.getBasis().getRow(1);
  const tf::Vector3& rowZ = tfTransform.getBasis().getRow(2);

  rotation = (cv::Mat_<double>(3, 3) <<
                    rowX.x(), rowX.y(), rowX.z(),
                    rowY.x(), rowY.y(), rowY.z(),
                    rowZ.x(), rowZ.y(), rowZ.z());

  up = cv::Vec3d(tfTransform.getOrigin().getX(),
                    tfTransform.getOrigin().getY(),
                    tfTransform.getOrigin().getZ());

  ROS_INFO_STREAM("transform " << rotation << std::endl << up);
}

bool Evaluator::computeRotation(const Eigen::Vector3d& surfaceNormal, const std::string& frameId)
{
  // Get surface normal in tilting frame.
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform(tiltingFrameId_, frameId, ros::Time(0), transform);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  cv::Mat rotation;
  cv::Vec3d up;
  convertFromRosTf(transform, rotation, up);

  //kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD pose;
  /*kindr::poses::eigen_impl::convertFromRosTf(transform, pose);
  ROS_INFO_STREAM("Pose of sensor frame in tilting frame: " << pose);
  Eigen::Vector3d surfaceNormalInTiltingFrame = pose.getRotation().rotate(surfaceNormal);
    if (surfaceNormalInTiltingFrame.z() < 0.0) surfaceNormalInTiltingFrame = -surfaceNormalInTiltingFrame;
  ROS_DEBUG_STREAM("Surface normal in tilting frame (" << tiltingFrameId_ << "): " << surfaceNormalInTiltingFrame.transpose());

  // Compute calibration angles.
  Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
  kindr::rotations::eigen_impl::RotationQuaternionPD rotation;
  rotation.setFromVectors(surfaceNormalInTiltingFrame, reference);

  std::cout << "===============================" << std::endl;
  std::cout << "Quaternion (qx, qy, qz, qw): " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ", " << rotation.w() << std::endl;
  kindr::rotations::eigen_impl::EulerAnglesYprPD euler(rotation);
  Eigen::Vector3d eulerVector = euler.getUnique().toImplementation() / M_PI * 180.0;
  std::cout << "Pitch: " << eulerVector(1) << " deg, Roll: " << eulerVector(2) << " deg" << std::endl; // Yaw should be 0 up to numerical errors.
  std::cout << "===============================" << std::endl;*/

  return true;
}

void Evaluator::compute_plane(sensor_msgs::PointCloud2 &msg)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud(cloud.makeShared());
  pcl::ModelCoefficients plan_coeff;
  //pcl::PointIndices inliers;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  seg.segment(*inliers, plan_coeff);
  if (inliers->indices.size () == 0)
   {
     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
     return;
   }

  // Extract the inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  // conbert to sensormsg type
  pcl::toROSMsg (*cloud_p, *final_cloud);
  // Publish the data
  pub.publish (final_cloud); // publish the new pcl

  // Publish the model coefficients
  /*pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(plan_coeff, ros_coefficients);
  pub_coeff.publish (ros_coefficients);*/

  // publish the plane normal
  Eigen::Vector3d normal(plan_coeff.values[0], plan_coeff.values[1], plan_coeff.values[2]);
  Eigen::Vector3d normal_ideal(0.0, 0.0, 1.0); //c=1, X-Y plane

  ROS_INFO_STREAM("Surface normal in sensor frame (" << msg.header.frame_id << "): " << normal.transpose());
  if (!computeRotation(normal, msg.header.frame_id))
    return;

  /*geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation. .pose.position.z = 0.0;
  pub_plane_norm.publish(pose);*/

  /*Eigen::Affine3f t=Eigen::Affine3f();
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  pcl::getEulerAngles();*/

  double a, b, c, d;
  a = plan_coeff.values[0];
  b = plan_coeff.values[1];
  c = plan_coeff.values[2];
  d = plan_coeff.values[3];

  double depth_mean = 0.0;

  double dst_inliers_mean = 0.0;
  double dst_inliers_deviation = 0.0;
  { // Compute distance means from plan for inliers
    std::vector<double> distances;
    for (size_t i = 0; i < inliers->indices.size(); ++i) {
      pcl::PointXYZ pt = cloud.points[inliers->indices[i]];
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

    std::cout <<  "Cloud size: "	<< msg.width << "x" << msg.height << std::endl
                << "Depth mean: " << depth_mean << std::endl
                << "Inliers nb: " << inliers->indices.size() << " Plan: "
                << plan_coeff.values[0] << " "
                << plan_coeff.values[1] << " "
                << plan_coeff.values[2] << " "
                << plan_coeff.values[3] << std::endl
                << "Inliers DistanceToPlan Mean: " << dst_inliers_mean << std::endl
                << "Inliers DistanceToPlan Std Deviation: " << dst_inliers_deviation << std::endl
                << "All pixels DistanceToPlan Mean: " << dst_full_mean << std::endl
                << "All pixels DistanceToPlan Std Deviation: " << dst_full_deviation << std::endl << std::endl;
}

float Evaluator::compute_hist(sensor_msgs::PointCloud2 &msg,
                              const std::vector<float> &mean_val,
                              const int x_min, const int x_max,
                              const int y_min, const int y_max,
                              const int points_nbr,
                              bool print)
{
  std::vector<int> mean_hist(mean_val.size()-1, 0);

  //compute hist
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  int pixel_nbr = msg.width*msg.height;
  for(int k=0; k<mean_val.size()-1; ++k)
  {
    iter_x = sensor_msgs::PointCloud2Iterator<float>(msg, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(msg, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(msg, "z");
    for (int count = 0; count < pixel_nbr; ++count, ++iter_x, ++iter_y, ++iter_z)
      if ((*iter_x > x_min) && (*iter_x < x_max)
          && (*iter_y > y_min) && (*iter_y < y_max))
      {
        float tmp = *iter_z;
        if ((tmp >= mean_val[k]) && (tmp < mean_val[k+1]))
          ++mean_hist[k];
      }
  }

  if (print)
  {
    std::cout << "Hist counts: ";
    for (std::vector<int>::iterator it=mean_hist.begin(); it!=mean_hist.end(); ++it)
      if (points_nbr > 0)
        std::cout << *it/static_cast<float>(points_nbr) << " ";
      else
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
    iter_x = sensor_msgs::PointCloud2Iterator<float>(msg, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(msg, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(msg, "z");
    for (int count = 0; count < pixel_nbr; ++count, ++iter_x, ++iter_y, ++iter_z)
      if ((*iter_x > x_min) && (*iter_x < x_max)
          && (*iter_y > y_min) && (*iter_y < y_max))
      {
        float tmp = *iter_z;

        if (tmp >= mean_val[max_id])
          mean_temp += tmp;
      }
    mean_temp /= static_cast<float>(max_count);
  }

  return mean_temp;
}

float Evaluator::compute_stat(sensor_msgs::PointCloud2 &msg,
                              const int &w, const int &h,
                              int &nbr)
{
  float mean = 0.0f;
  float min = std::numeric_limits<float>::max();
  float max = 0.0f;
  int pixel_nbr = w*h;

  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  for(int count=0; count < pixel_nbr; ++count, ++iter_z)
  {
    float tmp = *iter_z;

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
  iter_z = sensor_msgs::PointCloud2Iterator<float>(msg, "z");
  for (int count = 0; count < pixel_nbr; ++count, ++iter_z)
  {
    float tmp = *iter_z;

    if (tmp > 0.0f)
      var += (tmp-mean) * (tmp-mean);
  }
  var /= static_cast<float>(nbr);

  //std::cout << "- - - - - time mean var nbr min max " << std::endl;
  std::cout << "stat from_PC: " << mean << " " << var << " " << nbr << " " << min << " " << max << std::endl;
  return max;
}

void Evaluator::test_pc(const sensor_msgs::PointCloud2& msg)
{
  std::cerr << "Cloud size: "	<< msg.width << "x" << msg.height << std::endl;
  int w = msg.width;
  int h = msg.height;
  int points_nbr = 0;

  sensor_msgs::PointCloud2 msg2 = msg;

  //1) compute basic statistics: mean, min, max
  float max = compute_stat(msg2, w, h, points_nbr);

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
  mean_corners[0] = compute_hist(msg2, mean_val, 0, w/3, h/3*2, h);
  mean_corners[1] = compute_hist(msg2, mean_val, w/3*2, w, h/3*2, h);
  mean_corners[2] = compute_hist(msg2, mean_val, 0, w/3, 0, h/3);
  mean_corners[3] = compute_hist(msg2, mean_val, w/3*2, w, 0, h/3);

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
  compute_hist(msg2, test_val, 0, w, 0, h, points_nbr, true);
  std::cout << std::endl;

  compute_plane(msg2);
}
