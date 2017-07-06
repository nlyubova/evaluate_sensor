#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>

#include "evaluate_sensor/plane_fitting.hpp"

const double PI = 3.14159;

Plane_fitting::Plane_fitting()
{
  tiltingFrameId_ = "CameraDepth_frame";
}

static void convertFromRosTf(const tf::Transform& tfTransform,
                             cv::Mat &rotation, cv::Vec3d &up)
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

bool Plane_fitting::computeRotation(const Eigen::Vector3d& surfaceNormal,
                                    const std::string& frameId)
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

  return true;
}

/** Function that normalizes a vector
 * @param x the x component of the vector
 * @param y the y component of the vector
 * @param z the z component of the vector
 */
template<typename T>
void normalize_vector(T & x, T&y, T&z)
{
  T norm = std::sqrt(x * x + y * y + z * z);
  x /= norm;
  y /= norm;
  z /= norm;
}

void toEuler(double x,double y,double z,double angle) {
  std::cout << "axis-angle " << x << " " << y << " " << z << " " << angle << std::endl;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  double s = std::sin(angle);
  double c = std::cos(angle);
  double t = 1-c;
  //  if axis is not already normalised then uncomment this
  // double magnitude = Math.sqrt(x*x + y*y + z*z);
  // if (magnitude==0) throw error;
  // x /= magnitude;
  // y /= magnitude;
  // z /= magnitude;
  if ((x*y*t + z*s) > 0.998) { // north pole singularity detected
    roll = 2*std::atan2(x*std::sin(angle/2), std::cos(angle/2));
    pitch = PI/2;
    yaw = 0;
    return;
  }
  if ((x*y*t + z*s) < -0.998) { // south pole singularity detected
    roll = -2*std::atan2(x*std::sin(angle/2), std::cos(angle/2));
    pitch = -PI/2;
    yaw = 0;
    return;
  }
  roll = std::atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t);
  pitch = std::asin(x * y * t + z * s) ;
  yaw = std::atan2(x * s - y * z * t , 1 - (x*x + z*z) * t);
  std::cout << "RPY " << roll << " " << pitch << " " << yaw << std::endl;
}

/**
 * If the equation of the plane is ax+by+cz+d=0,
 * the pose (R,t) is such that it takes the horizontal plane (z=0)
 * to the current equation
 */
void
getPlaneTransform(const pcl::ModelCoefficients& plane_coefficients,
                  cv::Matx33f& rotation,
                  cv::Vec3f& translation)
{
  cv::Vec3d z(plane_coefficients.values[0],
          plane_coefficients.values[1],
          plane_coefficients.values[2]);

  translation = cv::Vec3d(-plane_coefficients.values[0]* plane_coefficients.values[3],
      -plane_coefficients.values[1]* plane_coefficients.values[3],
      -plane_coefficients.values[2]* plane_coefficients.values[3]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  //rotation = cv::Matx33f(-y[0], -z[0], x[0], -y[1], -z[1], x[1], -y[2], -z[2], x[2]);
  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
  //std::cout << "R " << rotation << std::endl;
  //std::cout << "T " << translation << std::endl;
}

geometry_msgs::Quaternion
maxtrixToPose(const cv::Matx33f &m)
{
  geometry_msgs::Quaternion q;

  float tr = m(0,0) + m(1,1) + m(2,2);

  if (tr > 0) {
    float S = sqrt(tr+1.0) * 2; // S=4*qw
    q.w = 0.25 * S;
    q.x = (m(2,1) - m(1,2)) / S;
    q.y = (m(0,2) - m(2,0)) / S;
    q.z = (m(1,0) - m(0,1)) / S;
  } else if ((m(0,0) > m(1,1))&(m(0,0) > m(2,2))) {
    float S = sqrt(1.0 + m(0,0) - m(1,1) - m(2,2)) * 2; // S=4*qx
    q.w = (m(2,1) - m(1,2)) / S;
    q.x = 0.25 * S;
    q.y = (m(0,1) + m(1,0)) / S;
    q.z = (m(0,2) + m(2,0)) / S;
  } else if (m(1,1) > m(2,2)) {
    float S = sqrt(1.0 + m(1,1) - m(0,0) - m(2,2)) * 2; // S=4*qy
    q.w = (m(0,2) - m(2,0)) / S;
    q.x = (m(0,1) + m(1,0)) / S;
    q.y = 0.25 * S;
    q.z = (m(1,2) + m(2,1)) / S;
  } else {
    float S = sqrt(1.0 + m(2,2) - m(0,0) - m(1,1)) * 2; // S=4*qz
    q.w = (m(1,0) - m(0,1)) / S;
    q.x = (m(0,2) + m(2,0)) / S;
    q.y = (m(1,2) + m(2,1)) / S;
    q.z = 0.25 * S;
  }
  return q;
}

void Plane_fitting::compute_plane(pcl::PointCloud<pcl::PointXYZ> cloud,
                                 sensor_msgs::PointCloud2::Ptr &final_cloud,
                                 geometry_msgs::PoseStamped &pose1)
{
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
  final_cloud = sensor_msgs::PointCloud2::Ptr(new (sensor_msgs::PointCloud2));

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: "
            << cloud_p->width * cloud_p->height << " data points." << std::endl;

  // conbert to sensormsg type
  pcl::toROSMsg (*cloud_p, *final_cloud);

  //----------------------------------------------
  cv::Matx33f R;
  cv::Vec3f T;
  getPlaneTransform(plan_coeff, R, T);

  pose1.pose.position.x = T(1);
  pose1.pose.position.y = T(0);// -T(1);
  pose1.pose.position.z = T(2);
  pose1.pose.orientation = maxtrixToPose(R);
  std::cout << pose1.pose.orientation << std::endl;
  //----------------------------------------------

  cv::Vec3d z1(plan_coeff.values[0], plan_coeff.values[1], plan_coeff.values[2]);
  /*std::cout << "z1 " << z2 << std::endl;
  normalize_vector(z2(0), z2(1), z2(2));
  std::cout << "z1 " << z2 << std::endl;*/

  //Eigen::Quaternion<double>::setFromTwoVectors

  cv::Vec3d z2(0.0, 0.0, 1.0); //c=1, X-Y plane
  //std::cout << "z2 " << z1 << std::endl;
  normalize_vector(z2(0), z2(1), z2(2));
  //std::cout << "z2 " << z1 << std::endl;

  //the angle to rotate with //from the dot product
  double angle = std::acos(z1.dot(z2));
  std::cout << "angle " << angle << std::endl;

  //the axis of rotation //from the cross product
  cv::Vec3d axis = z1.cross(z2);
  normalize_vector(axis(0), axis(1), axis(2));

  toEuler(axis(0), axis(1), axis(2), angle);


  tf::Vector3 axis_vector(plan_coeff.values[0], plan_coeff.values[1], plan_coeff.values[2]);
  tf::Quaternion q(axis_vector, angle);
  q.normalize();
  geometry_msgs::Quaternion orientation;
  tf::quaternionTFToMsg(q, orientation);

  pcl::PointXYZ cloud_center = cloud_p->points[cloud_p->size()/2];
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = pose1.header.frame_id ;
  pose.pose.position.x = cloud_center.x;
  pose.pose.position.y = cloud_center.y;
  pose.pose.position.z = cloud_center.z;
  pose.pose.orientation.x = orientation.x;
  pose.pose.orientation.y = orientation.y;
  pose.pose.orientation.z = orientation.z;
  pose.pose.orientation.w = orientation.w;
  //pub_plane_norm.publish(pose);


  //----------------------------------------------

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
    for (std::vector<double>::const_iterator it = distances.begin();
         it != distances.end(); ++it) {
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
    for (std::vector<double>::const_iterator it = distances.begin();
         it != distances.end(); ++it) {
      dst_full_deviation += std::pow((*it - dst_full_mean), 2);
    }
    dst_full_deviation = std::sqrt(dst_full_deviation / (float)distances.size());
  }

  dst_mean_.push_back(dst_full_mean);
  dst_std_.push_back(dst_full_deviation);
  double dst_mean = 0.0;
  for (int i = 0; i < dst_mean_.size(); ++i)
    dst_mean += dst_mean_[i];
  dst_mean /= dst_mean_.size();
  double dst_std = 0.0;
  for (int i = 0; i < dst_mean_.size(); ++i)
    dst_std += dst_std_[i];
  dst_std /= dst_std_.size();

  std::cout <<  "PC-based Cloud size: "	<< cloud.width << "x" << cloud.height << std::endl
              << "Depth mean: " << depth_mean << std::endl
              << "Inliers nb: " << inliers->indices.size()
              << ", " << inliers->indices.size()/static_cast<float>(cloud.size())
              << "% Plan: "
              << plan_coeff.values[0] << " "
              << plan_coeff.values[1] << " "
              << plan_coeff.values[2] << " "
              << plan_coeff.values[3] << std::endl
              << "Inliers DistanceToPlan Mean/std: " << dst_inliers_mean << "/" << dst_inliers_deviation << std::endl
              << "All pixels DistanceToPlan Mean/std: " << dst_full_mean << "/" << dst_full_deviation << " averaged " << dst_mean << "/" << dst_std << std::endl;
}
