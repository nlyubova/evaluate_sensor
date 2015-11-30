#ifndef PLANE_FITTING_HPP
#define PLANE_FITTING_HPP

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>
#include <Eigen/Core>

class Plane_fitting
{
public:
  Plane_fitting();

  void compute_plane(pcl::PointCloud<pcl::PointXYZ> cloud,
                     sensor_msgs::PointCloud2::Ptr &final_cloud,
                     geometry_msgs::PoseStamped &pose1);

  bool computeRotation(const Eigen::Vector3d& surfaceNormal,
                       const std::string& frameId);

private:
  //! Tf listener
  tf::TransformListener tfListener_;

  //! Reference frame id of the frame for which the
  //! rotation should be estimated. This does not have to be
  //! the frame in which the point clouds are published.
  std::string tiltingFrameId_;

  std::vector <double> dst_mean_;
  std::vector <double> dst_std_;
};

#endif // PLANE_FITTING_HPP
