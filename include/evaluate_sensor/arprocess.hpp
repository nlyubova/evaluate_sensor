#ifndef ARPROCESS_HPP
#define ARPROCESS_HPP

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

class ArProcessor
{
public:
  ArProcessor();
  bool detectMarkers(cv_bridge::CvImagePtr &img,
                     tf::Transform &transform);

  void setCamInfo(const cv::Mat &camMatrix,
                  const cv::Mat &distCoeffs,
                  cv::Size size,
                  const std::string &frame_name);

  aruco::MarkerDetector mDetector;

  vector<aruco::Marker> markers;

  aruco::CameraParameters camParam;

private:
  double markerSize;
};

#endif // ARPROCESS_HPP
