#ifndef ARDETECTION_HPP
#define ARDETECTION_HPP

#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco/charuco.hpp>

class Ardetector
{
public:
  Ardetector(const int &dictionaryId, const cv::Scalar &color);
  void init(const std::string &detector_params_file);

  void detectBoard(cv_bridge::CvImagePtr &img);
  bool detectMarkers(cv_bridge::CvImagePtr &img, tf::Transform &transform);

  void createBoard();
  void createMarker();

  void setCamInfo(const cv::Mat &camMatrix,
                              const cv::Mat &distCoeffs,
                              const std::string &frame_name);

  tf::Transform getPose(const cv::Vec3d &rvec, const cv::Vec3d &vvec);

private:
  const bool refindStrategy;
  int squaresX;
  int squaresY;
  float markerSize;
  float squareLength;
  float markerLength;
  int margins;
  float axisLength;
  int dictionaryId_;
  int borderBits;
  cv::Size imageSize;
  bool showRejected;
  int waitTime;
  bool showImage_;
  cv::Scalar color_;
  std::string frame_name_;

  cv::aruco::Dictionary dictionary;
  cv::aruco::CharucoBoard board;

  cv::aruco::DetectorParameters detectorParams;

  cv::Mat camMatrix_, distCoeffs_;

  std::string config_path;

};

#endif // ARDETECTION_HPP
