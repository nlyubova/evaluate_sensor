#include "evaluate_sensor/arprocess.hpp"

ArProcessor::ArProcessor():
  markerSize(18.0)
{
}


bool ArProcessor::detectMarkers(cv_bridge::CvImagePtr &img, tf::Transform &transform)
{

  //detection results will go into "markers"
  markers.clear();
  //Ok, let's detect
  mDetector.detect(img->image, markers, camParam, markerSize, false);

}

void ArProcessor::setCamInfo(const cv::Mat &camMatrix,
                            const cv::Mat &distCoeffs,
                            cv::Size size,
                            const std::string &frame_name)
{
  //camMatrix.copyTo(camMatrix_);
  //distCoeffs.copyTo(distCoeffs_);

  //frame_name_ = frame_name;

  camParam = aruco::CameraParameters(camMatrix, distCoeffs, size);
}

