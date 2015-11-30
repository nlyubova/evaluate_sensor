#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include "evaluate_sensor/ardetection.hpp"

static void help() {
    std::cout << "Create a ChArUco board image" << std::endl;
    std::cout << "Parameters: " << std::endl;
    std::cout << "-o <image> # Output image" << std::endl;
    std::cout << "-w <nsquares> # Number of squares in X direction" << std::endl;
    std::cout << "-h <nsquares> # Number of squares in Y direction" << std::endl;
    std::cout << "-sl <squareLength> # Square side lenght (in pixels)" << std::endl;
    std::cout << "-ml <markerLength> # Marker side lenght (in pixels)" << std::endl;
    std::cout << "-d <dictionary> # DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2, "
         << "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
         << "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
         << "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16" << std::endl;
    std::cout << "[-m <marginSize>] # Margins size (in pixels)"
         << "Default is (squareLength-markerLength)" << std::endl;
    std::cout << "[-bb <int>] # Number of bits in marker borders. Default is 1" << std::endl;
    std::cout << "[-si] # show generated image" << std::endl;
}

/**
 */
static bool readDetectorParameters(std::string filename, cv::aruco::DetectorParameters &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params.minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params.doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params.minOtsuStdDev;
    fs["errorCorrectionRate"] >> params.errorCorrectionRate;
    return true;
}

Ardetector::Ardetector(const int &dictionaryId, const cv::Scalar &color):
  refindStrategy(true),
  squaresX(2),//5),
  squaresY(2),//7),
  markerSize(18.0f),
  squareLength(50.0f), //40.0f),
  markerLength(40.0f), //30.0f),
  margins(squareLength - markerLength),
  axisLength(0.5f * ((float)std::min(squaresX, squaresY) * (squareLength))),
  dictionaryId_(dictionaryId),//cv::aruco::DICT_4X4_50),
  borderBits(1),
  imageSize(squaresX*squareLength + 2*margins, squaresY*squareLength + 2*margins),
  showRejected(false),
  waitTime(10), //0
  showImage_(true),
  color_(color),
  frame_name_("")
{

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  // create charuco board object
  board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);

  createBoard();
  createMarker();
}

void Ardetector::init(const std::string &detector_params_file)
{
  std::cout << " 82 init " << std::endl;
  bool readOk = readDetectorParameters(detector_params_file, detectorParams);
  if(!readOk) {
      std::cerr << "Invalid detector parameters file" << std::endl;
  }
  std::cout << " 87 init " << std::endl;
}

void Ardetector::setCamInfo(const cv::Mat &camMatrix,
                            const cv::Mat &distCoeffs,
                            const std::string &frame_name)
{
  camMatrix.copyTo(camMatrix_);

  distCoeffs.copyTo(distCoeffs_);

  frame_name_ = frame_name;
}

void Ardetector::createBoard()
{
  cv::Mat boardImage;
  board.draw(imageSize, boardImage, margins, borderBits);

  if(showImage_) {
    std::stringstream str;
    str << "board_" << dictionaryId_ << "_" << squaresX << "x" << squaresY << "_" << squareLength << "x" << markerLength << ".png";

    cv::imshow(str.str().c_str(), boardImage);
    cv::waitKey(1);
    cv::imwrite(str.str().c_str(), boardImage);
  }

}

void Ardetector::createMarker()
{
  cv::Mat markerImg;
  cv::aruco::drawMarker(dictionary, dictionaryId_, markerSize, markerImg, borderBits);

  if(showImage_) {
    std::stringstream str;
    str << "marker_" << dictionaryId_ << "_" << markerSize << "x" << markerSize << ".png";

    cv::imshow(str.str().c_str(), markerImg);
    cv::waitKey(1);
    cv::imwrite(str.str().c_str(), markerImg);
  }

}

tf::Transform Ardetector::getPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
{
  //transform rotation
  cv::Mat rvec1 = (cv::Mat_<float>(1, 3) <<
             static_cast<float>(rvec[0]), static_cast<float>(rvec[1]), static_cast<float>(rvec[2]));
  cv::Mat rot(3, 3, CV_32FC1);
  cv::Rodrigues(rvec1, rot);

  tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                     rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                     rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

// transform translation
  tf::Vector3 tf_orig(tvec[0]/1000.0, tvec[1]/1000.0, tvec[2]/1000.0);
  tf::Transform transform = tf::Transform(tf_rot, tf_orig);

  return transform;
}

bool Ardetector::detectMarkers(cv_bridge::CvImagePtr &img, tf::Transform &transform)
{
  std::vector< int > markerIds;
  std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
  std::vector< cv::Vec3d > rvecs, tvecs;

  //std::cout << " **** 138" << detectorParams.adaptiveThreshWinSizeMax << " " << detectorParams.adaptiveThreshWinSizeMin << std::endl;

  // 1- detect markers
  cv::aruco::detectMarkers(img->image, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);

  //std::cout << "+++++++++++++++markerIds.size() = " << markerIds.size() << " " << camMatrix_.total() << std::endl;

  if(showRejected && rejectedMarkers.size() > 0)
    cv::aruco::drawDetectedMarkers(img->image, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));

  if (markerIds.size() <= 0)
    return false;

  // 2- estimate pose
  if (camMatrix_.total() != 0) {
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, camMatrix_, distCoeffs_, rvecs, tvecs);

    // publish to ROS
    transform = getPose(rvecs[0], tvecs[0]);
    /*for(unsigned int i = 0; i < markerCorners.size(); i++)
      poseArray.poses.push_back(getPose(rvecs[i], tvecs[i]));*/
  }

  // 3- draw results
  //cv::Mat imageCopy;
  //img->image.copyTo(imageCopy);
  cv::aruco::drawDetectedMarkers(img->image, markerCorners, markerIds, color_);

  if (camMatrix_.total() != 0)
  for(unsigned int i = 0; i < markerIds.size(); i++)
    cv::aruco::drawAxis(img->image, camMatrix_, distCoeffs_, rvecs[i], tvecs[i], markerSize * 0.5f);

  return true;
}

void Ardetector::detectBoard(cv_bridge::CvImagePtr &img)
{
  std::vector< int > markerIds, charucoIds;
  std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
  std::vector< cv::Point2f > charucoCorners;
  cv::Vec3d rvec, tvec;
  std::vector< cv::Vec3d > rvecs, tvecs;

  // 1- detect markers
  cv::aruco::detectMarkers(img->image, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);

  std::cout << "+++++++++++++++markerIds.size() = " << markerIds.size() << std::endl;

  // refind strategy to detect more markers
  if(refindStrategy)
      cv::aruco::refineDetectedMarkers(img->image, board, markerCorners, markerIds, rejectedMarkers,
                                   camMatrix_, distCoeffs_);

  // interpolate charuco corners
  int interpolatedCorners = 0;
  if(markerIds.size() > 0)
      interpolatedCorners =
          cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img->image, board,
                                           charucoCorners, charucoIds, camMatrix_, distCoeffs_);

  // 2- estimate charuco board pose
  bool validPose = false;
  /*if ((markerIds.size() > 0) && (camMatrix_.total() != 0))
      validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board,
                                                  camMatrix_, distCoeffs_, rvec, tvec);*/
  if ((markerIds.size() > 0) && (camMatrix_.total() != 0))
      validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board,
                                                  camMatrix_, distCoeffs_, rvecs, tvecs);
  std::cout << "+++++++++++++++validPose= " << validPose << std::endl;

  // 3- draw results
  /*cv::Mat imageCopy;
  img->image.copyTo(imageCopy);*/
  if(markerIds.size() > 0) {
      cv::aruco::drawDetectedMarkers(img->image, markerCorners, color_);
  }

  if(showRejected && rejectedMarkers.size() > 0)
      cv::aruco::drawDetectedMarkers(img->image, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));

  if(interpolatedCorners > 0) {
      cv::aruco::drawDetectedCornersCharuco(img->image, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
  }

  if(validPose)
      cv::aruco::drawAxis(img->image, camMatrix_, distCoeffs_, rvecs, tvecs, axisLength);
}
