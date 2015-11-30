#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui/highgui.hpp>

#include "evaluate_sensor/evaluate_depth.hpp"

Evaluator_depth::Evaluator_depth()
{
}

void Evaluator_depth::compute_plane(cv_bridge::CvImagePtr img,
                                    sensor_msgs::PointCloud2::Ptr &final_cloud,
                                    geometry_msgs::PoseStamped &pose1)
{
  // Compute Plan
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cv::MatIterator_<cv::Vec3f> it = img->image.begin<cv::Vec3f>();
  cv::MatIterator_<cv::Vec3f> it_end = img->image.end<cv::Vec3f>();
  for (; it != it_end; ++it) {
    cv::Vec3f px = *it;
    cloud.push_back(pcl::PointXYZ(px[0], px[1], px[2]));
  }

  plane_fitting.compute_plane(cloud, final_cloud, pose1);
}

float Evaluator_depth::compute_hist(cv_bridge::CvImagePtr img,
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

float Evaluator_depth::compute_stat(cv_bridge::CvImagePtr img,
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

void Evaluator_depth::test_corners(cv_bridge::CvImagePtr cv_ptr, const int &w, const int &h, const int &max)
{
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
}

void Evaluator_depth::test_hist(cv_bridge::CvImagePtr cv_ptr, const int &w, const int &h, const int &bins)
{
  std::vector<float> test_val;
  test_val.resize(bins);
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


void Evaluator_depth::test_temp(cv_bridge::CvImagePtr img, const int &w, const int &h)
{
  float noise = 0.0f;
  if (imgPrev.empty())
  {
    img->image.copyTo(imgPrev);
    tempNoise = cv::Mat::zeros(h, w, img->image.type());
  }
  else
  {
    tempNoise += imgPrev - img->image;

    int nbr = 0;
    int pixel_nbr = w*h;
    for (int count = 0; count < pixel_nbr; ++count)
    {
      float tmp = 0.0f;

      if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        tmp = tempNoise.at<float>(count);
      else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        int tmp_int = tempNoise.at<short int>(count);
        tmp = static_cast<float>(tmp_int)/1000.0f;
      }
      else if (img->encoding == sensor_msgs::image_encodings::MONO8)
      {
        short int tmp_int = tempNoise.at<short int>(count);
        tmp = static_cast<float>(tmp_int)/1000.0f;
      }

      if (tmp > 0.0f)
      {
        noise += tmp;
        ++nbr;
      }
    }
    noise /= static_cast<float>(nbr);
    noiseTemp_.push_back(noise); //TODO, check size

    float noiseMean = 0.0;
    for (int i = 0; i < noiseTemp_.size(); ++i)
      noiseMean += noiseTemp_[i];
    noiseMean /= noiseTemp_.size();

    std::cout << "Temporal noise = " << noise << " / " << noiseMean << std::endl;
  }

  //cv::imshow("temp noise", tempNoise);
}

