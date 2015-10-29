#include <sensor_msgs/image_encodings.h>

#include "evaluate_sensor/evaluate.hpp"

Evaluator::Evaluator():
  nh_(""),
  it_(nh_),
  floatvschar_(false),
  initialized_(false)
{
  std::string topic_depth_img = "/camera/depth_registered/image_raw";
  nh_.getParam("depth_img_topic", topic_depth_img);
  ROS_INFO_STREAM("topic_depth_img: " << topic_depth_img);

  sub_img_ = it_.subscribe(topic_depth_img, 10, &Evaluator::test, this);
}

float Evaluator::compute_hist(cv_bridge::CvImagePtr img, const std::vector<float> &mean_val, const int i_min, const int i_max, const int j_min, const int j_max, bool print)
{
  std::vector<int> mean_hist(mean_val.size()-1, 0);

  //compute hist
  for(int k=0; k<mean_val.size()-1; ++k)
    for(int i=i_min; i<i_max; ++i)
      for(int j=j_min; j<j_max; ++j)
      {
        float tmp = 0.0;
        if (floatvschar_)
          tmp = img->image.at<float>(i*j);
        else
        {
          int tmp_int = img->image.at<short int>(i*j);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }

        if ((tmp >= mean_val[k]) && (tmp < mean_val[k+1]))
          ++mean_hist[k];
      }

  if (print)
  {
    std::cout << "- - - - - - - - - hist ";
    for (std::vector<int>::iterator it=mean_hist.begin(); it!=mean_hist.end(); ++it)
      std::cout << *it << " ";
    std::cout << std::endl;
  }

  //find a bin with most of data
  int max_count = mean_hist[0];
  int max_id = 0;
  for (int i=0; i<=mean_hist.size(); ++i)
    if (max_count < mean_hist[i])
    {
      max_id = i;
      max_count = mean_hist[i];
    }

  //count mean at image corners
  float mean_temp = 0.0f;
  if (max_count > 0)
  {
    for(int i=i_min; i<i_max; i++)
      for(int j=j_min; j<j_max; j++)
      {
        float tmp = 0.0;
        if (floatvschar_)
          tmp = img->image.at<float>(i*j);
        else
        {
          int tmp_int = img->image.at<short int>(i*j);
          tmp = static_cast<float>(tmp_int)/1000.0f;
        }

        if (tmp >= mean_val[max_id])
          mean_temp += tmp;
      }
    mean_temp /= static_cast<float>(max_count);
  }

  return mean_temp;
}

float Evaluator::compute_stat(cv_bridge::CvImagePtr img, int w, int h)
{
  int nbr = 0;
  float mean = 0.0f;
  float min = std::numeric_limits<float>::max();
  float max = 0.0f;

  for (int count = 0; count < w*h; ++count)
  {
    float tmp = 0.0;

    if (floatvschar_)
      tmp = img->image.at<float>(count);
    else
    {
      int tmp_int = img->image.at<short int>(count);
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
  mean /= (float)nbr;
  float var = 0.0f;
  for (int count = 0; count < w*h; ++count)
  {
    float tmp = 0.0;

    if (floatvschar_)
      tmp = img->image.at<float>(count);
    else
    {
      int tmp_int = img->image.at<short int>(count);
      tmp = static_cast<float>(tmp_int)/1000.0f;
    }

    if (tmp > 0.0f)
      var += (tmp-mean) * (tmp-mean);
  }
  var /= static_cast<float>(nbr);
  //std::cout << "- - - - - time mean var nbr min max " << std::endl;
  std::cout << "- - - - - " << mean << " " << var << " " << nbr << " " << min << " " << max << std::endl;
  return max;
}

void Evaluator::test(const sensor_msgs::ImageConstPtr& img)
{
  if (!initialized_)
  {
    initialized_ = true;
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      floatvschar_ = true;
    //else if (img.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  }

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

  std::vector<float> mean_corners(mean_val.size()-1, 0.0);

  //compute the histogram for the corners
  mean_corners[0] = compute_hist(cv_ptr, mean_val, 0, w/3, h/3*2, h);
  mean_corners[1] = compute_hist(cv_ptr, mean_val, w/3*2, w, h/3*2, h);
  mean_corners[2] = compute_hist(cv_ptr, mean_val, 0, w/3, 0, h/3);
  mean_corners[3] = compute_hist(cv_ptr, mean_val, w/3*2, w, 0, h/3);

  for (std::vector<float>::iterator it=mean_corners.begin(); it!=mean_corners.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;

  //compute the histogram for the whole image
  std::vector<float> test_val;
  test_val.resize(51);
  for (int i=0; i<test_val.size();++i)
    test_val[i] = 1.0f * static_cast<float>(i)/test_val.size();
  test_val[0] = 0.12f;
  std::cout << "Histogram bins: ";
  for (std::vector<float>::iterator it=test_val.begin(); it!=test_val.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;

  //compute the histogram for the whole image
  compute_hist(cv_ptr, test_val, 0, w, 0, h, true);
}

