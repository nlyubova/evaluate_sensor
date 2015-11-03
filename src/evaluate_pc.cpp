#include "evaluate_sensor/evaluate.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>

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
    std::cout << "- - - - - - - - - hist ";
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
  std::cout << "- - - - pointcloud " << mean << " " << var << " " << nbr << " " << min << " " << max << std::endl;
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
}
