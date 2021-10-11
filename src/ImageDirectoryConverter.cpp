/*
 * @Author: kinggreat24
 * @Date: 2021-01-28 20:44:54
 * @LastEditTime: 2021-01-28 20:47:23
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /nuscenes2bag/src/ImageDirectoryConverter.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>

namespace nuscenes2bag {

boost::optional<sensor_msgs::Image>
readImageFile(const fs::path& filePath) noexcept
{
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    return boost::optional<sensor_msgs::Image>(*msg);

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

  return boost::none;
}

cv::Mat readCvImageFile(const fs::path& filePath) noexcept
{
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_UNCHANGED);
  }catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }
  return image;
}


}