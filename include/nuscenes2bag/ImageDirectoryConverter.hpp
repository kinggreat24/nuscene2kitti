/*
 * @Author: kinggreat24
 * @Date: 2021-01-28 20:44:36
 * @LastEditTime: 2021-01-28 22:17:14
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /nuscenes2bag/include/nuscenes2bag/ImageDirectoryConverter.hpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once

#include "sensor_msgs/Image.h"
#include "nuscenes2bag/Filesystem.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <boost/optional.hpp>

namespace nuscenes2bag {

boost::optional<sensor_msgs::Image> readImageFile(const fs::path& filePath) noexcept;
cv::Mat readCvImageFile(const fs::path& filePath) noexcept;
}
