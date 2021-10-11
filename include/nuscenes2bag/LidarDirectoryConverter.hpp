/*
 * @Author: kinggreat24
 * @Date: 2021-01-29 09:53:51
 * @LastEditTime: 2021-01-29 09:55:15
 * @LastEditors: kinggreat24
 * @Description:
 * @FilePath: /nuscenes2bag/include/nuscenes2bag/LidarDirectoryConverter.hpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

namespace nuscenes2bag {

boost::optional<sensor_msgs::PointCloud2> readLidarFile(const fs::path& filePath);

std::vector<float> readBinaryPcdFile(const fs::path& filePath);
void writeBinaryPcdFile(const fs::path& filePath, std::vector<float>& binFileValues);
}
