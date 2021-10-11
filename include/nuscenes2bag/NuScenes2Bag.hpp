/*
 * @Author: kinggreat24
 * @Date: 2021-01-29 13:43:00
 * @LastEditTime: 2021-01-29 13:43:13
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /nuscenes2bag/include/nuscenes2bag/NuScenes2Bag.hpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once

#include <rosbag/bag.h>

#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/Filesystem.hpp"

#include <boost/optional.hpp>


namespace nuscenes2bag {

struct NuScenes2Bag {

public:
  NuScenes2Bag();

  void convertDirectory(const fs::path &inDatasetPath,
                        const std::string& version,
                        const fs::path &outputRosbagPath,
                        int32_t threadNumber,
                        boost::optional<int32_t> sceneNumberOpt
                        );

  // void convertDirectory(const fs::path &inDatasetPath,
  //                       const std::string& version,
  //                       const fs::path &outputRosbagPath,
  //                       int32_t threadNumber,
  //                       boost::optional<std::vector<int32_t> > sceneNumberOpt
  //                       );

private:
  std::string inDatasetPathString;
};

}