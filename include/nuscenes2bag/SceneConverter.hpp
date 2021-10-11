/*
 * @Author: kinggreat24
 * @Date: 2021-01-28 20:28:36
 * @LastEditTime: 2021-01-29 11:06:32
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /nuscenes2bag/include/nuscenes2bag/SceneConverter.hpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/FileProgress.hpp"
#include "rosbag/bag.h"

#include <Eigen/Core>

namespace nuscenes2bag {

class SceneConverter {
    public:
    SceneConverter(const MetaDataProvider& metaDataProvider);

    void submit(const Token& sceneToken, FileProgress& fileProgress);

    void run(const fs::path& inPath, const fs::path& outDirectoryPath, FileProgress& fileProgress,bool save_rosbag= false);

    private:
    void convertSampleDatas(rosbag::Bag& outBag, const fs::path &inPath, FileProgress& fileProgress);
    void convertEgoPoseInfos(rosbag::Bag& outBag, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo);
    void convertOdomDatas(const fs::path& outDirectoryPath, const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfo,
        const fs::path &inPath,FileProgress& fileProgress);

    bool getEgoPoseByToken(const Token& token, Eigen::Matrix4d& Twc_baselink);

    private:
    const MetaDataProvider& metaDataProvider;
    std::vector<SampleDataInfo> sampleDatas;
    std::vector<EgoPoseInfo> egoPoseInfos;
    SceneId sceneId;
    Token sceneToken;
};

}