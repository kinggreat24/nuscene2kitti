/*
 * @Author: kinggreat24
 * @Date: 2021-01-28 21:53:01
 * @LastEditTime: 2021-01-29 13:28:54
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /nuscenes2bag/include/nuscenes2bag/MetaDataTypes.hpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#pragma once 

#include <string>
#include <array>

#include <boost/optional.hpp>

#include "nuscenes2bag/ToDebugString.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"

#include "nuscenes2bag/Filesystem.hpp"

namespace nuscenes2bag {

struct SceneInfo {
    Token token; 
    uint32_t sampleNumber;
    SceneId sceneId;
    std::string name;
    std::string description;
    Token firstSampleToken; 
    Token logToken; 
};

struct SampleInfo {
    Token scene_token;
    Token token;
    TimeStamp timeStamp;
};

struct SampleDataInfo {
    // Token scene_token;
    Token token;
    TimeStamp timeStamp;
    Token egoPoseToken;
    Token calibratedSensorToken;
    std::string fileFormat;
    bool isKeyFrame;
    std::string fileName;
};

struct CalibratedSensorInfo {
    Token token;
    Token sensorToken;
    double translation[3];
    double rotation[4];
    boost::optional<IntrinsicsMatrix> cameraIntrinsics;
};

struct CalibratedSensorName {
    Token token;
    std::string name;
    std::string modality;
};

struct CalibratedSensorInfoAndName {
    CalibratedSensorInfo info;
    CalibratedSensorName name;

    inline friend bool operator<(const CalibratedSensorInfoAndName& l, const CalibratedSensorInfoAndName& r)
    {
        return l.info.token < r.info.token;
    }
};

struct EgoPoseInfo {
    Token token;
    TimeStamp timeStamp;
    double translation[3];
    double rotation[4];
};


struct OdomDataInfo{
    CalibratedSensorInfo calibration_info;
    Token egoPoseToken;
    TimeStamp timestamp;
    fs::path sample_file;
};

struct sort_odom_data_by_timestamp
{
    inline bool operator()(const OdomDataInfo* a, const OdomDataInfo* b)
    {
        return (a->timestamp < b->timestamp);
    }
};

template <> std::string to_debug_string(const SceneInfo& t);
template <> std::string to_debug_string(const SampleInfo& t);
template <> std::string to_debug_string(const SampleDataInfo& t);

}