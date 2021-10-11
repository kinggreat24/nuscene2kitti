#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/utils.hpp"

#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"

#include <array>
#include <iostream>
#include <regex>
#include <string>

#include <unistd.h>
#include <fstream>
#include <sys/stat.h>

using namespace std;

namespace nuscenes2bag {

SceneConverter::SceneConverter(const MetaDataProvider& metaDataProvider)
  : metaDataProvider(metaDataProvider)
{}

boost::optional<SampleType>
getSampleType(const std::string& filename)
{
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
    { { "CAM", SampleType::CAMERA },
      { "RADAR", SampleType::RADAR },
      { "LIDAR", SampleType::LIDAR } }
  };
  for (const auto& strAndSampleType : pairs) {
    const auto& str = strAndSampleType.first;
    const auto& sampleType = strAndSampleType.second;
    if (filename.find(str) != string::npos) {
      return boost::optional<SampleType>(sampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return boost::none;
}

template<typename T>
void
writeMsg(const std::string topicName,
         const std::string& frameID,
         const TimeStamp timeStamp,
         rosbag::Bag& outBag,
         boost::optional<T> msgOpt)
{
  if (msgOpt) {
    auto& msg = msgOpt.value();
    msg.header.frame_id = frameID;
    msg.header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg.header.stamp, msg);
  }
}

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void
SceneConverter::submit(const Token& sceneToken, FileProgress& fileProgress)
{

  boost::optional<SceneInfo> sceneInfoOpt =
    metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt);
  SceneInfo& sceneInfo = sceneInfoOpt.value();

  sceneId = sceneInfo.sceneId;
  this->sceneToken = sceneToken;
  sampleDatas  = metaDataProvider.getSceneSampleData(sceneToken);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneToken);
  fileProgress.addToProcess(sampleDatas.size());
}

  
bool SceneConverter::getEgoPoseByToken(const Token& token, Eigen::Matrix4d& Twc_baselink)
{
  bool flag = false;
  for (const auto& egoPose : egoPoseInfos) 
  {
    if(egoPose.token == token)
    {
      const double* rotation       = egoPose.rotation;
      const double* translation    = egoPose.translation;
      Twc_baselink.block<3,3>(0,0) = Eigen::Quaterniond(rotation[0],rotation[1],rotation[2],rotation[3]).toRotationMatrix();
      Twc_baselink.block<3,1>(0,3) = Eigen::Vector3d(translation[0],translation[1],translation[2]);
      flag = true;
      break;
    }
  }

  return flag;
}


void
SceneConverter::run(const fs::path& inPath,
                    const fs::path& outDirectoryPath,
                    FileProgress& fileProgress,
                    bool save_rosbag)
{
  auto sensorInfos = metaDataProvider.getSceneCalibratedSensorInfo(sceneToken);

  if(save_rosbag)
  {
    std::string bagName =
    outDirectoryPath.string() + "/" + std::to_string(sceneId) + ".bag";

    rosbag::Bag outBag;
    outBag.open(bagName, rosbag::bagmode::Write);

    convertEgoPoseInfos(outBag, sensorInfos);
    convertSampleDatas(outBag, inPath, fileProgress);

    outBag.close();
  }
  
  convertOdomDatas(outDirectoryPath,sensorInfos,inPath,fileProgress);
}

void
SceneConverter::convertOdomDatas(
  const fs::path& outDirectoryPath,
  const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos,
  const fs::path& inPath,
  FileProgress& fileProgress)
{
  int cnt = 0;
  
  //创建保存图像、激光以及时间文件的目录
  //(1)scene目录
  std::string scene_dir =
    outDirectoryPath.string() + "/" + std::to_string(sceneId);
  if (access(scene_dir.c_str(), F_OK) == -1) //如果文件夹不存在
    mkdir(scene_dir.c_str(),S_IRWXU);

  //(2)保存图像的目录
  std::string image_dir = scene_dir + string("/image_0");
  if (access(image_dir.c_str(), F_OK) == -1) //如果文件夹不存在
    mkdir(image_dir.c_str(),S_IRWXU);

  //(3)保存激光的目录
  std::string lidar_dir = scene_dir + string("/velodyne");
  if (access(lidar_dir.c_str(), F_OK) == -1) //如果文件夹不存在
    mkdir(lidar_dir.c_str(),S_IRWXU);

  //(4)保存激光的目录
  std::string calibration_dir = scene_dir + string("/calibration");
  if (access(calibration_dir.c_str(), F_OK) == -1) //如果文件夹不存在
    mkdir(calibration_dir.c_str(),S_IRWXU);

  //(5)相机与激光雷达外参
  std::ofstream lidar2cam_fs(calibration_dir + string("/lidar2cam_extrinsic.txt"));

  //(6)相机内参
  std::ofstream cam_intrinsic_fs(calibration_dir + string("/cam_intrinsic.txt"));

  //(4)时间戳文件
  std::ofstream timestamp_fs(scene_dir + string("/times.txt"));
  std::ofstream sync_timestamp_fs(scene_dir + string("/sync_times.txt"));
  std::ofstream velodyne_timestamp_fs(scene_dir + string("/velodynes.txt"));
  std::ofstream cam_front_timestamp_fs(scene_dir + string("/images.txt"));

  //(5)相机姿态的文件
  std::ofstream cam_pose_fs(scene_dir + string("/pose.txt"));

  char file_name[1024] = { 0 };

  //先将图像以及雷达对应的文件名与时间存储下来，然后排序了之后进行保存
  std::vector<OdomDataInfo*> lidar_top_vec;
  std::vector<OdomDataInfo*> cam_front_vec;

  std::cout<<"Scaning sample data..."<<std::endl;
  for (const auto& sampleData : sampleDatas) 
  {
    fs::path sampleFilePath = inPath / sampleData.fileName;

    boost::optional<SampleType> sampleTypeOpt =
      getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();

    CalibratedSensorInfo calibratedSensorInfo =
      metaDataProvider.getCalibratedSensorInfo(
        sampleData.calibratedSensorToken);

    CalibratedSensorName calibratedSensorName =
      metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    std::string sensorName = toLower(calibratedSensorName.name);

    OdomDataInfo *odom_data_info = new OdomDataInfo();
    odom_data_info->calibration_info = calibratedSensorInfo;
    odom_data_info->sample_file      = sampleFilePath;
    odom_data_info->timestamp        = sampleData.timeStamp;
    odom_data_info->egoPoseToken     = sampleData.egoPoseToken;

    if (sampleType == SampleType::CAMERA && sensorName == "cam_front") 
    {
      cam_front_vec.push_back(odom_data_info);
    }
    else if(sampleType == SampleType::LIDAR)
    {
      lidar_top_vec.push_back(odom_data_info);
    }

    fileProgress.addToProcessed(1);
  }

  std::cout<<"There are: "<<cam_front_vec.size()<<" images and "<<lidar_top_vec.size()<<" lidar sweeps"<<std::endl;

  //按照时间排序
  sort(cam_front_vec.begin(), cam_front_vec.end(), sort_odom_data_by_timestamp());
  sort(lidar_top_vec.begin(), lidar_top_vec.end(), sort_odom_data_by_timestamp());
  
  //写入时间文件                      
  char tmp_buffer[128]={0};
  for(auto cam_data = cam_front_vec.begin(); cam_data != cam_front_vec.end(); cam_data++ )
  {
    sprintf(tmp_buffer,"%0.6f",(*cam_data)->timestamp*1e-6);
    cam_front_timestamp_fs << tmp_buffer <<std::endl;
  }

  for(auto lidar_data = lidar_top_vec.begin(); lidar_data != lidar_top_vec.end(); lidar_data++ )
  {
    sprintf(tmp_buffer,"%0.6f",(*lidar_data)->timestamp*1e-6);
    velodyne_timestamp_fs << tmp_buffer <<std::endl;
  }

  std::cout<<"Save sample data to file..."<<std::endl;
  fileProgress.addToProcess(cam_front_vec.size());
  //保存图像以及激光雷达数据
  for(auto cam_data = cam_front_vec.begin(); cam_data != cam_front_vec.end(); cam_data++)
  {
    //对于每一帧图片，寻找一个时间最接近的激光数据
    TimeStamp cam_timestamp = (*cam_data)->timestamp;
    TimeStamp min_diff = 9999999;

    OdomDataInfo* lidar_odom_data = static_cast<OdomDataInfo*>(NULL);

    for(auto lidar_data = lidar_top_vec.begin(); lidar_data != lidar_top_vec.end(); lidar_data++)
    {
      TimeStamp time_diff = abs((*lidar_data)->timestamp - cam_timestamp);
      if(time_diff < min_diff)
      {
        min_diff = time_diff;
        lidar_odom_data = (*lidar_data);
      }  
    }

    //保存图像、激光雷达等数据
    /*************                      (1)保存图像          **************/
    cv::Mat im = readCvImageFile((*cam_data)->sample_file);
    sprintf(file_name,"%s/%06d.png",image_dir.c_str(),cnt);
    cv::imwrite(file_name,im);
    

    /*************                   (2)保存激光点云数据              **************/
    std::vector<float> lidar_points = readBinaryPcdFile(lidar_odom_data->sample_file);
    sprintf(file_name,"%s/%06d.bin",lidar_dir.c_str(),cnt);
    fs::path lidar_save_path(file_name);
    writeBinaryPcdFile(lidar_save_path,lidar_points);


    /*************                   (3)保存相机内参文件                  **************/
    double fx = (*cam_data)->calibration_info.cameraIntrinsics.get().at(0).at(0);
    double fy = (*cam_data)->calibration_info.cameraIntrinsics.get().at(1).at(1);
    double cx = (*cam_data)->calibration_info.cameraIntrinsics.get().at(0).at(2);;
    double cy = (*cam_data)->calibration_info.cameraIntrinsics.get().at(1).at(2);;
    sprintf(file_name, "%0.6f %0.6f %0.6f %0.6f",fx,fy,cx,cy);
    cam_intrinsic_fs <<file_name<<std::endl;


    /*************              (4)保存外参文件(激光雷达到相机的变换)                **************/
    //获取当前帧相对于车体的坐标变换
    Eigen::Matrix4d T_baselink_cam = Eigen::Matrix4d::Identity();
    double* rotation_cam    = (*cam_data)->calibration_info.rotation;
    double* translation_cam = (*cam_data)->calibration_info.translation;
    T_baselink_cam.block<3,3>(0,0) = Eigen::Quaterniond(rotation_cam[0],rotation_cam[1],rotation_cam[2],rotation_cam[3]).toRotationMatrix();
    T_baselink_cam.block<3,1>(0,3) = Eigen::Vector3d(translation_cam[0],translation_cam[1],translation_cam[2]);

    //得到图像时刻车体的世界坐标
    Eigen::Matrix4d T_world_baselink_cam = Eigen::Matrix4d::Identity();
    bool flag = getEgoPoseByToken((*cam_data)->egoPoseToken,T_world_baselink_cam);
    if(!flag)
    {
      std::cerr<<"Can not get the pose of baselink at camera obstain time"<<std::endl;
      return;
    }

    //得到激光时刻车体的世界坐标
    Eigen::Matrix4d T_world_baselink_lidar = Eigen::Matrix4d::Identity();
    flag = getEgoPoseByToken(lidar_odom_data->egoPoseToken,T_world_baselink_lidar);
    if(!flag)
    {
      std::cerr<<"Can not get the pose of baselink at lidar obstain time"<<std::endl;
      return;
    }

    //获取当前激光雷达相对于车体的坐标变换
    Eigen::Matrix4d T_baselink_lidar = Eigen::Matrix4d::Identity();
    double* rotation_lidar    = lidar_odom_data->calibration_info.rotation;
    double* translation_lidar = lidar_odom_data->calibration_info.translation;
    T_baselink_lidar.block<3,3>(0,0) = Eigen::Quaterniond(rotation_lidar[0],rotation_lidar[1],rotation_lidar[2],rotation_lidar[3]).toRotationMatrix();
    T_baselink_lidar.block<3,1>(0,3) = Eigen::Vector3d(translation_lidar[0],translation_lidar[1],translation_lidar[2]);

    //相机的姿态(用于精度评定)
    Eigen::Matrix4d T_world_cam = T_world_baselink_cam * T_baselink_cam;
    for(int i=0;i<3;i++)
      for(int j=0;j<4;j++)
        cam_pose_fs<<T_world_cam(i,j)<<" ";
    cam_pose_fs<<std::endl;

    //lidar到相机的变换
    Eigen::Matrix4d T_cam_lidar = T_baselink_cam.inverse() * T_world_baselink_cam.inverse() * T_world_baselink_lidar * T_baselink_lidar;
    Eigen::Matrix4d T_lidar_cam = T_cam_lidar.inverse();
    lidar2cam_fs<<" T_cam_lidar: ";
    for(int i=0;i<4;i++)
      for(int j=0;j<4;j++)
        lidar2cam_fs <<T_cam_lidar(i,j)<<", ";
    lidar2cam_fs<<std::endl;

    lidar2cam_fs<<" T_lidar_cam: ";
    for(int i=0;i<4;i++)
      for(int j=0;j<4;j++)
        lidar2cam_fs <<T_lidar_cam(i,j)<<", ";
    lidar2cam_fs<<std::endl;
    

    /*************                         (5)保存同步时间戳文件                **************/
    static bool write_header = false;
    if(!write_header)
    {
      sync_timestamp_fs <<"# Image file       timestamp(ms)      Lidar file     timestamp(ms)"<<std::endl;
      write_header = true;
    }
    sprintf(file_name, "%s %0.6f %s %0.6f",(*cam_data)->sample_file.string().c_str(),(*cam_data)->timestamp*1e-6,
      lidar_odom_data->sample_file.string().c_str(),lidar_odom_data->timestamp*1e-6);
    sync_timestamp_fs <<file_name<<std::endl;

    //保存同步后的图像的时间戳
    sprintf(file_name, "%0.6f", (*cam_data)->timestamp*1e-6);
    timestamp_fs << file_name <<std::endl;

    cnt += 1;
    fileProgress.addToProcessed(1);
  }

  cam_front_timestamp_fs.flush();
  cam_front_timestamp_fs.close();

  velodyne_timestamp_fs.flush();
  velodyne_timestamp_fs.close();

  timestamp_fs.flush();
  timestamp_fs.close();

  cam_pose_fs.flush();
  cam_pose_fs.close();

  lidar2cam_fs.flush();
  lidar2cam_fs.close();
}

  void SceneConverter::convertSampleDatas(
    rosbag::Bag & outBag, const fs::path& inPath, FileProgress& fileProgress)
  {
    for (const auto& sampleData : sampleDatas) {
      fs::path sampleFilePath = inPath / sampleData.fileName;

      boost::optional<SampleType> sampleTypeOpt =
        getSampleType(sampleFilePath.string());
      if (!sampleTypeOpt) {
        continue;
      }
      SampleType& sampleType = sampleTypeOpt.value();

      CalibratedSensorInfo calibratedSensorInfo =
        metaDataProvider.getCalibratedSensorInfo(
          sampleData.calibratedSensorToken);
      CalibratedSensorName calibratedSensorName =
        metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
      std::string sensorName = toLower(calibratedSensorName.name);

      if (sampleType == SampleType::CAMERA) {
        auto topicName = sensorName + "/raw";
        auto msg = readImageFile(sampleFilePath);
        writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

      } else if (sampleType == SampleType::LIDAR) {
        auto topicName = sensorName;

        // PointCloud format:
        auto msg = readLidarFile(sampleFilePath); // x,y,z,intensity
        // auto msg = readLidarFileXYZIR(sampleFilePath); //
        // x,y,z,intensity,ring

        writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

      } else if (sampleType == SampleType::RADAR) {
        auto topicName = sensorName;
        auto msg = readRadarFile(sampleFilePath);
        writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

      } else {
        cout << "Unknown sample type" << endl;
      }

      fileProgress.addToProcessed(1);
    }
  }

  geometry_msgs::TransformStamped makeTransform(const char* frame_id,
                                                const char* child_frame_id,
                                                const double* translation,
                                                const double* rotation,
                                                ros::Time stamp = ros::Time(0))
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = std::string(frame_id);
    msg.header.stamp = stamp;
    msg.child_frame_id = std::string(child_frame_id);
    assignArray2Vector3(msg.transform.translation, translation);
    assignArray2Quaternion(msg.transform.rotation, rotation);
    return msg;
  }

  geometry_msgs::TransformStamped makeIdentityTransform(
    const char* frame_id,
    const char* child_frame_id,
    ros::Time stamp = ros::Time(0))
  {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = std::string(frame_id);
    msg.header.stamp = stamp;
    msg.child_frame_id = std::string(child_frame_id);
    msg.transform.rotation.w = 1;
    return msg;
  }

  void SceneConverter::convertEgoPoseInfos(
    rosbag::Bag & outBag,
    const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos)
  {

    //获得传感器到base_link的外参关系
    std::vector<geometry_msgs::TransformStamped> constantTransforms;
    for (const auto& calibratedSensorInfo : calibratedSensorInfos) {
      auto sensorTransform =
        makeTransform("base_link",
                      toLower(calibratedSensorInfo.name.name).c_str(),
                      calibratedSensorInfo.info.translation,
                      calibratedSensorInfo.info.rotation);
      constantTransforms.push_back(sensorTransform);
    }

    // map与odom为同一坐标系
    geometry_msgs::TransformStamped tfMap2Odom =
      makeIdentityTransform("map", "odom");
    constantTransforms.push_back(tfMap2Odom);

    //获取
    const std::string odomTopic = "/odom";
    for (const auto& egoPose : egoPoseInfos) {
      // write odom
      nav_msgs::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
      outBag.write(odomTopic.c_str(), odomMsg.header.stamp, odomMsg);

      // write TFs
      geometry_msgs::TransformStamped tfOdom2Base =
        egoPoseInfo2TransformStamped(egoPose);
      tf::tfMessage tfMsg;
      tfMsg.transforms.push_back(tfOdom2Base);
      for (const auto& constantTransform : constantTransforms) {
        auto constantTransformWithNewStamp = constantTransform;
        constantTransformWithNewStamp.header.stamp = odomMsg.header.stamp;
        tfMsg.transforms.push_back(constantTransformWithNewStamp);
      }
      outBag.write("/tf", odomMsg.header.stamp, tfMsg);
    }
  }
}