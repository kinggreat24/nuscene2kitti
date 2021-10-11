# nuScenes2KITTI

 - Ubuntu 18.04 Melodic: ![](https://github.com/clynamen/nuscenes2bag/workflows/ubuntu_1804_melodic/badge.svg)
 - Ubuntu 16.04 Kinetic: ![](https://github.com/clynamen/nuscenes2bag/workflows/ubuntu_1604_kinetic/badge.svg)

Simple C++ tool for converting the [nuScenes](https://www.nuscenes.org/) dataset from [Aptiv](https://www.aptiv.com).

## Install

The `master` branch targets Ubuntu 18.04 and newer.
The `ubuntu_1604` branch uses C++11 and has been tested on Ubuntu 16.04.

The tool is a normal ROS package. Place it under a workspace and build it with catkin.

## Usage

 - **convert single scene** 

   ```shell
   rosrun nuscenes2bag nuscenes2bag --scene_number $1 --dataroot /media/kinggreat24/T7/data/nu_scenes/nuscenes_test  --out /media/kinggreat24/T7/data/nuscenes_bags_test/ --version v1.0-test
   ```

- **convert all scenes**

  ```shell
  rosrun nuscenes2bag nuscenes2bag --dataroot /media/kinggreat24/T7/data/nu_scenes/nuscenes_test  --out /media/kinggreat24/T7/data/nuscenes_bags_test/ --version v1.0-test
  ```

  

