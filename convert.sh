###
 # @Author: kinggreat24
 # @Date: 2021-01-28 19:52:57
 # @LastEditTime: 2021-01-28 19:54:58
 # @LastEditors: kinggreat24
 # @Description: 
 # @FilePath: /nuscenes2bag/convert.sh
 # @可以输入预定的版权声明、个性签名、空行等
### 
echo "Converting scene $1"
rosrun nuscenes2bag nuscenes2bag --scene_number $1 --dataroot /media/kinggreat24/T7/data/nu_scenes/nuscenes_test  --out /media/kinggreat24/T7/data/nuscenes_bags_test/ --version v1.0-test
