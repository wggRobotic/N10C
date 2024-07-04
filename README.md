# N10C 
N10C is the GUI for our Robot and uses the GUItar library (https://github.com/Scriptor25/GUItar).
![N10C](https://github.com/wggRobotic/N10C/assets/123776648/4eff2922-9cdc-476f-9868-ce21768795a4)
## Installation 
```bash
#getcode
mkdir -p ~/ros2_ws/src
cd ros2_ws/src
git clone https://github.com/wggRobotic/N10C.git
cd ..
colcon build
source install/local_setup.bash
```
## Usage
### ROS2 Run 
```bash 
ros2 run n10c n10c --ros-args -p image0:=/yourFrontCamTopic -p image1:=/yourRearCamTopic -p image2:=/yourMotionCaptureTopic -p image3:=/yourDepthCamTopic -p twist:=/yourTwistTopic -p barcode:=/yourBarcodeMsgsTopic -p enable:=/yourActivatingService

```
### ROS2 Launch

It is recommended to write a separate launch file for your application. You can be inspired by ours. There you can also see if you want to use ours you need n10_cam_dif (https://github.com/wggRobotic/N10-CAM-DIF) and zbar_ros( https://github.com/ros-drivers/zbar_ros)



```bash
ros2 launch n10c n10.launch.py
```
