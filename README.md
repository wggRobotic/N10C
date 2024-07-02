# N10C 
N10C is the GUI for our Robot and uses the GUItar library (https://github.com/Scriptor25/GUItar).
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
# ROS2 Run 
```bash 
ros2 run n10c n10c --ros-args -p image0:=/yourFrontCamTopic -p image1:=/yourRearCamTopic -p image2:=/yourMotionCaptureTopic -p image3:=/yourDepthCamTopic -p twist:=/yourTwistTopic -p barcode:=/yourBarcodeMsgsTopic -p enable:=/yourActivatingService

```
