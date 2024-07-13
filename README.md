# N10C 

N10C: our Robot GUI using [GUItar](https://github.com/Scriptor25/GUItar).

![N10C](https://github.com/wggRobotic/N10C/assets/123776648/4eff2922-9cdc-476f-9868-ce21768795a4)

## Build

First clone the repo into your source folder inside your colcon workspace:

```bash
git clone --recurse-submodules https://github.com/wggRobotic/N10C.git
```

After that go to the root of the colcon workspace, build and install:

```bash
colcon build
source install/local_setup.bash
```

Note: to update the submodules, use

```bash
git submodule update --remote --recursive --init
```

If you have trouble with using any joystick/gamepad related functionality, simply configure the project with cmake

```bash
cmake -B build -S .
```

and build the ```update_mappings``` target

```bash
cmake --build build --target update_mappings
```

If the joystick still is not detected as a gamepad, do following steps:

1. Open [mappings.h](deps/GUItar/Dependencies/GLFW/src/mappings.h), search for your controller name/guid and copy the whole line
2. Open [mappings.h.in](deps/GUItar/Dependencies/GLFW/src/mappings.h.in) and paste the line from mappings.h
3. Copy and paste the guid from the command line output from n10c into the mappings.h.in under the system macro you're using, e.g. Linux
4. Rebuild with colcon

## Usage

### ROS2 Run

Here an example command of how to run the node:

```bash 
ros2 run n10c n10c --ros-args -p image0:=/yourFrontCamTopic -p image1:=/yourRearCamTopic -p image2:=/yourMotionCaptureTopic -p image3:=/yourDepthCamTopic -p twist:=/yourTwistTopic -p barcode:=/yourBarcodeMsgsTopic -p enable:=/yourActivatingService
```

Note: all the parameters are optional and default to some topic you can view by using 

```bash
ros2 topic list
```

### ROS2 Launch

It is recommended to write a separate launch file for your application. We provided an example one inside the launch directory. If you want to use this example you'll need the [n10_cam_dif](https://github.com/wggRobotic/N10-CAM-DIF) and [zbar_ros](https://github.com/ros-drivers/zbar_ros) packages.

```bash
ros2 launch n10c n10.launch.py
```
