#!/bin/bash

cd ../..
colcon build --packages-select n10c --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
. ./install/local_setup.bash
ros2 launch n10c n10.launch.py
