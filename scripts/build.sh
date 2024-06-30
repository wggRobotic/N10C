cd ../..
colcon build --packages-select n10c --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
. ./install/setup.sh
ros2 run n10c n10c --ros-args -p twist:=/cmd_vel
#ros2 launch n10c n10.launch.py --debug