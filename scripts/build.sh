cd ../..
colcon build --packages-select n10c --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
. ./install/setup.sh
ros2 run n10c n10c --ros-args -p image0:=/image_raw -p enable:=/eduard/enable
