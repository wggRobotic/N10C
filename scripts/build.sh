cd ../..
colcon build --packages-select n10c --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
. ~/.bashrc
ros2 run n10c n10c
