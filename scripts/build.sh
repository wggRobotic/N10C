cd ../..
colcon build --packages-select n10c
source install/local_setup.sh
ros2 run n10c n10c
