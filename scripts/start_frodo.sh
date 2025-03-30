#!/bin/bash
colcon build --packages-select amrl_msgs amrl_maps graph_navigation
source install/setup.bash
ros2 run graph_navigation navigation --test_obstacle