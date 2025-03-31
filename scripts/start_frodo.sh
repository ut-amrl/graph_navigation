#!/bin/bash
colcon build --packages-select amrl_msgs amrl_maps graph_navigation
source install/setup.bash
cd /frodo_autonomy/src/graph_navigation && ros2 run graph_navigation navigation --robot_config config/navigation_frodo_creste.lua