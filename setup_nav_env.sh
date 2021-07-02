#!/bin/bash
tmux new-session -d -s kinect 'cd ~/amrl/k4a_ros && ./bin/depth_to_lidar -rgb'
tmux new-session -d -s repub 'ROS_IP=10.0.0.123 rosrun image_transport republish raw in:=/vis_image compressed out:=/vis_image'
rosnode kill navigation
