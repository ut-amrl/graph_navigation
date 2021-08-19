#!/bin/bash
tmux new-session -d -s kinect 'cd ~/k4a_ros && ./bin/depth_to_lidar -rgb'
tmux new-session -d -s repub 'rosrun image_transport republish raw in:=/vis_image compressed out:=/vis_image'
rosnode kill navigation