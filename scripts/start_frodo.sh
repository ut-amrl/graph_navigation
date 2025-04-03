#!/bin/bash

# Default config that we'll use only if the user does NOT specify --robot_config
DEFAULT_CONFIG="config/navigation_frodo_creste.lua"
ROBOT_CONFIG_FLAG="--robot_config"

# 1) Build your packages, source the workspace
colcon build --packages-select amrl_msgs amrl_maps graph_navigation
source install/setup.bash

# 2) Switch to your package’s directory
cd /frodo_autonomy/src/graph_navigation
echo "Current directory: $(pwd)"
echo "Running graph_navigation with the following command:"

if [ -z "$1" ]; then
  echo "No arguments provided. Using the default config: $DEFAULT_CONFIG"
  # If no arguments are provided, use the default config
  ros2 run graph_navigation navigation --robot_config "$DEFAULT_CONFIG"
else
    # If arguments are provided, check if --robot_config is among them
    USER_SPECIFIED_CONFIG=false
    for ARG in "$@"; do
        if [[ "$ARG" == "$ROBOT_CONFIG_FLAG" ]]; then
        USER_SPECIFIED_CONFIG=true
        break
        fi
    done
    
    echo "User specified --robot_config: $USER_SPECIFIED_CONFIG"
    echo "Default config: $DEFAULT_CONFIG"
    echo "User args: $@"
    
    if [ "$USER_SPECIFIED_CONFIG" = false ]; then
        echo "User did NOT specify --robot_config. Using the default config: $DEFAULT_CONFIG"
        echo "ros2 run graph_navigation navigation --robot_config $DEFAULT_CONFIG $@"
        # The user did NOT specify --robot_config, so prepend our default
        # ros2 run graph_navigation navigation \
        # --robot_config "$DEFAULT_CONFIG" \
        # "$@"
    else
        # The user already has --robot_config ... so just pass everything as-is
        echo "ros2 run graph_navigation navigation $@"
        ros2 run graph_navigation navigation "$@"
    fi
fi


# # 3) Check if user already provided --robot_config
# #    If so, just pass all args "$@" directly to ros2 run.
# #    If not, prepend our default --robot_config before the rest of the args.
# USER_SPECIFIED_CONFIG=false
# for ARG in "$@"; do
#   if [[ "$ARG" == "$ROBOT_CONFIG_FLAG" ]]; then
#     USER_SPECIFIED_CONFIG=true
#     break
#   fi
# done

# echo "User specified --robot_config: $USER_SPECIFIED_CONFIG"
# echo "Default config: $DEFAULT_CONFIG"
# echo "User args: $@"
# echo "Running graph_navigation with the following command:"

# if [ "$USER_SPECIFIED_CONFIG" = false ]; then
#   # The user did NOT specify --robot_config, so prepend our default
#   ros2 run graph_navigation navigation \
#     --robot_config "$DEFAULT_CONFIG" \
#     "$@"
# else
#   # The user already has --robot_config ... so just pass everything as-is
#   ros2 run graph_navigation navigation "$@"
# fi
