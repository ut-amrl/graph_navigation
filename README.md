# AMRL Graph Navigation

[![Build Status](https://github.com/ut-amrl/graph_navigation/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/graph_navigation/actions)

This is the **ROS2 Version** of the AMRL Graph Navigation package. Refer to `ros_noetic` branch for the ROS1 version.

## Overview

The AMRL Graph Navigation package provides a sophisticated navigation system that combines:
- **Graph-based Global Planning**: A* search on topological graphs for long-range navigation
- **Local Obstacle Avoidance**: Dynamic window approach with motion primitives
- **Hierarchical Planning**: Multi-level planning from global to local scales

## System Dependencies

### Core Dependencies
1. [glog](https://github.com/google/glog) - Logging library
1. [gflags](https://github.com/gflags/gflags) - Command-line flag processing
1. [Lua5.1](http://www.lua.org/) - Configuration file processing
1. [Eigen3](https://eigen.tuxfamily.org/) - Linear algebra library
1. [OpenCV](https://opencv.org/) - Computer vision library
1. [Boost](https://www.boost.org/) - C++ utility libraries

### Install System Dependencies
```bash
# Install dependencies automatically
./InstallPackages

# Or manually:
sudo apt update
sudo apt install -y \
    g++ cmake build-essential \
    libgoogle-glog-dev libgflags-dev \
    liblua5.1-dev libeigen3-dev \
    libboost-all-dev libopencv-dev \
    libyaml-cpp-dev python3 python3-pip
```

## ROS2 Dependencies
### Required ROS2 Packages
1. [AMRL Maps](https://github.com/ut-amrl/amrl_maps) - Map representations
1. [AMRL ROS Messages](https://github.com/ut-amrl/amrl_msgs) - Custom message definitions

### Install ROS2 Dependencies
```bash
# Install ROS2 navigation dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-lifecycle \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-cv-bridge
```

## Setup and Build

1.  **Clone this repository and initialize submodules:**
    ```bash
    git clone https://github.com/ut-amrl/graph_navigation.git
    cd graph_navigation
    git submodule update --init --recursive
    ```

2.  **Add the install path to your `AMENT_PREFIX_PATH` in `~/.bashrc`:**
    ```bash
    echo "export AMENT_PREFIX_PATH=$(pwd)/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

3.  **Install dependencies:**
    ```bash
    ./InstallPackages
    ```

4.  **Build and install:**
    ```bash
    make -j$(nproc)
    ```
    This will automatically build and install the package for ROS2. Binaries will be in `install/bin/`.

## Configuration
Configuration files are written in Lua.

By default, the navigation system will try to load the robot configuration file `config/navigation.lua`. To specify a different robot config file, use the `--robot_config` flag (e.g., `--robot_config config/my_robot.lua`).

The base configuration directory is assumed to be `config`, but it can be overridden using the `--config_dir` flag. For example:
```bash
ros2 run graph_navigation navigation \
  --config_dir ~/robot_configs \
  --robot_config robot1.lua
```
This will load the `~/robot_configs/robot1.lua` file. 

The robot configuration file defines the ROS2 topics to listen to, initialization conditions, and navigation algorithm parameters.
```lua
-- Example navigation.lua settings
NavigationParameters = {
  dt = 0.025,                    -- Control loop frequency
  max_linear_speed = 1.0,        -- Maximum linear velocity (m/s)
  max_angular_speed = 1.0,       -- Maximum angular velocity (rad/s)
  obstacle_margin = 0.15,        -- Safety margin around obstacles (m)
  carrot_dist = 1.5,            -- Lookahead distance (m)
  robot_width = 0.44,           -- Robot width (m)
  robot_length = 0.5,           -- Robot length (m)
}
```

## Command-Line Flags

The `ros2 run graph_navigation navigation` executable supports the following command-line flags:

| Long Option         | Short | Argument Type | Description                                               |
|---------------------|-------|---------------|-----------------------------------------------------------|
| `--robot_config`    |       | STRING        | Robot config file (default: `config/navigation.lua`)    |
| `--maps_dir`        |       | STRING        | Directory containing AMRL maps                            |
| `--map`             |       | STRING        | Name of navigation map file (default: `UT_Campus`)        |
| `--twist_drive_topic` |     | STRING        | Drive Command Topic (default: `navigation/cmd_vel`)     |
| `--no_joystick`     |       | NONE          | Whether to use a joystick or not                          |
| `--no_intermed`     |       | NONE          | Whether to disable intermediate planning                  |
| `--debug_images`    |       | NONE          | Show debug images                                         |

**Examples:**

- Run with a specific robot config and enable debug images:
  ```bash
  ros2 run graph_navigation navigation \
    --robot_config config/my_robot.lua \
    --maps_dir /path/to/my_maps \
    --map MyMapName \
    --debug_images
  ```

## ROS2 Topics and Services

### Subscribed Topics
- `localization` (amrl_msgs/Localization2DMsg) - Robot pose estimates
- `odom` (nav_msgs/Odometry) - Wheel odometry
- `scan` (sensor_msgs/LaserScan) - Laser scan data
- `human_states` (amrl_msgs/HumanStateArrayMsg) - Human detections for social nav
- `/move_base_simple/goal` (geometry_msgs/PoseStamped) - Navigation goals

### Published Topics
- `cmd_vel` (geometry_msgs/Twist) - Velocity commands
- `ackermann_curvature_drive` (amrl_msgs/AckermannCurvatureDriveMsg) - Ackermann commands
- `visualization` (amrl_msgs/VisualizationMsg) - Navigation visualizations
- `trajectory` (nav_msgs/Path) - Planned path
- `navigation_goal_status` (amrl_msgs/NavStatusMsg) - Navigation status

### Services
- `GraphNav` (graph_navigation/GraphNav) - Global path planning

## Visualization

The navigation system publishes rich visualization data for debugging and monitoring:

1. **Robot visualization**: Current robot pose and safety margins
1. **Path options**: All considered motion primitives
1. **Selected path**: Currently executing trajectory
1. **Global plan**: High-level navigation plan
1. **Obstacles**: Detected obstacles and cost maps

## Code Structure
- `src/navigation/` - Core navigation algorithms
- `src/shared/` - AMRL shared utilities (cross-compatible)
- `src/visualization/` - Visualization utilities
- `scripts/` - Python utilities and examples
- `config/` - Configuration files