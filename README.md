# AMRL Graph Navigation

[![Build Status](https://github.com/ut-amrl/graph_navigation/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/graph_navigation/actions)

**ROS2 Version** - Graph-based navigation system with hierarchical planning for autonomous mobile robots.

## Overview

The AMRL Graph Navigation package provides a sophisticated navigation system that combines:
- **Graph-based Global Planning**: A* search on topological graphs for long-range navigation
- **Local Obstacle Avoidance**: Dynamic window approach with motion primitives
- **Social Navigation**: Human-aware navigation behaviors for crowded environments
- **Hierarchical Planning**: Multi-level planning from global to local scales

## System Dependencies

### Core Dependencies
1. [glog](https://github.com/google/glog) - Logging library
1. [gflags](https://github.com/gflags/gflags) - Command-line flag processing
1. [Lua5.1](http://www.lua.org/) - Configuration file processing
1. [Eigen3](https://eigen.tuxfamily.org/) - Linear algebra library
1. [OpenCV](https://opencv.org/) - Computer vision library
1. [Boost](https://www.boost.org/) - C++ utility libraries

### Optional Dependencies
1. [LibTorch](https://pytorch.org/get-started/locally/) - For deep learning-based cost evaluation
   - Requires the cxx11 ABI version libtorch
   - Install to `/opt/libtorch` or update CMakeLists.txt path
1. **For GPU acceleration**: [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) and [CuDNN](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html)

### Install System Dependencies

#### Ubuntu 20.04/22.04
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

This package requires **ROS2 Humble** or later.

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

## Build Instructions

### Method 1: ROS2 Workspace Build (Recommended)
```bash
# Create workspace and clone dependencies
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the navigation package
git clone https://github.com/ut-amrl/graph_navigation.git
cd graph_navigation
git submodule update --init --recursive

# Clone dependencies (adjust URLs as needed)
cd ../
git clone https://github.com/ut-amrl/amrl_maps.git
git clone https://github.com/ut-amrl/amrl_msgs.git

# Build the workspace
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Method 2: Local Build
```bash
# Clone and build locally
git clone https://github.com/ut-amrl/graph_navigation.git
cd graph_navigation
git submodule update --init --recursive

# Build using make (creates install/ directory)
make

# For parallel build (ensure sufficient RAM)
make -j$(nproc)
```

## Configuration

Navigation parameters are configured via Lua files in the `config/` directory:

- `config/navigation.lua` - Main navigation parameters
- `config/gym_nav.lua` - Social navigation parameters
- `config/camera_calibration.yaml` - Camera calibration (if using vision)

### Key Configuration Parameters

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

## Usage

### Standard Navigation Node
```bash
# Launch main navigation
ros2 run graph_navigation navigation \
  --robot_config config/navigation.lua \
  --maps_dir /path/to/maps \
  --map your_map_name

# With additional options
ros2 run graph_navigation navigation \
  --robot_config config/navigation.lua \
  --maps_dir /path/to/maps \
  --map your_map_name \
  --debug_images \
  --twist_drive_topic cmd_vel
```

### Social Navigation Node
```bash
# Launch social navigation
ros2 run graph_navigation social_nav \
  --robot_config config/gym_nav.lua \
  --maps_dir /path/to/maps \
  --map your_map_name \
  --social_mode
```

### Waypoint Navigation Script
```bash
# Run waypoint navigation
ros2 run graph_navigation waypoint_navigation.py \
  --map your_map_name \
  --waypoints waypoints.json \
  --loop  # Optional: loop through waypoints
```

Example waypoints.json:
```json
[
  {"x": 0.0, "y": 0.0, "theta": 0.0},
  {"x": 5.0, "y": 0.0, "theta": 1.57},
  {"x": 5.0, "y": 5.0, "theta": 3.14}
]
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
- `SocialNav` (graph_navigation/SocialNav) - Social navigation service

## Visualization

The navigation system publishes rich visualization data for debugging and monitoring:

1. **Robot visualization**: Current robot pose and safety margins
1. **Path options**: All considered motion primitives
1. **Selected path**: Currently executing trajectory
1. **Global plan**: High-level navigation plan
1. **Obstacles**: Detected obstacles and cost maps

Use RViz2 to visualize the navigation:
```bash
ros2 run rviz2 rviz2 -d config/navigation.rviz  # If config exists
```

## Development

### Code Structure
- `src/navigation/` - Core navigation algorithms
- `src/social_nav/` - Social navigation extensions  
- `src/shared/` - AMRL shared utilities (cross-compatible)
- `src/visualization/` - Visualization utilities
- `scripts/` - Python utilities and examples
- `config/` - Configuration files

### Building with Debug Info
```bash
# Debug build
make BUILD_TYPE=Debug

# Or with ROS2 workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing
```bash
# Run unit tests (if available)
colcon test --packages-select graph_navigation

# Or using make
make test
```

## Migration from ROS1

This package has been fully migrated from ROS1 to ROS2. Key changes:
- Publishers/Subscribers use ROS2 rclcpp API
- Services use ROS2 service definitions  
- Python scripts use rclpy instead of rospy
- CMakeLists.txt uses ament_cmake instead of rosbuild
- Message types use ROS2 conventions (msg/ and srv/ subdirectories)

The core navigation algorithms remain unchanged and cross-compatible.

## Troubleshooting

### Common Issues

1. **Map not found**: Ensure the maps directory path is correct and contains the specified map files
1. **TF errors**: Check that localization is publishing proper transforms
1. **High CPU usage**: Reduce the number of motion primitives or increase dt parameter
1. **No laser data**: Verify laser topic name and message format

### Debug Options
```bash
# Enable debug visualizations
--debug_images

# Increase verbosity
--v=1  # or --v=2 for more verbose output

# Log to file
--log_dir=/tmp/navigation_logs/
```

## Contributing

Please follow the AMRL coding standards and submit pull requests for review.

## License

This software is released under the GNU Lesser General Public License Version 3. See [LICENSE](LICENSE) for details.

## Citation

If you use this software in your research, please cite:
```bibtex
@software{amrl_graph_navigation,
  title = {AMRL Graph Navigation},
  author = {Biswas, Joydeep and Holtz, Jarrett and others},
  year = {2021},
  url = {https://github.com/ut-amrl/graph_navigation}
}
```
