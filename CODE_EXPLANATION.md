# AMRL Graph Navigation - Code Structure & Architecture

## Repository Overview

This repository implements a sophisticated graph-based navigation system for autonomous mobile robots. The system combines global path planning using A* search on graph maps with local obstacle avoidance using dynamic window approach techniques. It supports both standard autonomous navigation and social navigation behaviors.

## Repository Structure & Component Analysis

### 🗂️ Root Directory Structure

```
graph_navigation/
├── src/                    # Main source code directory
│   ├── navigation/         # Core navigation algorithms
│   ├── shared/            # AMRL shared utilities (ROS1/ROS2 compatible)
│   ├── config_reader/     # Configuration file parser
│   ├── vector_map/        # Map representation and operations
│   ├── visualization/     # Visualization utilities
│   └── third_party/       # External dependencies
├── config/                # Configuration files
├── srv/                   # ROS service definitions
├── CMakeLists.txt         # Build configuration
├── Makefile              # Build wrapper
└── manifest.xml          # ROS1 package manifest
```

## 🧠 Core Algorithm Components

### 1. Navigation Core (`src/navigation/`)

**Core Algorithm Files:**
- **`navigation.cc/.h`** - Main navigation controller class
  - Implements hierarchical planning (global → intermediate → local)
  - Manages robot state, odometry, and sensor data
  - Coordinates between planning layers
  - **Core Algorithm**: YES

- **`graph_domain.h`** - Graph representation for navigation
  - Defines states, edges, and navigation graph structure
  - Supports JSON serialization/deserialization
  - **Core Algorithm**: YES

- **`astar.h`** - A* search implementation
  - Template-based A* for graph domains
  - **Core Algorithm**: YES

- **`motion_primitives.cc/.h`** - Path rollout generation
  - Ackermann motion model implementation
  - Constant curvature arc generation
  - **Core Algorithm**: YES

- **`constant_curvature_arcs.cc/.h`** - Geometric path primitives
  - Arc generation and collision checking
  - **Core Algorithm**: YES

**Path Evaluation:**
- **`linear_evaluator.cc/.h`** - Simple linear cost evaluation
- **`deep_cost_map_evaluator.cc/.h`** - Neural network-based evaluation
- **`image_based_evaluator.cc/.h`** - Base class for image-based evaluation
- **`image_tiler.cc/.h`** - Image processing utilities

**ROS Entry Points (Wrappers):**
- **`navigation_main.cc`** - Main ROS1 node for navigation
  - **ROS Wrapper**: YES - Needs ROS2 conversion
  - Handles ROS subscribers, publishers, services
  - Manages visualization and status publishing

- **`social_main.cc`** - Social navigation ROS1 node
  - **ROS Wrapper**: YES - Needs ROS2 conversion
  - Implements social behaviors (follow, pass, halt)

- **`social_nav.cc/.h`** - Social navigation algorithms
  - **Core Algorithm**: YES with ROS dependencies

### 2. Map Representation (`src/vector_map/`)

- **`vector_map.cc/.h`** - 2D line-based map representation
  - Ray casting and occlusion culling
  - Sensor simulation capabilities
  - **Core Algorithm**: YES

### 3. Visualization (`src/visualization/`)

- **`visualization.cc/.h`** - Visualization utilities
  - **ROS Wrapper**: YES - Needs ROS2 conversion
  - Creates visualization markers and messages

### 4. Shared Libraries (`src/shared/`)

**Status**: ✅ **Already ROS1/ROS2 Compatible**
- Math utilities (`math/`)
- ROS helpers (`ros/`) - Has ROS1/ROS2 compatibility
- General utilities (`util/`)
- No conversion needed

### 5. Configuration (`src/config_reader/`)

**Status**: ✅ **ROS Independent**
- Lua configuration file parser
- No ROS dependencies

## 🔧 ROS Interface Components

### Services (`srv/`)
- **`graphNavSrv.srv`** - Global path planning service
- **`socialNavSrv.srv`** - Social navigation service
- **Status**: Needs ROS2 conversion

### Configuration (`config/`)
- **`navigation.lua`** - Main navigation parameters
- **`gym_nav.lua`** - Gym environment parameters
- **`camera_calibration*.yaml`** - Camera calibration data
- **Status**: ROS independent

## 🚀 ROS Entry Points & Data Flow

### Main ROS Nodes

1. **`navigation_main.cc`** - Primary navigation node
   - **Inputs**: 
     - `/scan` (LaserScan)
     - `/odom` (Odometry)
     - `localization` (Localization2DMsg)
     - `/camera/rgb/image_raw/compressed` (CompressedImage)
     - Goal commands via topics/services
   - **Outputs**:
     - `ackermann_curvature_drive` (AckermannCurvatureDriveMsg)
     - `navigation/cmd_vel` (Twist)
     - `visualization` (VisualizationMsg)
     - `navigation_goal_status` (NavStatusMsg)

2. **`social_main.cc`** - Social navigation node
   - **Additional Inputs**:
     - `human_states` (HumanStateArrayMsg)
   - **Outputs**: Same as navigation_main + social behavior markers

### ROS Dependencies to Convert

**Message Types:**
- `amrl_msgs/*` - Custom AMRL messages
- Standard ROS message types (geometry_msgs, sensor_msgs, etc.)

**ROS1 Specific APIs:**
- `ros::init`, `ros::NodeHandle`, `ros::spin`
- `ros::Publisher`, `ros::Subscriber`
- `ros::ServiceServer`, `ros::ServiceClient`
- `tf::TransformListener`
- `image_transport::ImageTransport`

## 🏗️ Build System

### Current (ROS1)
- **`CMakeLists.txt`** - Uses `rosbuild` macros
- **`Makefile`** - Build wrapper
- **`manifest.xml`** - ROS1 package manifest

### Dependencies
- **ROS1 Packages**: `roscpp`, `rosbag`, `tf`, `image_transport`
- **System Libraries**: `glog`, `gflags`, `lua5.1`, `costmap_2d`
- **External**: LibTorch, OpenCV

## 🌐 AMRL Ecosystem Integration

### Input Dependencies
- **`amrl_maps`** - Map files (`.navigation.json`, `.vectormap.txt`)
- **`amrl_msgs`** - Message definitions
- **Robot Hardware**: Lidar, cameras, odometry
- **Localization System**: Robot pose estimates

### Output Interfaces
- **Motion Commands**: Ackermann drive commands or twist commands
- **Status Information**: Navigation status, goal progress
- **Visualization**: Path plans, obstacles, robot state
- **Services**: Path planning queries

### Integration Points
- **Localization**: Consumes pose estimates from localization system
- **Motion Control**: Sends commands to low-level motion controllers
- **Perception**: Processes lidar and camera data
- **Planning**: Provides global and local path planning services

## 🎯 ROS2 Transition Strategy

### Files Requiring ROS2 Conversion

**High Priority (Core ROS Wrappers):**
1. `src/navigation/navigation_main.cc` - Main navigation node
2. `src/navigation/social_main.cc` - Social navigation node
3. `src/visualization/visualization.cc/.h` - Visualization utilities
4. `srv/*.srv` - Service definitions

**Medium Priority (Configuration):**
6. `CMakeLists.txt` - Build system conversion
7. `package.xml` - Create ROS2 package manifest
8. `manifest.xml` - Remove ROS1 manifest

**Low Priority (Already Compatible):**
- `src/shared/` - Already ROS1/ROS2 compatible
- `src/config_reader/` - ROS independent
- `src/vector_map/` - ROS independent
- Core algorithm files - Minimal ROS dependencies

### Architecture Preservation
- **Core algorithms remain unchanged** - Pure C++ mathematical/algorithmic code
- **ROS wrapper layer conversion** - Replace ROS1 APIs with ROS2 equivalents
- **Configuration system preserved** - Lua-based configuration continues to work
- **Visualization system adapted** - ROS2 visualization message types

## 🔄 Data Flow Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensors       │    │   Localization  │    │   Map Server    │
│   (Lidar,       │────│                 │    │                 │
│    Camera)      │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Navigation Node                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │   Global    │  │Intermediate │  │   Local     │            │
│  │   Planner   │→ │   Planner   │→ │   Planner   │            │
│  │   (A*)      │  │             │  │   (DWA)     │            │
│  └─────────────┘  └─────────────┘  └─────────────┘            │
└─────────────────────────────────────────────────────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Motor         │    │   Visualization │    │   Status &      │
│   Controllers   │    │   (RViz)        │    │   Monitoring    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 📝 Summary

This repository implements a complete autonomous navigation stack with the following key characteristics:

- **Core Algorithms**: Graph-based global planning + local obstacle avoidance
- **ROS Integration**: Comprehensive ROS1 wrapper layer requiring ROS2 conversion
- **Social Navigation**: Advanced behaviors for human-robot interaction
- **Modularity**: Clear separation between algorithms and ROS interfaces
- **Extensibility**: Plugin-based path evaluation (linear, neural network)
- **AMRL Ecosystem**: Integrates with AMRL maps, messages, and hardware systems

The transition to ROS2 should focus on converting the ROS wrapper layer while preserving the core algorithmic components unchanged. 