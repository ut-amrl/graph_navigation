//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand, Arthur Zhang (C) 2025
*/
//========================================================================

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "glog/logging.h"
#include <gflags/gflags.h>

// Submodules from Graph Navigation
#include "navigation.h"
#include "navigation_types.h"
#include "navigation_parameters.h"
#include "ros_adapter.h"
#include "navigation_flags.h"

// Submodules from shared
#include "config_reader/config_reader.h"
#include "shared/util/helpers.h"

using std::endl;
using std::string;
using navigation::Navigation;
using navigation::MotionLimits;
using navigation::NavigationParameters;
using navigation::OSMPlannerParameters;
using navigation::RosAdapter;

// Map loading logic
#ifdef ROS1
  #include "ros/ros.h"
  #include "ros/package.h"
  const string kAmrlMapsDir = ros::package::getPath("amrl_maps");
#else
  #include "rclcpp/rclcpp.hpp"
  #include <ament_index_cpp/get_package_share_directory.hpp>
  const std::string kAmrlMapsDir = ament_index_cpp::get_package_share_directory("amrl_maps");
#endif

DEFINE_string(robot_config, "config/navigation.lua", "Robot config file");
DEFINE_string(maps_dir, kAmrlMapsDir, "Directory containing AMRL maps");
DEFINE_string(map, "UT_Campus", "Name of navigation map file");

DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");
DEFINE_bool(do_intermed, false, "Use intermediate goals or not");
DEFINE_bool(no_joystick, true, "Use joystick or not");
DEFINE_bool(debug_images, false, "Show debug images");
DEFINE_bool(simulate, false, "Simulate robot");

bool run_ = true;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

inline std::string GetMapPath(const std::string& dir, const std::string& name) {
  return dir + "/maps/" + name + "/" + name + ".navigation.json";
}

inline std::string GetDeprecatedMapPath(const std::string& dir,
                                        const std::string& name) {
  return dir + "/" + name + "/" + name + ".navigation.txt";
}

int LoadCameraCalibrationCV(const std::string& calibration_file,
  cv::Mat* camera_mat_ptr,
  cv::Mat* dist_coeffs_cv_ptr,
  cv::Mat* homography_mat_ptr,
  cv::Mat* rectification_mat_ptr,
  cv::Mat* new_camera_mat_ptr,
  cv::Mat* projection_mat_ptr) {
  cv::FileStorage camera_settings(calibration_file, cv::FileStorage::READ);

  if (!camera_settings.isOpened()) {
  std::cerr << "Failed to open camera settings file at: " << calibration_file
  << endl;
  return -1;
  }

  cv::FileNode node = camera_settings["K"];
  if (!node.empty() && camera_mat_ptr != nullptr) {
  *camera_mat_ptr = node.mat();
  } else {
  std::cerr << "Camera calibration matrix not read! Check configuration "
  "file is in default yaml format.";
  }

  node = camera_settings["D"];
  if (!node.empty() && dist_coeffs_cv_ptr != nullptr) {
  *dist_coeffs_cv_ptr = node.mat();
  } else {
  std::cerr << "Camera distortion coefficients not read! Check "
  "configuration file is in default yaml format.";
  }

  node = camera_settings["H"];
  if (!node.empty() && homography_mat_ptr != nullptr) {
  *homography_mat_ptr = node.mat();
  } else {
  std::cerr << "Camera homography matrix not read! Check configuration file "
  "is in default yaml format.";
  }

  node = camera_settings["R"];
  if (!node.empty() && rectification_mat_ptr != nullptr) {
  *rectification_mat_ptr = node.mat();
  } else {
  std::cerr << "Camera rectification matrix not read! Check configuration "
  "file is in default yaml format.";
  }

  node = camera_settings["P"];
  if (!node.empty() && new_camera_mat_ptr != nullptr) {
  *new_camera_mat_ptr = node.mat();
  } else {
  std::cerr << "Camera new camera matrix not read! Check configuration file "
  "is in default yaml format.";
  }

  node = camera_settings["W"];
  if (!node.empty() && projection_mat_ptr != nullptr) {
  *projection_mat_ptr = node.mat();
  } else {
  std::cerr << "Camera projection matrix not read! Check configuration file "
  "is in default yaml format.";
  }

  return 0;
}

void LoadOSMPlannerConfig(navigation::OSMPlannerParameters* params) {
  CONFIG_STRING(osrm_file, "OSMPlannerParameters.osrm_file");
  CONFIG_DOUBLE(osrm_path_resolution,
                "OSMPlannerParameters.osrm_path_resolution");
  config_reader::ConfigReader reader({FLAGS_robot_config});
  params->osrm_file = CONFIG_osrm_file;
  params->osrm_path_resolution = CONFIG_osrm_path_resolution;
}

void LoadConfig(navigation::NavigationParameters* params) {
  #define REAL_PARAM(x) CONFIG_DOUBLE(x, "NavigationParameters." #x);
  #define NATURALNUM_PARAM(x) CONFIG_UINT(x, "NavigationParameters." #x);
  #define STRING_PARAM(x) CONFIG_STRING(x, "NavigationParameters." #x);
  #define BOOL_PARAM(x) CONFIG_BOOL(x, "NavigationParameters." #x);
    REAL_PARAM(dt);
    REAL_PARAM(max_linear_accel);
    REAL_PARAM(max_linear_decel);
    REAL_PARAM(max_linear_speed);
    REAL_PARAM(max_angular_accel);
    REAL_PARAM(max_angular_decel);
    REAL_PARAM(max_angular_speed);
    REAL_PARAM(intermediate_goal_tolerance);
    REAL_PARAM(system_latency);
    REAL_PARAM(obstacle_margin);
    NATURALNUM_PARAM(num_options);
    REAL_PARAM(robot_width);
    REAL_PARAM(robot_length);
    REAL_PARAM(robot_wheelbase);
    REAL_PARAM(base_link_offset);
    REAL_PARAM(max_free_path_length);
    REAL_PARAM(max_clearance);
    BOOL_PARAM(can_traverse_stairs);
    BOOL_PARAM(use_map_speed);
    REAL_PARAM(target_dist_tolerance);
    REAL_PARAM(target_vel_tolerance);
    REAL_PARAM(target_angle_tolerance);
    REAL_PARAM(local_fov);
    BOOL_PARAM(use_kinect);
    STRING_PARAM(model_path);
    STRING_PARAM(evaluator_type);
    STRING_PARAM(carrot_planner_type);
    STRING_PARAM(camera_calibration_path);
    REAL_PARAM(local_costmap_resolution);
    REAL_PARAM(max_inflation_radius);
    REAL_PARAM(local_costmap_size);
    REAL_PARAM(min_inflation_radius);
    REAL_PARAM(global_costmap_resolution);
    REAL_PARAM(global_costmap_size_x);
    REAL_PARAM(global_costmap_size_y);
    REAL_PARAM(global_costmap_origin_x);
    REAL_PARAM(global_costmap_origin_y);
    REAL_PARAM(carrot_dist);
    REAL_PARAM(lidar_range_min);
    REAL_PARAM(lidar_range_max);
    REAL_PARAM(replan_dist);
    REAL_PARAM(object_lifespan);
    REAL_PARAM(inflation_coeff);
    REAL_PARAM(distance_weight);
    REAL_PARAM(recovery_carrot_dist);
  
    config_reader::ConfigReader reader({FLAGS_robot_config});
    params->do_intermed = !FLAGS_do_intermed;
    params->dt = CONFIG_dt;
    params->linear_limits =
        MotionLimits(CONFIG_max_linear_accel, CONFIG_max_linear_decel,
                     CONFIG_max_linear_speed);
    params->angular_limits =
        MotionLimits(CONFIG_max_angular_accel, CONFIG_max_angular_decel,
                     CONFIG_max_angular_speed);
    params->intermediate_goal_tolerance = CONFIG_intermediate_goal_tolerance;
    params->system_latency = CONFIG_system_latency;
    params->obstacle_margin = CONFIG_obstacle_margin;
    params->num_options = CONFIG_num_options;
    params->robot_width = CONFIG_robot_width;
    params->robot_length = CONFIG_robot_length;
    params->robot_wheelbase = CONFIG_robot_wheelbase;
    params->base_link_offset = CONFIG_base_link_offset;
    params->max_free_path_length = CONFIG_max_free_path_length;
    params->max_clearance = CONFIG_max_clearance;
    params->can_traverse_stairs = CONFIG_can_traverse_stairs;
    params->use_map_speed = CONFIG_use_map_speed;
    params->target_dist_tolerance = CONFIG_target_dist_tolerance;
    params->target_vel_tolerance = CONFIG_target_vel_tolerance;
    params->target_angle_tolerance = CONFIG_target_angle_tolerance;
    params->local_fov = CONFIG_local_fov;
    params->use_kinect = CONFIG_use_kinect;
    params->model_path = CONFIG_model_path;
    params->evaluator_type = CONFIG_evaluator_type;
    params->carrot_planner_type = CONFIG_carrot_planner_type;
    params->local_costmap_resolution = CONFIG_local_costmap_resolution;
    params->max_inflation_radius = CONFIG_max_inflation_radius;
    params->local_costmap_size = CONFIG_local_costmap_size;
    params->min_inflation_radius = CONFIG_min_inflation_radius;
    params->global_costmap_resolution = CONFIG_global_costmap_resolution;
    params->global_costmap_size_x = CONFIG_global_costmap_size_x;
    params->global_costmap_size_y = CONFIG_global_costmap_size_y;
    params->global_costmap_origin_x = CONFIG_global_costmap_origin_x;
    params->global_costmap_origin_y = CONFIG_global_costmap_origin_y;
    params->carrot_dist = CONFIG_carrot_dist;
    params->lidar_range_min = CONFIG_lidar_range_min;
    params->lidar_range_max = CONFIG_lidar_range_max;
    params->replan_dist = CONFIG_replan_dist;
    params->object_lifespan = CONFIG_object_lifespan;
    params->inflation_coeff = CONFIG_inflation_coeff;
    params->distance_weight = CONFIG_distance_weight;
    params->recovery_carrot_dist = CONFIG_recovery_carrot_dist;
  
    // TODO Rather than loading camera homography from a file, compute it from
    // camera transformation info
    LoadCameraCalibrationCV(CONFIG_camera_calibration_path, &params->K,
                            &params->D, &params->H, &params->R, &params->P,
                            &params->W);
  }

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  signal(SIGINT, SignalHandler);

  // ROS Specific global declarations
  #ifdef ROS1
    ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
  #else
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("navigation");
  #endif

  // Map Loading
  std::string map_path = GetMapPath(FLAGS_maps_dir, FLAGS_map);
  std::string deprecated_path =
      GetDeprecatedMapPath(FLAGS_maps_dir, FLAGS_map);
  if (!FileExists(map_path) && FileExists(deprecated_path)) {
    printf(
        "Could not find navigation map file at %s. An V1 nav-map was found\
            at %s. Please run map_upgrade from vector_display to upgrade this\
            map.\n",
        map_path.c_str(), deprecated_path.c_str());
    return 1;
  } else if (!FileExists(map_path)) {
    printf("Could not find navigation map file at %s.\n", map_path.c_str());
    return 1;
  }

  // Load Navigation Parameters and Initialize
  NavigationParameters params;
  LoadConfig(&params);

  // Load Global Planner Parameters
  OSMPlannerParameters osm_planner_params;
  LoadOSMPlannerConfig(&osm_planner_params);
  
  printf("Loaded OSM Planner Parameters\n");

  // Initialize navigation and ros adaptors
  auto navigation = std::make_shared<Navigation>();
  auto adapter = RosAdapter::create(nh, params);

  printf("Created ROS Adapter and Navigation Node\n");
  navigation->Initialize(params, adapter, FLAGS_maps_dir, FLAGS_map);
  adapter->Initialize(navigation);

  printf("Initialized ROS Adapter and Navigation Node\n");

  if (run_) {
    // Spin me round round baby right round
    adapter->spinLoop();
  }

  printf("Exiting Navigation Main\n");
  return 0;
}
