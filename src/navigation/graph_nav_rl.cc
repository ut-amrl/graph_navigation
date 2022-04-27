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
\file    graph_nav_rl.cc
\brief   Slim wrapper to simulator ang graph nav for reinforcement learning.
\author  Joydeep Biswas, (C) 2022
*/
//========================================================================

#include <stdio.h>

#include <iostream>

#include "glog/logging.h"

#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "image_transport/image_transport.h"
#include "ros/ros.h"

#include "simulator/simulator.h"
#include "navigation/constant_curvature_arcs.h"
#include "navigation/motion_primitives.h"
#include "navigation/navigation.h"
#include "navigation/graph_nav_rl.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization/visualization.h"

// CLI Flag to enable test obstacle avoidance mode. This will be set to true in
// the Init() function.
DECLARE_bool(test_avoidance);
DECLARE_int32(v);

using amrl_msgs::VisualizationMsg;
using amrl_msgs::AckermannCurvatureDriveMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using motion_primitives::PathRolloutBase;
using motion_primitives::ConstantCurvatureArc;
using navigation::Navigation;
using navigation::PathOption;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;
using geometry_msgs::PoseStamped;
using geometry_msgs::TwistStamped;
using geometry::kEpsilon;
using navigation::MotionLimits;
using ros_helpers::InitRosHeader;

const string kAmrlMapsDir = ros::package::getPath("amrl_maps");
DEFINE_string(robot_config, "config/ut_automata.lua", "Robot config file");
DEFINE_string(maps_dir, kAmrlMapsDir, "Directory containing AMRL maps");
DEFINE_bool(no_joystick, true, "Whether to use a joystick or not");

CONFIG_STRING(image_topic, "NavigationParameters.image_topic");
CONFIG_STRING(laser_topic, "NavigationParameters.laser_topic");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(localization_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");
CONFIG_FLOAT(laser_loc_x, "NavigationParameters.laser_loc.x");
CONFIG_FLOAT(laser_loc_y, "NavigationParameters.laser_loc.y");

DEFINE_string(map, "EmptyMap", "Name of navigation map file");
DEFINE_string(twist_drive_topic, "/cmd_vel", "Drive Command Topic");
DEFINE_bool(debug_images, false, "Show debug images");

namespace {

Simulator* simulator_ptr_ = nullptr;
ros::NodeHandle* ros_nh_ = nullptr;
navigation::Navigation* navigation_ptr_ = nullptr;
double sim_time_ = 0.0;
double sim_step_interval_ = 0.0;

// Message publishers.
image_transport::ImageTransport* image_transport_ptr_ = nullptr;
ros::Publisher viz_pub_;
ros::Publisher path_pub_;
ros::Publisher carrot_pub_;
image_transport::Publisher viz_img_pub_;
ros::Publisher localization_pub_;

// Messages.
amrl_msgs::AckermannCurvatureDriveMsg drive_msg_;
visualization_msgs::Marker line_list_marker_;
visualization_msgs::Marker pose_marker_;
visualization_msgs::Marker target_marker_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
vector<Vector2f> point_cloud_;
vector<Vector2f> executed_trajectory_;
amrl_msgs::AckermannCurvatureDriveMsg robot_cmd_;
}  // namespace

namespace graph_nav_rl {

int LoadCameraCalibrationCV(
    const std::string& calibration_file,
    cv::Mat* camera_mat_ptr,
    cv::Mat* dist_coeffs_cv_ptr,
    cv::Mat* homography_mat_ptr) {
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

  return 0;
}

void LoadConfig(navigation::NavigationParameters* params) {
  #define REAL_PARAM(x) CONFIG_DOUBLE(x, "NavigationParameters."#x);
  #define NATURALNUM_PARAM(x) CONFIG_UINT(x, "NavigationParameters."#x);
  #define STRING_PARAM(x) CONFIG_STRING(x, "NavigationParameters."#x);
  #define BOOL_PARAM(x) CONFIG_BOOL(x, "NavigationParameters."#x);
  REAL_PARAM(dt);
  REAL_PARAM(max_linear_accel);
  REAL_PARAM(max_linear_decel);
  REAL_PARAM(max_linear_speed);
  REAL_PARAM(max_angular_accel);
  REAL_PARAM(max_angular_decel);
  REAL_PARAM(max_angular_speed);
  REAL_PARAM(carrot_dist);
  REAL_PARAM(system_latency);
  REAL_PARAM(obstacle_margin);
  NATURALNUM_PARAM(num_options);
  REAL_PARAM(robot_width);
  REAL_PARAM(robot_length);
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
  STRING_PARAM(camera_calibration_path);

  config_reader::ConfigReader reader({FLAGS_robot_config});
  params->dt = CONFIG_dt;
  params->linear_limits = MotionLimits(
      CONFIG_max_linear_accel,
      CONFIG_max_linear_decel,
      CONFIG_max_linear_speed);
  params->angular_limits = MotionLimits(
      CONFIG_max_angular_accel,
      CONFIG_max_angular_decel,
      CONFIG_max_angular_speed);
  params->carrot_dist = CONFIG_carrot_dist;
  params->system_latency = CONFIG_system_latency;
  params->obstacle_margin = CONFIG_obstacle_margin;
  params->num_options = CONFIG_num_options;
  params->robot_width = CONFIG_robot_width;
  params->robot_length = CONFIG_robot_length;
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

  // TODO Rather than loading camera homography from a file, compute it from camera transformation info
  LoadCameraCalibrationCV(CONFIG_camera_calibration_path, &params->K, &params->D, &params->H);
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  const size_t kMaxTrajectoryLength = 1000;
  static string map  = "";
  if (FLAGS_v > 3) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_ptr_->UpdateLocation(
      Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
  if (map != msg.map) {
    map = msg.map;
    navigation_ptr_->UpdateMap(navigation::GetMapPath(FLAGS_maps_dir, msg.map));
  }
  if (executed_trajectory_.size() > kMaxTrajectoryLength) {
    executed_trajectory_.erase(executed_trajectory_.begin());
  }
  executed_trajectory_.push_back(Vector2f(msg.pose.x, msg.pose.y));
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 3) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation::Odom odom;
  odom.time = msg.header.stamp.toSec();
  odom.orientation = {static_cast<float>(msg.pose.pose.orientation.w),
                      static_cast<float>(msg.pose.pose.orientation.x),
                      static_cast<float>(msg.pose.pose.orientation.y),
                      static_cast<float>(msg.pose.pose.orientation.z)};
  odom.position = {static_cast<float>(msg.pose.pose.position.x),
                   static_cast<float>(msg.pose.pose.position.y),
                   static_cast<float>(msg.pose.pose.position.z)};
  navigation_ptr_->UpdateOdometry(odom);
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 3) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(CONFIG_laser_loc_x, CONFIG_laser_loc_y);
  static float cached_dtheta_ = 0;
  static float cached_angle_min_ = 0;
  static size_t cached_num_rays_ = 0;
  static vector<Vector2f> cached_rays_;
  if (cached_angle_min_ != msg.angle_min ||
      cached_dtheta_ != msg.angle_increment ||
      cached_num_rays_ != msg.ranges.size()) {
    cached_angle_min_ = msg.angle_min;
    cached_dtheta_ = msg.angle_increment;
    cached_num_rays_ = msg.ranges.size();
    cached_rays_.resize(cached_num_rays_);
    for (size_t i = 0; i < cached_num_rays_; ++i) {
      const float a =
          cached_angle_min_ + static_cast<float>(i) * cached_dtheta_;
      cached_rays_[i] = Vector2f(cos(a), sin(a));
    }
  }
  CHECK_EQ(cached_rays_.size(), msg.ranges.size());
  point_cloud_.resize(cached_rays_.size());
  for (size_t i = 0; i < cached_num_rays_; ++i) {
    const float r =
      ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max) ?
      msg.ranges[i] : msg.range_max);
    point_cloud_[i] = r * cached_rays_[i] + kLaserLoc;
  }
  navigation_ptr_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
}


void DrawRobot() {
  const float kRobotLength = navigation_ptr_->GetRobotLength();
  const float kRobotWidth = navigation_ptr_->GetRobotWidth();
  const float kBaseLinkOffset = navigation_ptr_->GetBaseLinkOffset();
  const float kObstacleMargin = navigation_ptr_->GetObstacleMargin();
  {
    // How much the robot's body extends behind of its base link frame.
    const float l1 = -0.5 * kRobotLength - kBaseLinkOffset - kObstacleMargin;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * kRobotLength - kBaseLinkOffset + kObstacleMargin;
    const float w = 0.5 * kRobotWidth + kObstacleMargin;
    visualization::DrawLine(
        Vector2f(l1, w), Vector2f(l1, -w), 0xC0C0C0, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l2, w), Vector2f(l2, -w), 0xC0C0C0, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l1, w), Vector2f(l2, w), 0xC0C0C0, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l1, -w), Vector2f(l2, -w), 0xC0C0C0, local_viz_msg_);
  }

  {
    // How much the robot's body extends behind of its base link frame.
    const float l1 = -0.5 * kRobotLength - kBaseLinkOffset;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * kRobotLength - kBaseLinkOffset;
    const float w = 0.5 * kRobotWidth;
    visualization::DrawLine(
        Vector2f(l1, w), Vector2f(l1, -w), 0x000000, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l2, w), Vector2f(l2, -w), 0x000000, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l1, w), Vector2f(l2, w), 0x000000, local_viz_msg_);
    visualization::DrawLine(
        Vector2f(l1, -w), Vector2f(l2, -w), 0x000000, local_viz_msg_);
  }

  for (size_t i = 0; i + 1 < executed_trajectory_.size(); ++i) {
    const auto& p1 = executed_trajectory_[i];
    const auto& p2 = executed_trajectory_[i + 1];
    visualization::DrawLine(p1, p2, 0xC0C0C0, global_viz_msg_);
  }
}


vector<PathOption> ToOptions(vector<std::shared_ptr<PathRolloutBase>> paths) {
  vector<PathOption> options;
  for (size_t i = 0; i < paths.size(); ++i) {
    const ConstantCurvatureArc arc =
      *reinterpret_cast<ConstantCurvatureArc*>(paths[i].get());
    PathOption option;
    option.curvature = arc.curvature;
    option.free_path_length = arc.Length();
    option.clearance = arc.Clearance();
    option.closest_point = arc.obstruction;
    option.obstacle_constrained = arc.obstacle_constrained;
    options.push_back(option);
  }
  return options;
}

void DrawPathOptions() {
  vector<std::shared_ptr<PathRolloutBase>> path_rollouts =
      navigation_ptr_->GetLastPathOptions();
  auto path_options = ToOptions(path_rollouts);
  std::shared_ptr<PathRolloutBase> best_option =
      navigation_ptr_->GetOption();
  for (const auto& o : path_options) {
    const uint32_t color = (o.obstacle_constrained ? 0x0000FF : 0x00FF00);
    visualization::DrawPathOption(o.curvature,
        o.free_path_length,
        o.clearance,
        color,
        false,
        local_viz_msg_);
    visualization::DrawPoint(o.closest_point, 0xFF0000, local_viz_msg_);
  }
  // for (const auto& p : point_cloud_) {
  //   visualization::DrawPoint(p, 0xFF0000, local_viz_msg_);
  // }
  if (best_option != nullptr) {
    const ConstantCurvatureArc best_arc =
      *reinterpret_cast<ConstantCurvatureArc*>(best_option.get());
    visualization::DrawPathOption(best_arc.curvature,
        best_arc.length,
        best_arc.clearance,
        0xFF0000,
        true,
        local_viz_msg_);
  }
}

void DrawTarget() {
  const float carrot_dist = navigation_ptr_->GetCarrotDist();
  const Eigen::Vector2f target = navigation_ptr_->GetTarget();
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(target, 0.2, 0x10E000, msg_copy);
  visualization::DrawArc(
      Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);
  // viz_pub_.publish(msg_copy);
  visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);
}

nav_msgs::Path CarrotToNavMsgsPath(const Vector2f& carrot) {
  nav_msgs::Path carrotNav;
  carrotNav.header.stamp=ros::Time::now();
  carrotNav.header.frame_id="map";
  geometry_msgs::PoseStamped carrotPose;
  carrotPose.pose.position.x = carrot.x();
  carrotPose.pose.position.y = carrot.y();

  carrotPose.pose.orientation.x = 0;
  carrotPose.pose.orientation.y = 0;
  carrotPose.pose.orientation.z = 0;
  carrotPose.pose.orientation.w = 1;

  carrotPose.header.stamp = ros::Time::now();
  carrotPose.header.frame_id = "map";
  carrotNav.poses.push_back(carrotPose);
  return carrotNav;
}

void PublishPath() {
  const auto path = navigation_ptr_->GetPlanPath();
  if (path.size() >= 2) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp=ros::Time::now();
    path_msg.header.frame_id="map";
    for (size_t i = 0; i < path.size(); i++) {
      geometry_msgs::PoseStamped pose_plan;
      pose_plan.pose.position.x = path[i].loc.x();
      pose_plan.pose.position.y = path[i].loc.y();

      pose_plan.pose.orientation.x = 0;
      pose_plan.pose.orientation.y = 0;
      pose_plan.pose.orientation.z = 0;
      pose_plan.pose.orientation.w = 1;

      pose_plan.header.stamp = ros::Time::now();
      pose_plan.header.frame_id = "map";
      path_msg.poses.push_back(pose_plan);

      path_pub_.publish(path_msg);
    }
    for (size_t i = 1; i < path.size(); i++) {
      visualization::DrawLine(path[i - 1].loc, path[i].loc, 0xFF007F00,
          global_viz_msg_);
    }
    Vector2f carrot;
    bool foundCarrot = navigation_ptr_->GetCarrot(carrot);
    if (foundCarrot) {
      carrot_pub_.publish(CarrotToNavMsgsPath(carrot));
    }
  }
}

navigation::Twist ToTwist(amrl_msgs::AckermannCurvatureDriveMsg msg) {
  navigation::Twist twist;
  twist.time = msg.header.stamp.toSec();
  twist.linear = {msg.velocity, 0, 0};
  twist.angular = {0.0f, 0.0f, msg.velocity * msg.curvature};
  return twist;
}

extern "C" int Test(const char* str) {
  printf("%s\n", str);
  return 42;
}

extern "C"
void Init() {
  int argc = 1;
  char argv0[] = "graph_nav_rl";
  char* argv[] = {argv0};
  ros::init(argc, argv, "ut_automata_rl_simulator");
  FLAGS_test_avoidance = true;
  // FLAGS_v = 3;
  ros_nh_ = new ros::NodeHandle();
  simulator_ptr_ = new Simulator();
  simulator_ptr_->SetStepMode(true);
  simulator_ptr_->Init(*ros_nh_);
  sim_step_interval_ = simulator_ptr_->GetStepInterval();

  // Publishers
  viz_pub_ = ros_nh_->advertise<VisualizationMsg>("visualization", 1);
  image_transport_ptr_ = new image_transport::ImageTransport(*ros_nh_);
  viz_img_pub_ = image_transport_ptr_->advertise("vis_image", 1);
  path_pub_ = ros_nh_->advertise<nav_msgs::Path>("trajectory", 1);
  carrot_pub_ = ros_nh_->advertise<nav_msgs::Path>("carrot", 1, true);
  localization_pub_ =
      ros_nh_->advertise<amrl_msgs::Localization2DMsg>("/localization", 1);

  // Messages
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  robot_cmd_.curvature = 0;
  robot_cmd_.velocity = 0;

  // Load Parameters and Initialize
  navigation::NavigationParameters params;
  LoadConfig(&params);
  navigation_ptr_ = new navigation::Navigation();
  std::string map_path = navigation::GetMapPath(FLAGS_maps_dir, FLAGS_map);
  navigation_ptr_->Initialize(params, map_path);
}

extern "C"
bool Step(int action_size, double* action,
          int observation_size, double* observation) {
  CHECK(simulator_ptr_ != nullptr);
  CHECK(navigation_ptr_ != nullptr);
  CHECK(action != nullptr);
  CHECK(observation != nullptr);

  // Advance simulation time.
  sim_time_ += sim_step_interval_;

  // Get the action from the RL agent.
  CHECK_EQ(action_size, 2);
  // Vector2f rl_action(action[0], action[1]);
  // printf("Action: %f, %f\n", action[0], action[1]);

  // Run Navigation to get commands
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  Vector2f cmd_vel(0, 0);
  float cmd_angle_vel(0);
  const bool success = navigation_ptr_->Run(
      sim_time_, cmd_vel, cmd_angle_vel);
  robot_cmd_.velocity = cmd_vel.x();
  if (fabs(cmd_vel.x()) > 0.01) {
    robot_cmd_.curvature = cmd_angle_vel / cmd_vel.x();
  } else {
    robot_cmd_.curvature = 0;
  }

  // Publish visualizations.
  DrawRobot();
  DrawTarget();
  DrawPathOptions();
  PublishPath();
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);

  // Run one time-step of the simulation.
  nav_msgs::Odometry odom_msg;
  sensor_msgs::LaserScan scan_msg;
  amrl_msgs::Localization2DMsg localization_msg;
  simulator_ptr_->Step(robot_cmd_, &odom_msg, &scan_msg, &localization_msg);
  odom_msg.header.stamp.fromSec(sim_time_);
  scan_msg.header.stamp.fromSec(sim_time_);
  localization_msg.header.stamp.fromSec(sim_time_);
  robot_cmd_.header.stamp.fromSec(sim_time_);
  // This command is going to take effect system latency period after. Hence
  // modify the timestamp to reflect the time when it will take effect.
  navigation_ptr_->UpdateCommandHistory(ToTwist(robot_cmd_));

  // Propagate messages to navigation.
  OdometryCallback(odom_msg);
  LaserCallback(scan_msg);
  LocalizationCallback(localization_msg);
  localization_pub_.publish(localization_msg);
  ros::spinOnce();
  return success;
}

extern "C"
void Reset() {
  CHECK(simulator_ptr_ != nullptr);
  simulator_ptr_->ResetState();
}

}  // namespace graph_nav_rl