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
\file    social_main.cc
\brief   Main entry point for social navigation
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "amrl_msgs/HumanStateArrayMsg.h"
#include "amrl_msgs/HumanStateMsg.h"
#include "constant_curvature_arcs.h"
#include "motion_primitives.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "graph_navigation/graphNavSrv.h"
#include "graph_navigation/socialNavSrv.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "nav_msgs/Odometry.h"
#include "amrl_msgs/SocialPipsSrv.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/math/line2d.h"
#include "shared/ros/ros_helpers.h"
#include "std_msgs/Bool.h"
#include "visualization/visualization.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "social_nav.h"

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using amrl_msgs::HumanStateMsg;
using amrl_msgs::HumanStateArrayMsg;
using amrl_msgs::SocialPipsSrv;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Human;
using navigation::Navigation;
using navigation::SocialNav;
using navigation::SocialAction;
using navigation::MotionLimits;
using navigation::PathOption;
using motion_primitives::PathRolloutBase;
using motion_primitives::ConstantCurvatureArc;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using sensor_msgs::PointCloud;
using std::string;
using std::vector;
using Eigen::Vector2f;
using graph_navigation::graphNavSrv;
using graph_navigation::socialNavSrv;
using geometry::Line2f;
using geometry::kEpsilon;
using ros_helpers::InitRosHeader;

// Found in Config File
CONFIG_STRING(laser_topic, "NavigationParameters.laser_topic");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(loc_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");
CONFIG_FLOAT(laser_loc_x, "NavigationParameters.laser_loc.x");
CONFIG_FLOAT(laser_loc_y, "NavigationParameters.laser_loc.y");

// Command Line Flags
DEFINE_bool(service_mode, false, "Listen to a service instead of topics.");
DEFINE_bool(social_mode, true, "Listen to a service instead of topics.");
DEFINE_bool(bag_mode, false, "Publish Drive Commands or not.");
DEFINE_string(topic_prefix, "", "Prefix for robot id.");
DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");

const string kAmrlMapsDir = ros::package::getPath("amrl_maps");
DEFINE_string(maps_dir, kAmrlMapsDir, "Directory containing AMRL maps");
DEFINE_string(map, "GDC1", "Name of navigation map file");
const string kGraphNavDir = ros::package::getPath("graph_navigation");
DEFINE_string(robot_config,
              kGraphNavDir + "/config/gym_nav.lua", "Path to config file");

DEFINE_double(dt, 0.025, "Delta T");
DECLARE_string(helpon);
DECLARE_int32(v);

bool run_ = true;
bool simulate_ = false;
bool enabled_ = true;
bool r_loc_ = false;
bool r_odom_ = false;
bool r_humans_ = false;
bool r_laser_ = false;
SocialNav* navigation_ = nullptr;
vector<Vector2f> point_cloud_;
Vector2f current_loc_(0, 0);
float current_angle_(0);
Vector2f current_vel_(0, 0);
float current_angular_vel_(0);
navigation::Odom odom_;
Vector2f goal_(0, 0);
float goal_angle_(0);
vector<Human> humans_;
bool goal_set_ = false;
int current_action_ = 0;
int follow_target_ = 0;
vector_map::VectorMap map_;

// Publishers
ros::Publisher ackermann_drive_pub_;
ros::Publisher vis_pub_;
ros::Publisher twist_drive_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
ros::Publisher viz_pub_;
ros::Publisher map_lines_publisher_;
ros::Publisher pose_marker_publisher_;
ros::Publisher human_marker_publisher_;

ros::ServiceClient pips_client_;

// Messages
visualization_msgs::Marker line_list_marker_;
visualization_msgs::Marker pose_marker_;
visualization_msgs::Marker human_marker_;
visualization_msgs::MarkerArray human_array_;
visualization_msgs::Marker target_marker_;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
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
}

AckermannCurvatureDriveMsg TwistToAckermann(
    const geometry_msgs::TwistStamped& twist) {
  AckermannCurvatureDriveMsg ackermann_msg;
  ackermann_msg.header = twist.header;
  ackermann_msg.velocity = twist.twist.linear.x;
  if (fabs(ackermann_msg.velocity) < kEpsilon) {
    ackermann_msg.curvature = 0;
  } else {
    ackermann_msg.curvature = twist.twist.angular.z / ackermann_msg.velocity;
  }
  return ackermann_msg;
}

geometry_msgs::TwistStamped AckermannToTwist(
    const AckermannCurvatureDriveMsg& msg) {
  geometry_msgs::TwistStamped  twist_msg;
  twist_msg.header = msg.header;
  twist_msg.twist.linear.x = msg.velocity;
  twist_msg.twist.linear.y = 0;
  twist_msg.twist.linear.z = 0;
  twist_msg.twist.angular.x = 0;
  twist_msg.twist.angular.y = 0;
  twist_msg.twist.angular.z = msg.velocity * msg.curvature;
  return twist_msg;
}

navigation::Twist ToTwist(geometry_msgs::TwistStamped twist_msg) {
  navigation::Twist twist;
  twist.time = twist_msg.header.stamp.toSec();
  twist.linear = {static_cast<float>(twist_msg.twist.linear.x),
                  static_cast<float>(twist_msg.twist.linear.y),
                  static_cast<float>(twist_msg.twist.linear.z)};
  twist.angular = {static_cast<float>(twist_msg.twist.angular.x),
                  static_cast<float>(twist_msg.twist.angular.y),
                  static_cast<float>(twist_msg.twist.angular.z)};
  return twist;
}

bool PlanService(graphNavSrv::Request &req,
                 graphNavSrv::Response &res) {
  const Vector2f start(req.start.x, req.start.y);
  const Vector2f end(req.end.x, req.end.y);
  const vector<int> plan = navigation_->GlobalPlan(start, end);
  res.plan = plan;
  return true;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  r_loc_ = true;
  static string map  = "";
  current_loc_ = {msg.pose.x, msg.pose.y};
  current_angle_ = msg.pose.theta;
}

void GoalCallback(const amrl_msgs::Pose2Df& msg) {
  goal_set_ = true;
  goal_ = {msg.x, msg.y};
  goal_angle_ = msg.theta;
}

void VelocityCb(const geometry_msgs::Twist& msg) {
  current_vel_ = {msg.linear.x, msg.linear.y};
  current_angular_vel_ = msg.angular.z;
}

navigation::Odom OdomHandler(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
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
  return odom;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  r_odom_ = true;
  odom_ = OdomHandler(msg);
}

void LaserHandler(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
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
}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  r_laser_ = true;
  LaserHandler(msg);
}

vector<Human> ToHumans(const vector<geometry_msgs::Pose2D>& poses,
                       const vector<geometry_msgs::Pose2D>& vels) {
  vector<Human> humans;
  for (size_t i = 0; i < poses.size(); ++i) {
    Human human;
    const auto pose = poses[i];
    const auto vel = vels[i];
    human.id = i;
    human.pose = {pose.x, pose.y};
    human.vel = {vel.x, vel.y};
    humans.push_back(human);
  }
  return humans;
}

void HumanCallback(const HumanStateArrayMsg& msg) {
  humans_.clear();
  r_humans_ = true;
  for (const auto& human : msg.human_states) {
    Human new_human;
    new_human.id = human.id;
    new_human.pose = {human.pose.x, human.pose.y};
    new_human.vel = {human.translational_velocity.x,
                     human.translational_velocity.y};
    humans_.push_back(new_human);
  }
}

geometry_msgs::TwistStamped GetTwistMsg(const Vector2f& vel,
                                        const float& ang_vel) {
  geometry_msgs::TwistStamped drive_msg;
  InitRosHeader("base_link", &drive_msg.header);
  drive_msg.header.stamp = ros::Time::now();

  drive_msg.twist.angular.x = 0;
  drive_msg.twist.angular.y = 0;
  drive_msg.twist.angular.z = ang_vel;
  drive_msg.twist.linear.x = vel.x();
  drive_msg.twist.linear.y = vel.y();
  drive_msg.twist.linear.z = 0;
  return drive_msg;
}

void SendCommand(const Vector2f& vel, const float& ang_vel) {
  geometry_msgs::TwistStamped drive_msg;
  InitRosHeader("base_link", &drive_msg.header);
  drive_msg.header.stamp = ros::Time::now();

  drive_msg.twist.angular.x = 0;
  drive_msg.twist.angular.y = 0;
  drive_msg.twist.angular.z = ang_vel;
  drive_msg.twist.linear.x = vel.x();
  drive_msg.twist.linear.y = vel.y();
  drive_msg.twist.linear.z = 0;

  auto ackermann_msg = TwistToAckermann(drive_msg);

  if (!FLAGS_bag_mode && !FLAGS_service_mode) {
    ackermann_drive_pub_.publish(ackermann_msg);
    twist_drive_pub_.publish(drive_msg.twist);
  }
  navigation_->UpdateCommandHistory(ToTwist(drive_msg));
}

vector<amrl_msgs::Pose2Df> HumanPoses(const vector<Human>& humans) {
  vector<amrl_msgs::Pose2Df> output;
  for (const Human& human : humans) {
    amrl_msgs::Pose2Df pose;
    pose.x = human.pose.x();
    pose.y = human.pose.y();
    output.push_back(pose);
  }
  return output;
}

vector<amrl_msgs::Pose2Df> HumanVels(const vector<Human>& humans) {
  vector<amrl_msgs::Pose2Df> output;
  for (const Human& human : humans) {
    amrl_msgs::Pose2Df vel;
    vel.x = human.vel.x();
    vel.y = human.vel.y();
    output.push_back(vel);
  }
  return output;
}

void FillRequest(SocialPipsSrv::Request* req) {
  amrl_msgs::Pose2Df robot_pose, robot_vel, goal_pose, door_pose;
  robot_pose.x = current_loc_.x();
  robot_pose.y = current_loc_.y();
  robot_pose.theta = current_angle_;
  if (!FLAGS_bag_mode) {
    current_vel_ = navigation_->GetGraphNav()->GetVelocity();
    current_angular_vel_ = navigation_->GetGraphNav()->GetAngularVelocity();
  }
  robot_vel.x = current_vel_.x();
  robot_vel.y = current_vel_.y();
  robot_vel.theta = current_angular_vel_;
  const Vector2f local_target = navigation_->GetLocalTarget();
  req->local_target.x = local_target.x();
  req->local_target.y = local_target.y();
  req->robot_poses = {robot_pose};
  req->robot_vels = {robot_vel};
  req->human_poses = HumanPoses(humans_);
  req->human_vels = HumanVels(humans_);

  // TODO(jaholtz) integrate the door detector into navigation, or
  // as a separate package that navigation can depend on, or make nav
  // listen to those messages and handle them.
  req->door_pose.x = 0;
  req->door_pose.y = 0;
  req->door_pose.theta = 0;
  // TODO(jaholtz) make sure this is the 'open' door state
  req->door_state = 0;
  req->robot_state = current_action_;
  req->follow_target = navigation_->GetTargetId();
}

void DrawTarget() {
  const float carrot_dist = navigation_->GetGraphNav()->GetCarrotDist();
  const Eigen::Vector2f target = navigation_->GetGraphNav()->GetTarget();
  const Eigen::Vector2f override = navigation_->GetGraphNav()->GetOverrideTarget();
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(target, 0.2, 0x10E000, msg_copy);
  visualization::DrawArc(
      Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);
  viz_pub_.publish(msg_copy);
  visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);
  visualization::DrawCross(override, 0.2, 0x800080, local_viz_msg_);
  const Eigen::Rotation2Df rot(current_angle_);
  const Vector2f global_target = rot * target + current_loc_;
  const Vector2f global_over = rot * override + current_loc_;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  ros_helpers::DrawCross(global_target, 0.05, color, &target_marker_);
  human_marker_publisher_.publish(target_marker_);
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  ros_helpers::DrawCross(global_over, 0.05, color, &target_marker_);
  human_marker_publisher_.publish(target_marker_);
  target_marker_.points.clear();
  target_marker_.colors.clear();
}

void DrawRobot() {
  const float kRobotLength = 0.5;
  const float kRobotWidth = 0.44;
  const float kRearAxleOffset = 0.0;
  const float kObstacleMargin = navigation_->GetGraphNav()->GetObstacleMargin();
  {
    // How much the robot's body extends behind of its base link frame.
    const float l1 = -0.5 * kRobotLength - kRearAxleOffset - kObstacleMargin;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * kRobotLength - kRearAxleOffset + kObstacleMargin;
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
    const float l1 = -0.5 * kRobotLength - kRearAxleOffset;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * kRobotLength - kRearAxleOffset;
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
    options.push_back(option);
  }
  return options;
}

void DrawPathOptions() {
  vector<std::shared_ptr<PathRolloutBase>> path_rollouts =
      navigation_->GetGraphNav()->GetLastPathOptions();
  auto path_options = ToOptions(path_rollouts);
  std::shared_ptr<PathRolloutBase> best_option =
      navigation_->GetGraphNav()->GetOption();
  for (const auto& o : path_options) {
    visualization::DrawPathOption(o.curvature,
        o.free_path_length,
        o.clearance,
        0x0000FF,
        false,
        local_viz_msg_);
  }
  if (best_option != nullptr) {
    const ConstantCurvatureArc best_arc =
      *reinterpret_cast<ConstantCurvatureArc*>(best_option.get());
    visualization::DrawPathOption(best_arc.curvature,
        best_arc.length,
        0.0,
        0xFF0000,
        true,
        local_viz_msg_);
  }
}

/**
 * Helper method that initializes visualization_msgs::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist,
 *                      linestrip, points
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void InitVizMarker(visualization_msgs::Marker& vizMarker, string ns,
    int id, string type, geometry_msgs::PoseStamped p,
    geometry_msgs::Point32 scale, double duration, vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = ros::Time::now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::Marker::LINE_LIST;
  } else if (type == "linestrip") {
    vizMarker.type = visualization_msgs::Marker::LINE_STRIP;
  } else if (type == "points") {
    vizMarker.type = visualization_msgs::Marker::POINTS;
  } else {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  vizMarker.lifetime = ros::Duration(duration);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::Marker::ADD;
}

void InitSimulatorVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.02;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  InitVizMarker(line_list_marker_, "map_lines", 0, "linelist", p, scale, 0.0,
      color);

  p.pose.position.z = 0.0;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  scale.x = 0.5;
  scale.y = 0.44;
  scale.z = 0.5;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;

  InitVizMarker(pose_marker_,
      "robot_position",
      1,
      "cube",
      p,
      scale,
      0.0,
      color);

  InitVizMarker(human_marker_,
      "human_position",
      1,
      "arrow",
      p,
      scale,
      0.0,
      color);

  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;

  InitVizMarker(target_marker_,
      "targets",
      1,
      "points",
      p,
      scale,
      0.0,
      color);

}

void DrawMap() {
  ros_helpers::ClearMarker(&line_list_marker_);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &line_list_marker_);
  }
}

void PublishVisualizationMarkers() {
  // Fill the map lines list
  if (FLAGS_bag_mode) {
    DrawMap();
    map_lines_publisher_.publish(line_list_marker_);
  }
  tf::Quaternion robotQ = tf::createQuaternionFromYaw(current_angle_);
  const float rear_axle_offset = 0.0;
  pose_marker_.pose.position.x =
    current_loc_.x() -
    cos(current_angle_) * rear_axle_offset;
  pose_marker_.pose.position.y =
    current_loc_.y() -
    sin(current_angle_) * rear_axle_offset;
  pose_marker_.pose.position.z = 0.5 * 0.5;
  pose_marker_.pose.orientation.w = 1.0;
  pose_marker_.pose.orientation.x = robotQ.x();
  pose_marker_.pose.orientation.y = robotQ.y();
  pose_marker_.pose.orientation.z = robotQ.z();
  pose_marker_.pose.orientation.w = robotQ.w();
  pose_marker_publisher_.publish(pose_marker_);
  auto temp_marker = human_marker_;
  temp_marker.action = visualization_msgs::Marker::DELETEALL;
  human_array_.markers.push_back(temp_marker);
  human_marker_publisher_.publish(human_array_);
  human_array_.markers.clear();
  if (humans_.size() > 0) {
    for (const Human& human : humans_) {
      const Vector2f human_pose = human.pose;
      const Eigen::Rotation2Df rot(current_angle_);
      const Vector2f transformed = (rot * human_pose) + current_loc_;
      const auto point1 = ros_helpers::Eigen2DToRosPoint(transformed);
      const Vector2f t_vel = human.vel + current_vel_;
      const auto transformed_vel = (rot * t_vel) + transformed;
      const auto point2 = ros_helpers::Eigen2DToRosPoint(transformed_vel);
      vector<geometry_msgs::Point> points;
      points.push_back(point1);
      human_marker_.points.push_back(point1);
      human_marker_.points.push_back(point2);
      human_marker_.scale.x = 0.1;
      human_marker_.scale.y = 0.1;
      human_marker_.scale.z = 0.0;
      human_array_.markers.push_back(human_marker_);
      human_marker_.points.clear();
      human_marker_.id += 1;
    }
  } else {
    auto temp_marker = human_marker_;
    temp_marker.action = visualization_msgs::Marker::DELETEALL;
    human_array_.markers.push_back(temp_marker);
  }
  human_marker_publisher_.publish(human_array_);
  human_array_.markers.clear();
  human_marker_.id = 1;
}

bool Synced() {
  if (r_loc_ && r_odom_ && r_humans_ && r_laser_) {
    r_loc_ = r_odom_ = r_humans_ = r_laser_ = false;
    return true;
  }
  return false;
}

void RunSocial() {
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  SocialAction action = SocialAction::GoAlone;
  if (FLAGS_social_mode) {
    SocialPipsSrv::Request req;
    SocialPipsSrv::Response res;
    FillRequest(&req);
    pips_client_.call(req, res);
    current_action_ = res.action;
    if (res.action == 0) {
      action = SocialAction::GoAlone;
    } else if (res.action == 1) {
      action = SocialAction::Halt;
    } else if (res.action == 2) {
      action = SocialAction::Follow;
    } else if (res.action == 3) {
      action = SocialAction::Pass;
    }
  }

  Vector2f cmd_vel;
  float cmd_angle_vel;
  navigation_->SetNavGoal({goal_.x(), goal_.y()},
                          goal_angle_);
  navigation_->Run(ros::Time::now().toSec(),
                   action,
                   current_loc_,
                   current_angle_,
                   odom_,
                   point_cloud_,
                   humans_,
                   cmd_vel, cmd_angle_vel);
  SendCommand(cmd_vel, cmd_angle_vel);
  DrawRobot();
  DrawPathOptions();
  DrawTarget();
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

void RunBagfile() {
  if (Synced()) {
    RunSocial();
  }
}

bool SocialService(socialNavSrv::Request &req,
                   socialNavSrv::Response& res) {
  visualization::ClearVisualizationMsg(local_viz_msg_);
  SocialAction action = SocialAction::GoAlone;
  if (req.action == 1) {
    action = SocialAction::Halt;
  } else if (req.action == 2) {
    action = SocialAction::Follow;
  } else if (req.action == 3) {
    action = SocialAction::Pass;
  }

  navigation::Odom odom = OdomHandler(req.odom);
  odom.time += FLAGS_dt;

  Vector2f cmd_vel;
  float cmd_angle_vel;
  navigation_->SetNavGoal({req.goal_pose.x, req.goal_pose.y},
                          req.goal_pose.theta);
  LaserHandler(req.laser);
  navigation_->Run(ros::Time::now().toSec(),
                   action,
                   {req.loc.x, req.loc.y},
                   req.loc.theta,
                   odom,
                   point_cloud_,
                   ToHumans(req.human_poses, req.human_vels),
                   cmd_vel, cmd_angle_vel);
  auto twist = GetTwistMsg(cmd_vel, cmd_angle_vel);
  auto ackermann = TwistToAckermann(twist);
  res.cmd_vel = ackermann.velocity;
  res.cmd_curve = ackermann.curvature;
  res.target_id = navigation_->GetTargetId();
  const Vector2f local_target = navigation_->GetLocalTarget();
  res.local_target.x = local_target.x();
  res.local_target.y = local_target.y();
  SendCommand(cmd_vel, cmd_angle_vel);
  DrawRobot();
  DrawPathOptions();
  DrawTarget();
  // PublishVisualizationMarkers();
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  return true;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;
  navigation_ = new SocialNav();

  // Map Loading
  std::string map_path = navigation::GetMapPath(FLAGS_maps_dir, FLAGS_map);
  std::string deprecated_path =
      navigation::GetDeprecatedMapPath(FLAGS_maps_dir, FLAGS_map);
  if (!FileExists(map_path) && FileExists(deprecated_path)) {
    printf("Could not find navigation map file at %s. An V1 nav-map was found\
            at %s. Please run map_upgrade from vector_display to upgrade this\
            map.\n", map_path.c_str(), deprecated_path.c_str());
    return 1;
  } else if (!FileExists(map_path)) {
    printf("Could not find navigation map file at %s.\n", map_path.c_str());
    return 1;
  }

  navigation::NavigationParameters params;
  LoadConfig(&params);
  navigation_->GetGraphNav()->Initialize(params, map_path);
  if (FLAGS_bag_mode) {
    map_.Load("/home/jaholtz/code/amrl_maps/AHG2/AHG2.vectormap.txt");
    DrawMap();
  }
  // Necessary Publishers
  ackermann_drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  twist_drive_pub_ = n.advertise<geometry_msgs::Twist>(
      FLAGS_twist_drive_topic, 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);
  vis_pub_ =
      n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  map_lines_publisher_ =
      n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  pose_marker_publisher_ =
      n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  human_marker_publisher_ =
      n.advertise<visualization_msgs::MarkerArray>("/human_viz", 6);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");

  // Listen for Necessary Topics
  ros::Subscriber odom_sub =
      n.subscribe(FLAGS_topic_prefix + CONFIG_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_topic_prefix + CONFIG_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_topic_prefix + CONFIG_laser_topic, 1, &LaserCallback);
  ros::Subscriber human_sub =
      n.subscribe("human_states", 1, &HumanCallback);
  ros::Subscriber goal_sub =
      n.subscribe("/set_goal", 1, &GoalCallback);
  ros::Subscriber vel_sub =
    n.subscribe("/robot0/navigation/cmd_vel", 1, &VelocityCb);

  ros::ServiceServer nav_srv =
    n.advertiseService("graphNavSrv", &PlanService);
  ros::ServiceServer social_nav_srv =
    n.advertiseService("socialNavSrv", &SocialService);
  pips_client_ = n.serviceClient<SocialPipsSrv>("SocialPipsSrv");

  InitSimulatorVizMarkers();

  if (FLAGS_service_mode) {
    ros::spin();
  } else {
    RateLoop loop(1.0 / FLAGS_dt);
    while (run_ && ros::ok()) {
      ros::spinOnce();
      if (goal_set_) {
        if (FLAGS_bag_mode) {
          RunBagfile();
        } else {
          RunSocial();
        }
      }
      PublishVisualizationMarkers();
      viz_pub_.publish(local_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      loop.Sleep();
    }
  }
  delete navigation_;
  return 0;
}
