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
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "config_reader/config_reader.h"
#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "actionlib_msgs/GoalStatus.h"
#include "amrl_msgs/Pose2Df.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "graph_navigation/graphNavSrv.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/CompressedImage.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/util/helpers.h"
#include "shared/ros/ros_helpers.h"
#include "std_msgs/Bool.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization/visualization.h"

#include "motion_primitives.h"
#include "navigation.h"

using actionlib_msgs::GoalStatus;
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
using sensor_msgs::PointCloud;
using Eigen::Vector2f;
using graph_navigation::graphNavSrv;
using geometry_msgs::TwistStamped;
using geometry::kEpsilon;
using navigation::MotionLimits;
using ros_helpers::InitRosHeader;

const string kAmrlMapsDir = ros::package::getPath("amrl_maps");
const string kOpenCVWindow = "Image window";

DEFINE_string(robot_config, "config/navigation.lua", "Robot config file");
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

DEFINE_string(map, "UT_Campus", "Name of navigation map file");
DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");
DEFINE_bool(debug_images, false, "Show debug images");

// DECLARE_int32(v);

bool run_ = true;
bool simulate_ = false;
bool enabled_ = false;
bool received_odom_ = false;
bool received_laser_ = false;
Vector2f goal_ = {0, 0};
Vector2f current_loc_ = {0, 0};
Vector2f current_vel_ = {0, 0};
float current_angle_ = 0;
float goal_angle_ = 0;
navigation::Odom odom_;
vector<Vector2f> point_cloud_;
sensor_msgs::LaserScan last_laser_msg_;
cv::Mat last_image_;
Navigation navigation_;

// Publishers
ros::Publisher ackermann_drive_pub_;
ros::Publisher vis_pub_;
ros::Publisher twist_drive_pub_;
ros::Publisher viz_pub_;
ros::Publisher map_lines_publisher_;
ros::Publisher pose_marker_publisher_;
ros::Publisher nav_status_pub_;
ros::Publisher status_pub_;
ros::Publisher fp_pcl_pub_;
ros::Publisher path_pub_;
ros::Publisher carrot_pub_;
image_transport::Publisher viz_img_pub_;

// Messages
visualization_msgs::Marker line_list_marker_;
visualization_msgs::Marker pose_marker_;
visualization_msgs::Marker target_marker_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;

void EnablerCallback(const std_msgs::Bool& msg) {
  enabled_ = msg.data;
}

navigation::Odom OdomHandler(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 2) {
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
  received_odom_ = true;
  odom_ = OdomHandler(msg);
  navigation_.UpdateOdometry(odom_);
}

void LaserHandler(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 2) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(CONFIG_laser_loc_x, CONFIG_laser_loc_x);
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
  received_laser_ = true;
  LaserHandler(msg);
  navigation_.ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_.SetNavGoal(loc, angle);
  navigation_.Resume();
}

void GoToCallbackAMRL(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f loc(msg.pose.x, msg.pose.y);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), msg.pose.theta);
  navigation_.SetNavGoal(loc, msg.pose.theta);
  navigation_.Resume();
}

bool PlanServiceCb(graphNavSrv::Request &req,
                 graphNavSrv::Response &res) {
  const Vector2f start(req.start.x, req.start.y);
  const Vector2f end(req.end.x, req.end.y);
  const vector<int> plan = navigation_.GlobalPlan(start, end);
  res.plan = plan;
  return true;
}

// Probably a hack, overrides the carrot with a target from elsewhere.
// Primarily used to implement temporary navigation behaviors without
// overriding the global plan.
void OverrideCallback(const amrl_msgs::Pose2Df& msg) {
  const Vector2f loc(msg.x, msg.y);
  navigation_.SetOverride(loc, msg.theta);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  static string map  = "";
  if (FLAGS_v > 2) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_.UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
  if (map != msg.map) {
    map = msg.map;
    navigation_.UpdateMap(navigation::GetMapPath(FLAGS_maps_dir, msg.map));
  }
}

void HaltCallback(const std_msgs::Bool& msg) {
  navigation_.Pause();
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

void PublishNavStatus() {
  GoalStatus status;
  status.status = 1;

  status_pub_.publish(status);
}

void SendCommand(Eigen::Vector2f vel, float ang_vel) {
  geometry_msgs::TwistStamped drive_msg;
  InitRosHeader("base_link", &drive_msg.header);
  drive_msg.header.stamp = ros::Time::now();
  if (!FLAGS_no_joystick && !enabled_) {
    vel.setZero();
    ang_vel = 0;
  }

  drive_msg.twist.angular.x = 0;
  drive_msg.twist.angular.y = 0;
  drive_msg.twist.angular.z = ang_vel;
  drive_msg.twist.linear.x = vel.x();
  drive_msg.twist.linear.y = vel.y();
  drive_msg.twist.linear.z = 0;

  auto ackermann_msg = TwistToAckermann(drive_msg);

  ackermann_drive_pub_.publish(ackermann_msg);
  twist_drive_pub_.publish(drive_msg.twist);
  // This command is going to take effect system latency period after. Hence
  // modify the timestamp to reflect the time when it will take effect.
  navigation_.UpdateCommandHistory(ToTwist(drive_msg));
}

void PublishForwardPredictedPCL(const vector<Vector2f>& pcl) {
  PointCloud fp_pcl_msg;
  fp_pcl_msg.points.resize(pcl.size());
  for (size_t i = 0; i < pcl.size(); ++i) {
    fp_pcl_msg.points[i].x = pcl[i].x();
    fp_pcl_msg.points[i].y = pcl[i].y();
    fp_pcl_msg.points[i].z = 0.324;
  }
  fp_pcl_msg.header.stamp = ros::Time::now();
  fp_pcl_pub_.publish(fp_pcl_msg);
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
  const auto path = navigation_.GetPlanPath();
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
    bool foundCarrot = navigation_.GetCarrot(carrot);
    if (foundCarrot) {
      carrot_pub_.publish(CarrotToNavMsgsPath(carrot));
    }
  }
}

void DrawTarget() {
  const float carrot_dist = navigation_.GetCarrotDist();
  const Eigen::Vector2f target = navigation_.GetTarget();
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(target, 0.2, 0x10E000, msg_copy);
  visualization::DrawArc(
      Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);
  viz_pub_.publish(msg_copy);
  visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);
}

void DrawRobot() {
  const float kRobotLength = navigation_.GetRobotLength();
  const float kRobotWidth = navigation_.GetRobotWidth();
  const float kRearAxleOffset = 0.0;
  const float kObstacleMargin = navigation_.GetObstacleMargin();
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
      navigation_.GetLastPathOptions();
  auto path_options = ToOptions(path_rollouts);
  std::shared_ptr<PathRolloutBase> best_option =
      navigation_.GetOption();
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
        best_arc.clearance,
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

  // p.pose.orientation.w = 1.0;
  // scale.x = 0.02;
  // scale.y = 0.0;
  // scale.z = 0.0;
  // color[0] = 244.0 / 255.0;
  // color[1] = 0.0 / 255.0;
  // color[2] = 156.0 / 255.0;
  // color[3] = 1.0;
  // InitVizMarker(objectLinesMarker, "object_lines", 0, "linelist", p, scale,
      // 0.0, color);
}

void PublishVisualizationMarkers() {
  map_lines_publisher_.publish(line_list_marker_);
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
}

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

void ImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(
        msg, sensor_msgs::image_encodings::BGR8);
    last_image_ = image->image;
  } catch (cv_bridge::Exception& e) {
    fprintf(stderr, "cv_bridge exception: %s\n", e.what());
    return;
  }
  navigation_.ObserveImage(last_image_, msg->header.stamp.toSec());
  // Update GUI Window
  if (FLAGS_debug_images) {
    cv::imshow(kOpenCVWindow, last_image_);
    cv::waitKey(3);
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);

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

  // Load Parameters and Initialize
  navigation::NavigationParameters params;
  LoadConfig(&params);
  navigation_.Initialize(params, map_path);

  // Publishers
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  ackermann_drive_pub_ = n.advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  twist_drive_pub_ = n.advertise<geometry_msgs::Twist>(
      FLAGS_twist_drive_topic, 1);
  status_pub_ = n.advertise<GoalStatus>("navigation_goal_status", 1);
  viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);
  viz_img_pub_ = it_.advertise("vis_image", 1);
  fp_pcl_pub_ = n.advertise<PointCloud>("forward_predicted_pcl", 1);
  path_pub_ = n.advertise<nav_msgs::Path>("trajectory", 1);
  carrot_pub_ = n.advertise<nav_msgs::Path>("carrot", 1, true);

  // Messages
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitSimulatorVizMarkers();

  // Services
  ros::ServiceServer nav_srv =
    n.advertiseService("graphNavSrv", &PlanServiceCb);

  // Subscribers
  ros::Subscriber velocity_sub =
      n.subscribe(CONFIG_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(CONFIG_localization_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(CONFIG_laser_topic, 1, &LaserCallback);
  ros::Subscriber img_sub = 
      n.subscribe(CONFIG_image_topic, 1, &ImageCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
  ros::Subscriber goto_amrl_sub =
      n.subscribe("/move_base_simple/goal_amrl", 1, &GoToCallbackAMRL);
  ros::Subscriber enabler_sub =
      n.subscribe(CONFIG_enable_topic, 1, &EnablerCallback);
  ros::Subscriber halt_sub =
      n.subscribe("halt_robot", 1, &HaltCallback);
  ros::Subscriber override_sub =
      n.subscribe("nav_override", 1, &OverrideCallback);

  std_msgs::Header viz_img_header; // empty viz_img_header
  viz_img_header.stamp = ros::Time::now(); // time
  cv_bridge::CvImage viz_img;
  if (params.evaluator_type == "cost_map") {
    viz_img = cv_bridge::CvImage(viz_img_header, sensor_msgs::image_encodings::RGB8, navigation_.GetVisualizationImage());
  }
  
  RateLoop loop(1.0 / params.dt);
  while (run_ && ros::ok()) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);
    ros::spinOnce();

    // Run Navigation to get commands
    Vector2f cmd_vel(0, 0);
    float cmd_angle_vel(0);
    bool nav_succeeded = navigation_.Run(ros::Time::now().toSec(), cmd_vel, cmd_angle_vel);

    // Publish Nav Status
    PublishNavStatus();

    if(nav_succeeded) {
      // Publish Visualizations
      PublishForwardPredictedPCL(navigation_.GetPredictedCloud());
      DrawRobot();
      DrawTarget();
      DrawPathOptions();
      PublishVisualizationMarkers();
      PublishPath();
      local_viz_msg_.header.stamp = ros::Time::now();
      global_viz_msg_.header.stamp = ros::Time::now();
      viz_pub_.publish(local_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      if (params.evaluator_type == "cost_map") {
        viz_img.image = navigation_.GetVisualizationImage();
        viz_img_pub_.publish(viz_img.toImageMsg());
      }

      // Publish Commands
      SendCommand(cmd_vel, cmd_angle_vel);
    }
    loop.Sleep();
  }
  return 0;
}
