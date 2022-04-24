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
\file    simulator.h
\brief   C++ Implementation: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/package.h"

#include "simulator.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "config_reader/config_reader.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/random.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::PoseWithCovarianceStamped;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using std::atan2;
using std::max;
using std::string;
using std::vector;
using vector_map::VectorMap;

DEFINE_bool(localize, false, "Publish localization");

const string kAmrlMapsDir = ros::package::getPath("amrl_maps");

CONFIG_STRING(MapName, "map_name");
CONFIG_FLOAT(CarLength, "car_length");
CONFIG_FLOAT(CarWidth, "car_width");
CONFIG_FLOAT(CarHeight, "car_height");
CONFIG_FLOAT(RearAxleOffset, "rear_axle_offset");
CONFIG_FLOAT(LaserLocX, "laser_loc.x");
CONFIG_FLOAT(LaserLocY, "laser_loc.y");
CONFIG_FLOAT(LaserLocZ, "laser_loc.z");
CONFIG_FLOAT(StartX, "start_x");
CONFIG_FLOAT(StartY, "start_y");
CONFIG_FLOAT(StartAngle, "start_angle");
CONFIG_FLOAT(PublishRate, "publish_rate");
CONFIG_FLOAT(SubSampleRate, "sub_sample_rate");
CONFIG_FLOAT(MinTurnR, "min_turn_radius");
CONFIG_FLOAT(MaxAccel, "max_accel");
CONFIG_FLOAT(MaxSpeed, "max_speed");
CONFIG_FLOAT(LaserStdDev, "laser_noise_stddev");
CONFIG_FLOAT(AngularDriftRate, "angular_drift_rate");
CONFIG_FLOAT(AngularErrorRate, "angular_error_rate");
CONFIG_FLOAT(MaxLaserRange, "max_laser_range");
CONFIG_FLOAT(LaserAngleIncrement, "laser_angle_increment");
CONFIG_FLOAT(LaserFOV, "laser_fov");

string MapNameToFile(const string& map) {
  return kAmrlMapsDir + "/" + map + "/" + map + ".vectormap.txt";
}

Simulator::Simulator() :
    random_(GetMonotonicTime() * 1e6),
    odom_loc_(random_.UniformRandom(-10, 10),
              random_.UniformRandom(-10, 10)),
    odom_angle_(random_.UniformRandom(-M_PI, M_PI)) {
  t_last_cmd_ = GetMonotonicTime();
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
}

Simulator::~Simulator() { }

void Simulator::ResetState() {
  odom_loc_ = Vector2f(random_.UniformRandom(-10, 10),
                       random_.UniformRandom(-10, 10));
  odom_angle_ = random_.UniformRandom(-M_PI, M_PI);
  true_robot_loc_ = Vector2f(CONFIG_StartX, CONFIG_StartY);
  true_robot_angle_ = CONFIG_StartAngle;
  map_name_ = CONFIG_MapName;
  map_.Load(MapNameToFile(CONFIG_MapName));
}

void Simulator::Init(ros::NodeHandle& n) {
  config_reader::ConfigReader reader({"config/simulator.lua"});
  if (kAmrlMapsDir.empty()) {
    fprintf(stderr,
            "ERROR: AMRL maps directory not found. "
            "Make sure your ROS_PACKAGE_PATH environment variable includes "
            "the path to the amrl_maps repository.\n");
    exit(1);
  }
  scan_msg_.header.seq = 0;
  scan_msg_.header.frame_id = "base_laser";
  scan_msg_.angle_min = -0.5 * CONFIG_LaserFOV;
  scan_msg_.angle_max = 0.5 * CONFIG_LaserFOV;
  scan_msg_.range_min = 0.02;
  scan_msg_.range_max = CONFIG_MaxLaserRange;
  scan_msg_.angle_increment = CONFIG_LaserAngleIncrement;
  scan_msg_.intensities.clear();
  scan_msg_.time_increment = 0.0;
  scan_msg_.scan_time = 0.05;

  odom_msg_.header.seq = 0;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_footprint";

  ResetState();
  InitSimulatorVizMarkers();
  DrawMap();

  drive_subscriber_ = n.subscribe(
      "/ackermann_curvature_drive",
      1,
      &Simulator::DriveCallback,
      this);
  init_subscriber_ = n.subscribe(
      "/set_pose", 1, &Simulator::InitalLocationCallback, this);
  odometry_publisher_ = n.advertise<nav_msgs::Odometry>("/odom",1);
  laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
  map_publisher_ = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  robot_marker_publisher_ = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  true_pose_publisher_ = n.advertise<geometry_msgs::PoseStamped>(
      "/simulator_true_pose", 1);
  if (FLAGS_localize) {
    localization_publisher_ = n.advertise<amrl_msgs::Localization2DMsg>(
        "/localization", 1);
    localization_msg_.header.seq = 0;
    localization_msg_.header.frame_id = "map";
  }
  tf_broadcaster_ = new tf::TransformBroadcaster();
}

void Simulator::InitalLocationCallback(
    const amrl_msgs::Localization2DMsg& msg) {
  true_robot_loc_ = Vector2f(msg.pose.x, msg.pose.y);
  true_robot_angle_ = msg.pose.theta;
  if (map_name_ != msg.map) {
    map_.Load(MapNameToFile(msg.map));
    map_name_ = msg.map;
    DrawMap();
  }
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0 @ %s\n",
         true_robot_loc_.x(),
         true_robot_loc_.y(),
         RadToDeg(true_robot_angle_),
         msg.map.c_str());
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
void Simulator::InitVizMarker(
    visualization_msgs::Marker& vizMarker,
    string ns,
    int id,
    string type,
    geometry_msgs::PoseStamped p,
    geometry_msgs::Point32 scale,
    double duration,
    vector<float> color) {

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

void Simulator::InitSimulatorVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.05;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  InitVizMarker(
      line_list_marker_, "map_lines", 0, "linelist", p, scale, 0.0, color);
  line_list_marker_.header.frame_id = "map";

  p.pose.position.z = 0.5 * CONFIG_CarHeight;
  scale.x = CONFIG_CarLength;
  scale.y = CONFIG_CarWidth;
  scale.z = CONFIG_CarHeight;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;
  InitVizMarker(
      robot_pos_marker_, "robot_position", 1, "cube", p, scale, 0.0, color);
  robot_pos_marker_.header.frame_id = "map";
}

void Simulator::DrawMap() {
  ros_helpers::ClearMarker(&line_list_marker_);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &line_list_marker_);
  }
}

void Simulator::PublishOdometry() {
  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.pose.pose.position.x = odom_loc_.x();
  odom_msg_.pose.pose.position.y = odom_loc_.y();
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.x = 0;
  odom_msg_.pose.pose.orientation.y = 0;
  odom_msg_.pose.pose.orientation.z = sin(0.5 * odom_angle_);
  odom_msg_.pose.pose.orientation.w = cos(0.5 * odom_angle_);
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = robot_ang_vel_;
  odom_msg_.twist.twist.linear.x = robot_vel_;
  odom_msg_.twist.twist.linear.y = 0;
  odom_msg_.twist.twist.linear.z = 0.0;

  odometry_publisher_.publish(odom_msg_);

  robot_pos_marker_.pose.position.x =
      true_robot_loc_.x() - cos(true_robot_angle_) * CONFIG_RearAxleOffset;
  robot_pos_marker_.pose.position.y =
      true_robot_loc_.y() - sin(true_robot_angle_) * CONFIG_RearAxleOffset;
  robot_pos_marker_.pose.position.z = 0.5 * CONFIG_CarHeight;
  robot_pos_marker_.pose.orientation.x = 0;
  robot_pos_marker_.pose.orientation.y = 0;
  robot_pos_marker_.pose.orientation.z = sin(0.5 * true_robot_angle_);
  robot_pos_marker_.pose.orientation.w = cos(0.5 * true_robot_angle_);
}

void Simulator::PublishLaser() {
  scan_msg_.header.stamp = ros::Time::now();
  const Vector2f laserRobotLoc(CONFIG_LaserLocX, CONFIG_LaserLocY);
  const Vector2f laserLoc = true_robot_loc_ + Rotation2Df(true_robot_angle_) * laserRobotLoc;

  const int num_rays = static_cast<int>(
      1.0 + (scan_msg_.angle_max - scan_msg_.angle_min) /
      scan_msg_.angle_increment);
  map_.GetPredictedScan(laserLoc,
                        scan_msg_.range_min,
                        scan_msg_.range_max,
                        scan_msg_.angle_min + true_robot_angle_,
                        scan_msg_.angle_max + true_robot_angle_,
                        num_rays,
                        &scan_msg_.ranges);
  for (float& r : scan_msg_.ranges) {
    if (r > scan_msg_.range_max - 0.1) {
      r = scan_msg_.range_max;
      continue;
    }
    r = max<float>(0.0, r + random_.Gaussian(0, CONFIG_LaserStdDev));
  }
  laser_publisher_.publish(scan_msg_);
}

void Simulator::PublishTransform() {
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
"/odom"));

  transform.setOrigin(tf::Vector3(true_robot_loc_.x(), true_robot_loc_.y(), 0.0));
  q.setRPY(0.0,0.0,true_robot_angle_);
  transform.setRotation(q);
  tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom",
"/base_footprint"));

  transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
"/base_footprint", "/base_link"));

  transform.setOrigin(tf::Vector3(CONFIG_LaserLocX, CONFIG_LaserLocY, CONFIG_LaserLocZ));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
"/base_link", "/base_laser"));
}

void Simulator::PublishVisualizationMarkers() {
  map_publisher_.publish(line_list_marker_);
  robot_marker_publisher_.publish(robot_pos_marker_);
}

float AbsBound(float x, float bound) {
  if (x > 0.0 && x > bound) {
    return bound;
  } else if (x < 0.0 && x < -bound) {
    return -bound;
  }
  return x;
}

void Simulator::DriveCallback(const AckermannCurvatureDriveMsg& msg) {
 if (!isfinite(msg.velocity) || !isfinite(msg.curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
           msg.velocity,
           msg.curvature);
    return;
  }
  last_cmd_ = msg;
  t_last_cmd_ = GetMonotonicTime();
}

void Simulator::Update() {
  static const double kMaxCommandAge = 0.1;
  if (!step_mode_ && GetMonotonicTime() > t_last_cmd_ + kMaxCommandAge) {
    last_cmd_.velocity = 0;
  }
  const float dt = CONFIG_SubSampleRate / CONFIG_PublishRate;

  // Epsilon curvature corresponding to a very large radius of turning.
  static const float kEpsilonCurvature = 1.0 / 1E3;
  // Commanded speed bounded to motion limit.
  const float desired_vel = AbsBound(last_cmd_.velocity, CONFIG_MaxSpeed);
  // Maximum magnitude of curvature according to turning limits.
  const float max_curvature = 1.0 / CONFIG_MinTurnR;
  // Commanded curvature bounded to turning limit.
  const float desired_curvature = AbsBound(last_cmd_.curvature, max_curvature);
  // Indicates if the command is for linear motion.
  const bool linear_motion = (fabs(desired_curvature) < kEpsilonCurvature);

  const float dv_max = dt * CONFIG_MaxAccel;
  const float bounded_dv = AbsBound(desired_vel - robot_vel_, dv_max);
  robot_vel_ = robot_vel_ + bounded_dv;
  const float dist = robot_vel_ * dt;

  // Robot-frame uncorrupted motion.
  float dtheta;
  Vector2f dLoc;
  if (linear_motion) {
    dLoc = Vector2f(dist, 0);
    dtheta = 0;
  } else {
    const float r = 1.0 / desired_curvature;
    dtheta = dist * desired_curvature;
    dLoc = r * Vector2f(sin(dtheta), 1.0 - cos(dtheta));
  }

  odom_loc_ += Rotation2Df(odom_angle_) * dLoc;
  odom_angle_ = AngleMod(odom_angle_ + dtheta);

  true_robot_loc_ += Rotation2Df(true_robot_angle_) * dLoc;
  true_robot_angle_ = AngleMod(true_robot_angle_ + dtheta +
      dist * CONFIG_AngularDriftRate +
      random_.Gaussian(0.0, fabs(dist) * CONFIG_AngularErrorRate));

  truePoseMsg.header.stamp = ros::Time::now();
  truePoseMsg.pose.position.x = true_robot_loc_.x();
  truePoseMsg.pose.position.y = true_robot_loc_.y();
  truePoseMsg.pose.position.z = 0;
  truePoseMsg.pose.orientation.w = cos(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.z = sin(0.5 * true_robot_angle_);
  truePoseMsg.pose.orientation.x = 0;
  truePoseMsg.pose.orientation.y = 0;
  true_pose_publisher_.publish(truePoseMsg);
}

void Simulator::RunIteration() {
  // Simulate time-step.
  Update();

  if (last_publish_time_ < GetMonotonicTime() - 1.0 / CONFIG_PublishRate) {
      //publish odometry and status
    PublishOdometry();
    //publish laser rangefinder messages
    PublishLaser();
    // publish visualization marker messages
    PublishVisualizationMarkers();
    //publish tf
    PublishTransform();
    last_publish_time_ = GetMonotonicTime();
  }

  if (FLAGS_localize) {
    localization_msg_.pose.x = true_robot_loc_.x();
    localization_msg_.pose.y = true_robot_loc_.y();
    localization_msg_.pose.theta = true_robot_angle_;
    localization_msg_.map = map_name_;
    localization_msg_.header.stamp = ros::Time::now();
    localization_publisher_.publish(localization_msg_);
  }
}

void Simulator::Run() {
  // main loop
  const double simulator_fps = CONFIG_PublishRate / CONFIG_SubSampleRate;
  RateLoop rate(simulator_fps);
  while (ros::ok()){
    ros::spinOnce();
    RunIteration();
    rate.Sleep();
  }
}

void Simulator::Step(const amrl_msgs::AckermannCurvatureDriveMsg& cmd,
                     nav_msgs::Odometry* odom_msg,
                     sensor_msgs::LaserScan* scan_msg,
                     amrl_msgs::Localization2DMsg* localization_msg) {
  step_mode_ = true;
  DriveCallback(cmd);
  const int num_iterations = ceil(1.0 / CONFIG_SubSampleRate);
  for (int i = 0; i < num_iterations; ++i) {
    RunIteration();
  }
  *odom_msg = odom_msg_;
  *scan_msg = scan_msg_;
  *localization_msg = localization_msg_;
}

void Simulator::SetStepMode(bool step_mode) {
  step_mode_ = step_mode;
}