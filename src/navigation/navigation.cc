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
\file    navigation.cc
\brief   Implementation for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "shared/math/math_util.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "navigation.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "astar.h"
#include "visualization/visualization.h"

#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "deep_cost_evaluator.h"
#include "deep_cost_map_evaluator.h"
#include "deep_irl_evaluator.h"

using actionlib_msgs::GoalStatus;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Line2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::Pose2Df;
using amrl_msgs::VisualizationMsg;
using geometry_msgs::Twist;
using geometry_msgs::TwistStamped;
using geometry_msgs::PoseStamped;
using navigation::MotionLimits;
using ros_helpers::DrawEigen2DLine;
using sensor_msgs::PointCloud;
using std::atan2;
using std::deque;
using std::max;
using std::min;
using std::swap;
using std::shared_ptr;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace motion_primitives;

// Special test modes.
DEFINE_bool(test_toc, false, "Run 1D time-optimal controller test");
DEFINE_bool(test_obstacle, false, "Run obstacle detection test");
DEFINE_bool(test_avoidance, false, "Run obstacle avoidance test");
DEFINE_bool(test_planner, false, "Run navigation planner test");
DEFINE_bool(test_latency, false, "Run Latency test");
DEFINE_double(test_dist, 0.5, "Test distance");
DEFINE_string(test_log_file, "", "Log test results to file");

DEFINE_double(max_curvature, 2.0, "Maximum curvature of turning");
DEFINE_bool(no_local, false, "can be used to turn off local planner");

// Name of topic to publish twist messages to.
DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "ROS topic to publish twist messages to.");

// Option to disregard joystick safety for running in simulation.
DEFINE_bool(no_joystick, false, "Disregards autonomy enable mode from joystick");

namespace {
ros::Publisher ackermann_drive_pub_;
ros::Publisher twist_drive_pub_;
ros::Publisher viz_pub_;
ros::Publisher status_pub_;
ros::Publisher fp_pcl_pub_;
ros::Publisher path_pub_;
ros::Publisher carrot_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
geometry_msgs::TwistStamped drive_msg_;
PointCloud fp_pcl_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// geometry_msgs::TwistStamped AckermannToTwist(
//     const AckermannCurvatureDriveMsg& msg) {
//   geometry_msgs::TwistStamped  twist_msg;
//   twist_msg.header = msg.header;
//   twist_msg.twist.linear.x = msg.velocity;
//   twist_msg.twist.linear.y = 0;
//   twist_msg.twist.linear.z = 0;
//   twist_msg.twist.angular.x = 0;
//   twist_msg.twist.angular.y = 0;
//   twist_msg.twist.angular.z = msg.velocity * msg.curvature;
//   CHECK(isfinite(twist_msg.twist.angular.z));
//   return twist_msg;
// }

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

nav_msgs::Path CarrotToNavMsgsPath (
  const Vector2f carrot) {
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

struct EightGridVisualizer {
  EightGridVisualizer(bool visualize) :
             kVisualize(visualize) { }

  void DrawEdge(const navigation::EightConnectedDomain::State& s1,
                const navigation::EightConnectedDomain::State& s2) {
    if (!kVisualize) return;
    static const bool kDebug = false;
    if (kDebug) {
      printf("%7.2f,%7.2f -> %7.2f,%7.2f\n",
            s1.x(),
            s1.y(),
            s2.x(),
            s2.y());
    }
    visualization::DrawLine(s1, s2, 0x606060, global_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    if (kDebug) Sleep(0.05);
  }

  const bool kVisualize;
};

struct GraphVisualizer {
  GraphVisualizer(bool visualize) :
             kVisualize(visualize) { }

  void DrawEdge(const navigation::GraphDomain::State& s1,
                const navigation::GraphDomain::State& s2) {
    if (!kVisualize) return;
    static const bool kDebug = false;
    if (kDebug) {
      printf("%7.2f,%7.2f -> %7.2f,%7.2f\n",
            s1.loc.x(),
            s1.loc.y(),
            s2.loc.x(),
            s2.loc.y());
    }
    visualization::DrawLine(s1.loc, s2.loc, 0xC0C0C0, global_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    if (kDebug) Sleep(0.05);
  }

  const bool kVisualize;
};

}  // namespace

namespace navigation {

Navigation::Navigation() :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    odom_initialized_(false),
    loc_initialized_(false),
    t_point_cloud_(0),
    t_odometry_(0),
    enabled_(false),
    initialized_(false),
    sampler_(nullptr),
    evaluator_(nullptr) {
  sampler_ = std::unique_ptr<PathRolloutSamplerBase>(new AckermannSampler());
}

void Navigation::Initialize(const NavigationParameters& params,
                            const string& map_file,
                            ros::NodeHandle* n) {
  ackermann_drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  twist_drive_pub_ = n->advertise<geometry_msgs::Twist>(
      FLAGS_twist_drive_topic, 1);
  status_pub_ = n->advertise<GoalStatus>(
      "navigation_goal_status", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  fp_pcl_pub_ = n->advertise<PointCloud>("forward_predicted_pcl", 1);
  path_pub_ = n->advertise<nav_msgs::Path>(
      "trajectory", 1, true);
  carrot_pub_ = n->advertise<nav_msgs::Path>("carrot",1,true);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  // Initialize status message
  status_msg_.status = 3;
  status_msg_.text = "Navigation Status";
  InitRosHeader("base_link", &drive_msg_.header);
  InitRosHeader("base_link", &fp_pcl_msg_.header);
  params_ = params;
  planning_domain_ = GraphDomain(map_file, &params_);
  initialized_ = true;
  sampler_->SetNavParams(params);
  
  PathEvaluatorBase* evaluator;
  if (params_.evaluator_type == "cost") {
    auto deep_evaluator = new DeepCostEvaluator(params.K, params.D, params.H, params.use_kinect, params.blur);
    deep_evaluator->LoadModel(params.model_path);
    evaluator = (PathEvaluatorBase*) deep_evaluator;
  } else if (params_.evaluator_type == "cost_map") {
    auto cost_map_evaluator = new DeepCostMapEvaluator(params.K, params.D, params.H, params.use_kinect, params.blur);
    cost_map_evaluator->LoadModel(params.model_path);
    evaluator = (PathEvaluatorBase*) cost_map_evaluator;
  } else if (params_.evaluator_type == "irl") {
    auto deep_evaluator = new DeepIRLEvaluator(params.K, params.D, params.H, params.use_kinect, params.blur);
    deep_evaluator->LoadModels(params.embedding_model_path, params.model_path);
    evaluator = (PathEvaluatorBase*) deep_evaluator;
  } else {
    printf("Uknown evaluator type %s\n", params.evaluator_type.c_str());
    exit(1);
  }
  evaluator_ = std::unique_ptr<PathEvaluatorBase>(evaluator);
}

bool Navigation::Enabled() const {
  return enabled_;
}

void Navigation::Enable(bool enable) {
  enabled_ = enable;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = false;
  plan_path_.clear();
}

void Navigation::ConvertPathToNavMsgsPath() {
  if(plan_path_.size()>0){
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
    for (size_t i = 0; i < plan_path_.size(); i++) {
      geometry_msgs::PoseStamped pose_plan;
      pose_plan.pose.position.x = plan_path_[i].loc.x();
      pose_plan.pose.position.y = plan_path_[i].loc.y();

      pose_plan.pose.orientation.x = 0;
      pose_plan.pose.orientation.y = 0;
      pose_plan.pose.orientation.z = 0;
      pose_plan.pose.orientation.w = 1;

      pose_plan.header.stamp = ros::Time::now();
      pose_plan.header.frame_id = "map";
      path.poses.push_back(pose_plan);

      path_pub_.publish(path);
    }
  }
}

void Navigation::UpdateMap(const string& map_path) {
  planning_domain_.Load(map_path);
  plan_path_.clear();
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  robot_loc_ = loc;
  robot_angle_ = angle;
  loc_initialized_ = true;
}

void Navigation::SendCommand(Eigen::Vector2f vel, float ang_vel) {
  drive_msg_.header.stamp = ros::Time::now();
  if (!FLAGS_no_joystick && !enabled_) {
    vel.setZero();
    ang_vel = 0;
  }
  
  drive_msg_.twist.angular.x = 0;
  drive_msg_.twist.angular.y = 0;
  drive_msg_.twist.angular.z = ang_vel;
  drive_msg_.twist.linear.x = vel.x();
  drive_msg_.twist.linear.y = vel.y();
  drive_msg_.twist.linear.z = 0;
  // printf("SendCommand: %f %f %f\n", drive_msg_.twist.linear.x, drive_msg_.twist.linear.y, drive_msg_.twist.angular.z);

  twist_drive_pub_.publish(drive_msg_.twist);

  auto ackermann_msg = TwistToAckermann(drive_msg_);
  ackermann_drive_pub_.publish(ackermann_msg);
  // This command is going to take effect system latency period after. Hence
  // modify the timestamp to reflect the time when it will take effect.
  auto cmd_copy = drive_msg_;
  cmd_copy.header.stamp += ros::Duration(params_.system_latency);
  command_history_.push_back(cmd_copy);
  if (false) {
    printf("Push %7.3f %7.3f %7.3f\n",
          command_history_.back().twist.linear.x,
          command_history_.back().twist.linear.y,
          command_history_.back().twist.angular.z);
  }
}

void Navigation::PruneLatencyQueue() {
  if (command_history_.empty()) return;
  const double update_time = min(t_point_cloud_, t_odometry_);
  static const bool kDebug = false;
  for (size_t i = 0; i < command_history_.size(); ++i) {
    const double t_cmd = command_history_[i].header.stamp.toSec();
    if (kDebug) {
      printf("Command %d %f\n", int(i), t_cmd - update_time);
    }
    if (t_cmd < update_time - params_.dt) {
      if (kDebug) {
        printf("Erase %d %f %f\n",
            int(i),
            t_cmd - update_time,
            command_history_[i].twist.linear.x);
      }
      command_history_.erase(command_history_.begin() + i);
      --i;
    }
  }
}

void Navigation::UpdateOdometry(const nav_msgs::Odometry& msg) {
  latest_odom_msg_ = msg;
  t_odometry_ = msg.header.stamp.toSec();
  PruneLatencyQueue();
  if (!odom_initialized_) {
    starting_loc_ = Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y);
    odom_initialized_ = true;
  }
}

void PublishForwardPredictedPCL(const vector<Vector2f>& pcl) {
  fp_pcl_msg_.points.resize(pcl.size());
  for (size_t i = 0; i < pcl.size(); ++i) {
    fp_pcl_msg_.points[i].x = pcl[i].x();
    fp_pcl_msg_.points[i].y = pcl[i].y();
    fp_pcl_msg_.points[i].z = 0.324;
  }
  fp_pcl_msg_.header.stamp = ros::Time::now();
  fp_pcl_pub_.publish(fp_pcl_msg_);
}

void Navigation::ForwardPredict(double t) {
  if (command_history_.empty()) {
    robot_vel_ = Vector2f(0, 0);
    robot_omega_ = 0;
  } else {
    const Twist latest_twist = command_history_.back().twist;
    robot_vel_ = Vector2f(latest_twist.linear.x, latest_twist.linear.y);
    robot_omega_ = latest_twist.angular.z;
  }
  if (false) {
    for (size_t i = 0; i < command_history_.size(); ++i) {
      const auto& c = command_history_[i];
      printf("%d %f %f\n", int(i), t - c.header.stamp.toSec(), c.twist.linear.x);
    }
    printf("Predict: %f %f\n", t - t_odometry_, t - t_point_cloud_);
  }
  const geometry_msgs::Pose odom_pose = latest_odom_msg_.pose.pose;
  odom_loc_ = Vector2f(odom_pose.position.x, odom_pose.position.y);
  odom_angle_ = 2.0f * atan2f(odom_pose.orientation.z, odom_pose.orientation.w);
  using Eigen::Affine2f;
  using Eigen::Rotation2Df;
  using Eigen::Translation2f;
  Affine2f lidar_tf = Affine2f::Identity();
  for (const TwistStamped& c : command_history_) {
    const double cmd_time = c.header.stamp.toSec();
    if (cmd_time > t) continue;
    if (cmd_time >= t_odometry_ - params_.dt) {
      const float dt = (t_odometry_ > cmd_time) ?
          min<double>(t_odometry_ - cmd_time, params_.dt) :
          min<double>(t - cmd_time, params_.dt);
      odom_loc_ += dt * (Rotation2Df(odom_angle_) * Vector2f(
          c.twist.linear.x, c.twist.linear.y));
      odom_angle_ = AngleMod(odom_angle_ + dt * c.twist.angular.z);
    }
    if (t_point_cloud_ >= cmd_time  - params_.dt) {
      const float dt = (t_point_cloud_ > cmd_time) ?
          min<double>(t_point_cloud_ - cmd_time, params_.dt) :
          min<double>(t - cmd_time, params_.dt);
      lidar_tf =
          Translation2f(-dt * Vector2f(c.twist.linear.x, c.twist.linear.y)) *
          Rotation2Df(-c.twist.angular.z * dt) *
          lidar_tf;
    }
  }
  fp_point_cloud_.resize(point_cloud_.size());
  for (size_t i = 0; i < point_cloud_.size(); ++i) {
    fp_point_cloud_[i] = lidar_tf * point_cloud_[i];
  }
  PublishForwardPredictedPCL(fp_point_cloud_);
}

void Navigation::TrapezoidTest() {
  if (!odom_initialized_) return;
  const float x = (odom_loc_ - starting_loc_).norm();
  const float speed = robot_vel_.norm();
  const float velocity_cmd = Run1DTimeOptimalControl(
      params_.linear_limits,
      x,
      speed,
      FLAGS_test_dist,
      0, 
      params_.dt);
  SendCommand(Vector2f(velocity_cmd, 0), 0);
}

void Navigation::ObstacleTest() {
  const float speed = robot_vel_.norm();
  float free_path_length = 30.0;
  float clearance = 10;
  GetStraightFreePathLength(&free_path_length, &clearance);
  const float dist_left =
      max<float>(0.0f, free_path_length - params_.obstacle_margin);
  printf("%f\n", free_path_length);
  const float velocity_cmd = Run1DTimeOptimalControl(
      params_.linear_limits,
      0,
      speed,
      dist_left,
      0, 
      params_.dt);
  SendCommand(Vector2f(velocity_cmd, 0), 0);
}

Vector2f GetClosestApproach(const PathOption& o, const Vector2f& target) {
  if (fabs(o.curvature) < kEpsilon) {
    // Straight line path
    if (target.x() > o.free_path_length) {
      return Vector2f(o.free_path_length, 0);
    } else if (target.x() < 0.0) {
      return Vector2f(0, 0);
    } else {
      return Vector2f(target.x(), 0);
    }
  }
  const float end_angle = fabs(o.curvature * o.free_path_length);
  const float turn_radius = 1.0f / o.curvature;
  const Vector2f turn_center(0, turn_radius);
  const Vector2f target_radial = target - turn_center;

  const Vector2f start(0, 0);
  const Vector2f middle_radial = fabs(turn_radius) * target_radial.normalized();
  const Vector2f middle = turn_center + middle_radial;
  const float middle_angle =
      atan2(fabs(middle_radial.x()), fabs(middle_radial.y()));

  const Vector2f end(fabs(turn_radius) * sin(end_angle),
                     turn_radius * (1.0f - cos(end_angle)));

  Vector2f closest_point = start;
  if (middle_angle < end_angle &&
      (closest_point - target).squaredNorm() >
      (middle - target).squaredNorm()) {
    closest_point = middle;
  }
  if ((closest_point - target).squaredNorm() >
      (end - target).squaredNorm()) {
    closest_point = end;
  }
  return closest_point;
}

float GetClosestDistance(const PathOption& o, const Vector2f& target) {
  const Vector2f closest_point = GetClosestApproach(o, target);
  return (target - closest_point).norm();
}

void Navigation::ObstAvTest() {
  const Vector2f kTarget(4, 0);
  local_target_ = kTarget;
  RunObstacleAvoidance();
}

void Navigation::ObserveImage(cv::Mat image, double time) {
  latest_image_ = image;
  t_image_ = time;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
  t_point_cloud_ = time;
  PruneLatencyQueue();
}

void Navigation::Plan(Eigen::Vector2f goal_loc) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  static const bool kVisualize = true;
  typedef navigation::GraphDomain Domain;
  planning_domain_.ResetDynamicStates();
  const uint64_t start_id = planning_domain_.AddDynamicState(robot_loc_);
  const uint64_t goal_id = planning_domain_.AddDynamicState(goal_loc);
  Domain::State start = planning_domain_.states[start_id];
  Domain::State goal = planning_domain_.states[goal_id];
  visualization::ClearVisualizationMsg(global_viz_msg_);
  GraphVisualizer graph_viz(kVisualize);
  visualization::DrawCross(goal.loc, 0.2, 0xFF0000, global_viz_msg_);
  visualization::DrawCross(start.loc, 0.2, 0x8F, global_viz_msg_);
  const bool found_path =
      AStar(start, goal, planning_domain_, &graph_viz, &plan_path_);
  if (found_path) {
    const uint32_t path_color = 0x10A000;
    CHECK(plan_path_.size() > 0);
    Vector2f s1 = plan_path_[0].loc;
    for (size_t i = 1; i < plan_path_.size(); ++i) {
      Vector2f s2 = plan_path_[i].loc;
      visualization::DrawLine(s1, s2, path_color, global_viz_msg_);
      s1 = s2;
    }
    viz_pub_.publish(global_viz_msg_);
    ConvertPathToNavMsgsPath();
  } else {
    printf("No path found!\n");
  }
}

void Navigation::PlannerTest() {
  if (!loc_initialized_) return;
  Plan(nav_goal_loc_);
}

DEFINE_double(max_plan_deviation, 0.5,
   "Maximum premissible deviation from the plan");

bool Navigation::PlanStillValid() {
  if (plan_path_.size() < 2) return false;
  for (size_t i = 0; i + 1 < plan_path_.size(); ++i) {
    const float dist_from_segment =
        geometry::DistanceFromLineSegment(robot_loc_,
                                          plan_path_[i].loc,
                                          plan_path_[i + 1].loc);
    if (dist_from_segment < FLAGS_max_plan_deviation) {
      return true;
    }
  }
  return false;
}

Vector2f Navigation::GetCarrot() {
  const float kSqCarrotDist = Sq(params_.carrot_dist);
  CHECK_GE(plan_path_.size(), 2u);

  if ((plan_path_[0].loc - robot_loc_).squaredNorm() < kSqCarrotDist) {
    // Goal is within the carrot dist.
    return plan_path_[0].loc;
  }

  // Find closest line segment in plan to current location
  float closest_dist = FLT_MAX;
  int i0 = 0, i1 = 1;
  for (size_t i = 0; i + 1 < plan_path_.size(); ++i) {
    const Vector2f v0 = plan_path_[i].loc;
    const Vector2f v1 = plan_path_[i + 1].loc;
    const float dist_to_segment = geometry::DistanceFromLineSegment(
        robot_loc_, v0, v1);
    if (dist_to_segment < closest_dist) {
      closest_dist = dist_to_segment;
      i0 = i;
      i1 = i + 1;
    }
  }
  // printf("closest: %d %d %f\n", i0, i1, closest_dist);

  if (closest_dist > kSqCarrotDist) {
    // Closest edge on the plan is farther than carrot dist to the robot.
    // The carrot will be the projection of the robot loc on to the edge.
    const Vector2f v0 = plan_path_[i0].loc;
    const Vector2f v1 = plan_path_[i1].loc;
    Vector2f carrot;
    float sqdist;
    geometry::ProjectPointOntoLineSegment(robot_loc_, v0, v1, &carrot, &sqdist);
    return carrot;
  }

  // Iterate from current line segment to goal until the segment intersects
  // the circle centered at the robot, of radius kCarrotDist.
  // The goal is not within carrot dist of the robot, and the robot is within
  // carrot dist of some line segment. Hence, there must exist at least one
  // vertex along the plan towards the goal that is out of the carrot dist.
  for (int i = i1; i - 1 >= 0; --i) {
    i0 = i;
    // const Vector2f v0 = plan_path_[i].loc;
    const Vector2f v1 = plan_path_[i - 1].loc;
    if ((v1 - robot_loc_).squaredNorm() > kSqCarrotDist) {
      break;
    }
  }
  i1 = i0 -1;
  // printf("i0:%d i1:%d\n", i0, i1);
  const Vector2f v0 = plan_path_[i0].loc;
  const Vector2f v1 = plan_path_[i1].loc;
  Vector2f r0, r1;
  #define V2COMP(v) v.x(), v.y()
  // printf("%f,%f %f,%f %f,%f %f\n",
  //     V2COMP(robot_loc_), V2COMP(v0), V2COMP(v1), (v0 - v1).norm());
  const int num_intersections = geometry::CircleLineIntersection<float>(
      robot_loc_, params_.carrot_dist, v0, v1, &r0, &r1);
  CHECK_GT(num_intersections, 0);
  if (num_intersections == 1) return r0;
  if ((r0 - v1).squaredNorm() < (r1 - v1).squaredNorm()) {
    return r0;
  } else {
    return r1;
  }
}

void Navigation::GetStraightFreePathLength(float* free_path_length,
                                           float* clearance) {
  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * params_.robot_length - params_.base_link_offset + params_.obstacle_margin;
  // The robot's half-width.
  const float w = 0.5 * params_.robot_width + params_.obstacle_margin;
  for (const Vector2f& p : fp_point_cloud_) {
    if (fabs(p.y()) > w || p.x() < 0.0f) continue;
    *free_path_length = min(*free_path_length, p.x() - l);
  }
  *clearance = params_.max_clearance;
  for (const Vector2f& p : point_cloud_) {
    if (p.x() - l > *free_path_length || p.x() < 0.0) continue;
    *clearance = min<float>(*clearance, fabs(fabs(p.y() - w)));
  }
  *clearance = max(0.0f, *clearance);
  *free_path_length = max(0.0f, *free_path_length);
}

Vector2f GetFinalPoint(const PathOption& o) {
  if (fabs(o.curvature) < 0.01) {
    return Vector2f(o.free_path_length, 0);
  } else {
    const float r = 1.0f / o.curvature;
    const float a = o.free_path_length / fabs(r);
    return Vector2f(fabs(r) * sin(a), r * (1.0 - cos(a)));
  }
}

void Navigation::DrawRobot() {
  {
    // How much the robot's body extends behind of its base link frame.
    const float l1 = -0.5 * params_.robot_length - params_.base_link_offset - params_.obstacle_margin;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * params_.robot_length - params_.base_link_offset + params_.obstacle_margin;
    const float w = 0.5 * params_.robot_width + params_.obstacle_margin;
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
    const float l1 = -0.5 * params_.robot_length - params_.base_link_offset;
    // How much the robot's body extends in front of its base link frame.
    const float l2 = 0.5 * params_.robot_length - params_.base_link_offset;
    const float w = 0.5 * params_.robot_width;
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

DEFINE_double(tx, 0.4, "Test obstacle point - X");
DEFINE_double(ty, -0.38, "Test obstacle point - Y");

void Navigation::RunObstacleAvoidance() {
  const bool debug = false;
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // static const bool kDebug = false;

  if (false) {
    fp_point_cloud_ = {
      Vector2f(FLAGS_tx, FLAGS_ty)
    };
    for (const Vector2f& v : fp_point_cloud_) {
      visualization::DrawCross(v, 0.05, 0xFF0000, local_viz_msg_);
    }
  }

  sampler_->Update(
      robot_vel_, robot_omega_, local_target_, fp_point_cloud_, latest_image_);
  evaluator_->Update(
      robot_vel_, robot_omega_, local_target_, fp_point_cloud_, latest_image_);
  auto paths = sampler_->GetSamples(params_.num_options);
  if (debug) {
    printf("%lu options\n", paths.size());
    int i = 0;
    for (auto p : paths) {
      ConstantCurvatureArc arc = 
          *reinterpret_cast<ConstantCurvatureArc*>(p.get());
      printf("%3d: %7.5f %7.3f %7.3f\n",
          i++, arc.curvature, arc.length, arc.curvature);
      visualization::DrawCross(arc.obstruction, 0.1, 0xFF0000, local_viz_msg_);
    }
  }
  if (paths.size() == 0) {
    // No options, just stop.
    if (debug) printf("No paths found\n");
    Halt();
    return;
  }
  auto best_path = evaluator_->FindBest(paths);
  if (best_path == nullptr) {
    if (debug) printf("No best path found\n");
    // No valid path found!
    Halt();
    return;
  }
  for (shared_ptr<PathRolloutBase>& o : paths) {
    ConstantCurvatureArc arc = 
        *reinterpret_cast<ConstantCurvatureArc*>(o.get());
    visualization::DrawPathOption(arc.curvature,
                                  arc.length,
                                  arc.clearance,
                                  0x0000FF,
                                  local_viz_msg_);
  }
  ConstantCurvatureArc arc = 
        *reinterpret_cast<ConstantCurvatureArc*>(best_path.get());
  visualization::DrawPathOption(arc.curvature,
                                arc.length,
                                arc.clearance,
                                0xFF0000,
                                local_viz_msg_);
  if (debug) printf("Best Path Chosen %f\n", arc.curvature);

  const Vector2f closest_point = best_path->EndPoint().translation;
  visualization::DrawCross(closest_point, 0.05, 0x00A000, local_viz_msg_);
  float ang_vel_cmd = 0;
  Vector2f vel_cmd(0, 0);

  float max_map_speed = params_.linear_limits.max_speed;
  planning_domain_.GetClearanceAndSpeedFromLoc(
      robot_loc_, nullptr, &max_map_speed);
  auto linear_limits = params_.linear_limits;
  linear_limits.max_speed = min(max_map_speed, linear_limits.max_speed);
  best_path->GetControls(linear_limits, params_.angular_limits, params_.dt, robot_vel_, robot_omega_, vel_cmd, ang_vel_cmd);
  if (debug) printf("cmd: %7.3f %7.3f %7.3f\n", vel_cmd.x(), vel_cmd.y(), ang_vel_cmd);
  // const float dist_left =
  //     max<float>(0.0, best_option.free_path_length - params_.obstacle_margin);

  // const float speed = robot_vel_.norm();
  // float max_speed = min<float>(params_.linear_limits.speed,
  //     sqrt(2.0f * params_.linear_limits.accel * best_option.clearance));
  // max_speed = min(max_map_speed, max_speed);
  // velocity_cmd = Run1DTOC(0, dist_left, speed, max_speed, params_.linear_limits.accel, params_.linear_limits.decel, params_.dt);
  SendCommand(vel_cmd, ang_vel_cmd);
}

void Navigation::Halt() {
  const float kEpsSpeed = 0.01;
  const float velocity = robot_vel_.x();
  float velocity_cmd = 0;
  if (fabs(velocity) > kEpsSpeed) {
    const float dv = params_.linear_limits.max_deceleration * params_.dt;
    if (velocity < -dv) {
      velocity_cmd = velocity + dv;
    } else if (velocity > dv) {
      velocity_cmd = velocity - dv;
    } else {
      velocity_cmd = 0;
    }
  }
  // TODO: Motion profiling for omega.
  // printf("%8.3f %8.3f\n", velocity, velocity_cmd);
  // printf("cmd: %7.3f %7.3f %7.3f\n", velocity_cmd, 0.0, 0.0);
  SendCommand(Vector2f(velocity_cmd, 0), 0);
}

void Navigation::TurnInPlace() {
  const float kMaxLinearSpeed = 0.1;
  const float velocity = robot_vel_.x();
  float angular_cmd = 0;
  if (fabs(velocity) > kMaxLinearSpeed) {
    Halt();
    return;
  }
  const float goal_theta = atan2(local_target_.y(), local_target_.x());
  const float dv = params_.dt * params_.angular_limits.max_acceleration;
  if (robot_omega_ * goal_theta < 0.0f) {
    // Turning the wrong way!
    if (fabs(robot_omega_) < dv) {
      angular_cmd = 0;
    } else {
      angular_cmd = robot_omega_ - Sign(robot_omega_) * dv;
    }
  } else {
    // printf("Running TOC\n");
    const float s = Sign(goal_theta);
    angular_cmd = Run1DTimeOptimalControl(
        params_.linear_limits,
        0,
        robot_omega_,
        s * goal_theta,
        0, 
        params_.dt);
  }
  // TODO: Motion profiling for omega.
  // printf("TurnInPlace: %8.3f %8.3f %8.3f\n",
  //        RadToDeg(goal_theta),
  //        RadToDeg(robot_omega_),
  //        RadToDeg(angular_cmd));
  SendCommand(Vector2f(0, 0), angular_cmd);
}

void Navigation::LatencyTest() {
  static FILE* fid = nullptr;
  if (!FLAGS_test_log_file.empty() && fid == nullptr) {
    fid = fopen(FLAGS_test_log_file.c_str(), "w");
  }
  const float kMaxSpeed = 0.75;
  const float kFrequency = 0.4;

  static double t_start_ = GetMonotonicTime();
  const double t = GetMonotonicTime() - t_start_;
  float v_current = robot_vel_.x();
  float v_cmd = kMaxSpeed *
      sin(2.0 * M_PI * kFrequency * t);
  SendCommand(Vector2f(v_cmd, 0), 0);
  printf("t:%f current:%f cmd:%f\n", t, v_current, v_cmd);
  if (fid != nullptr) {
    fprintf(fid, "%f %f %f\n", t, v_current, v_cmd);
  }
}

void Navigation::Abort() {
  if (!nav_complete_) {
    printf("Abort!\n");
  }
  nav_complete_ = true;
}

void Navigation::Run() {
  const bool kDebug = false;
  if (!initialized_) {
    if (kDebug) printf("Not initialized\n");
    return;
  }
  visualization::ClearVisualizationMsg(local_viz_msg_);
  DrawRobot();
  if (!odom_initialized_) {
    if (kDebug) printf("Odometry not initialized\n");
    return;
  }
  ForwardPredict(ros::Time::now().toSec() + params_.system_latency);
  if (FLAGS_test_toc) {
    TrapezoidTest();
    return;
  } else if (FLAGS_test_obstacle) {
    ObstacleTest();
    return;
  } else if (FLAGS_test_avoidance) {
    ObstAvTest();
    viz_pub_.publish(local_viz_msg_);
    return;
  } else if (FLAGS_test_planner) {
    PlannerTest();
    return;
  } else if (FLAGS_test_latency) {
    LatencyTest();
    return;
  }

  // Publish Navigation Status
  if (nav_complete_) {
    if (kDebug) printf("Nav complete\n");
    Halt();
    status_msg_.status = 3;
    status_pub_.publish(status_msg_);
    viz_pub_.publish(local_viz_msg_);
    return;
  }
  status_msg_.status = 1;
  status_pub_.publish(status_msg_);
  if (!PlanStillValid()) {
    if (kDebug) printf("Replanning\n");
    Plan(nav_goal_loc_);
  }
  // Get Carrot.
  const Vector2f carrot = GetCarrot();
  carrot_pub_.publish(CarrotToNavMsgsPath(carrot));
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(carrot, 0.2, 0x10E000, msg_copy);
  visualization::DrawArc(
      Vector2f(0, 0), params_.carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);
  viz_pub_.publish(msg_copy);
  // Check if complete.
  nav_complete_ = 
      (robot_loc_ - carrot).squaredNorm() < Sq(params_.target_dist_tolerance) &&
      (robot_vel_).squaredNorm() < Sq(params_.target_dist_tolerance);
  // Run local planner.
  if (nav_complete_) {
    if (kDebug) printf("Now complete\n");
    Halt();
  } else {
    // TODO check if the robot needs to turn around.
    local_target_ = Rotation2Df(-robot_angle_) * (carrot - robot_loc_);
    static const float kLocalFOV = DegToRad(360.0);
    const float theta = atan2(local_target_.y(), local_target_.x());
    if (local_target_.squaredNorm() > Sq(params_.carrot_dist)) {
      local_target_ = params_.carrot_dist * local_target_.normalized();
    }
    visualization::DrawCross(local_target_, 0.2, 0xFF0080, local_viz_msg_);
    if (!FLAGS_no_local) {
      // printf("Local target: %8.3f, %8.3f (%6.1f\u00b0)\n",
      //     local_target_.x(), local_target_.y(), RadToDeg(theta));
      if (fabs(theta) > kLocalFOV) {
        if (kDebug) printf("TurnInPlace\n");
        TurnInPlace();
      } else {
        if (kDebug) printf("ObstAv\n");
        RunObstacleAvoidance();
      }
    }
  }
  viz_pub_.publish(local_viz_msg_);
}

}  // namespace navigation
