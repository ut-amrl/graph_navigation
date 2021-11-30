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
#include "graph_navigation/MDPSolver.h"
#include "graph_navigation/TurnInPlace.h"
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
#include "graph_navigation/IntrospectivePerceptionInfoArray.h"
#include "nlohmann/json.hpp"


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
using graph_navigation::MDPSolver;
using geometry_msgs::Twist;
using geometry_msgs::TwistStamped;
using geometry_msgs::PoseStamped;
using ros_helpers::DrawEigen2DLine;
using sensor_msgs::PointCloud;
using std::atan2;
using std::deque;
using std::max;
using std::min;
using std::swap;
using std::string;
using std::vector;
using json = nlohmann::json;

using namespace math_util;
using namespace ros_helpers;
using namespace introspection;

// Special test modes.
DEFINE_bool(test_toc, false, "Run 1D time-optimal controller test");
DEFINE_bool(test_obstacle, false, "Run obstacle detection test");
DEFINE_bool(test_avoidance, false, "Run obstacle avoidance test");
DEFINE_bool(test_planner, false, "Run navigation planner test");
DEFINE_bool(test_latency, false, "Run Latency test");
DEFINE_bool(can_turn_in_place, true, "Enables in place turning behavior.");
DEFINE_double(test_dist, 0.5, "Test distance");
DEFINE_string(test_log_file, "", "Log test results to file");

DEFINE_double(max_curvature, 2.0, "Maximum curvature of turning");
DEFINE_double(navigation_tolerance, 0.5, 
              "Tolerance for distance to goal in meters.");

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
ros::Publisher introspective_perception_pub_;
ros::ServiceClient mdp_client_;
ros::ServiceClient turn_in_place_client_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
PointCloud fp_pcl_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

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
    initialized_(false) { }

void Navigation::Initialize(const NavigationParameters& params,
                            const string& map_file,
                            const string& failure_logs_path,
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
  introspective_perception_pub_ =
      n->advertise<graph_navigation::IntrospectivePerceptionInfoArray>(
          "/introspective_perception/info_array", 1);
  mdp_client_ = n->serviceClient<MDPSolver>("mdp_solver");
  turn_in_place_client_ = n->serviceClient<graph_navigation::TurnInPlace>(
      "/airsim_node/turn_in_place");
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  // Initialize status message
  status_msg_.status = status_msg_.SUCCEEDED;
  status_msg_.text = "Navigation Status";
  InitRosHeader("base_link", &drive_msg_.header);
  InitRosHeader("base_link", &fp_pcl_msg_.header);
  params_ = params;
  planning_domain_ = GraphDomain(map_file, &params_);

  if (params_.competence_aware && params_.load_failure_logs) {
    LoadFailureInstancesFromFile(failure_logs_path);
  }
  initialized_ = true;
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
  nav_aborted_ = false;
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

void Navigation::UpdateMap(const string& map_dir, const string& map_name) {
  planning_domain_.Load(GetMapPath(map_dir, map_name));
  if (params_.competence_aware && params_.load_failure_logs) {
    LoadFailureInstancesFromFile(GetFailureLogsPath(map_dir, map_name));
  }
  plan_path_.clear();
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  robot_loc_ = loc;
  robot_angle_ = angle;
  loc_initialized_ = true;
}

void Navigation::SendCommand(float vel, float curvature) {
  drive_msg_.header.stamp = ros::Time::now();
  if (!FLAGS_no_joystick && !enabled_) {
    vel = 0;
    curvature = 0;
  }
  drive_msg_.curvature = curvature;
  drive_msg_.velocity = vel;
  ackermann_drive_pub_.publish(drive_msg_);
  auto twist = AckermannToTwist(drive_msg_);
  twist_drive_pub_.publish(twist.twist);
  // This command is going to take effect system latency period after. Hence
  // modify the timestamp to reflect the time when it will take effect.
  twist.header.stamp += ros::Duration(params_.system_latency);
  command_history_.push_back(twist);
  if (false) {
    printf("Push %f %f\n",
          twist.twist.linear.x,
          command_history_.back().twist.linear.x);
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

float Navigation::Run1DTOC(float x_now,
                           float x_target,
                           float v_now,
                           float max_speed,
                           float a_max,
                           float d_max,
                           float dt) const {
  static FILE* fid = nullptr;
  // const bool test_mode = false;
  const bool test_mode =
      FLAGS_test_toc || FLAGS_test_obstacle || FLAGS_test_avoidance;
  if (test_mode && !FLAGS_test_log_file.empty() && fid == nullptr) {
    fid = fopen(FLAGS_test_log_file.c_str(), "w");
  }
  const float dist_left = x_target - x_now;
  float velocity_cmd = 0;
  const float speed = fabs(v_now);
  const float dv_a = dt * a_max;
  const float dv_d = dt * d_max;
  float accel_stopping_dist =
      (speed + 0.5 * dv_a) * dt +
      Sq(speed + dv_a) / (2.0 * d_max);
  float cruise_stopping_dist =
      speed * dt +
      Sq(speed) / (2.0 * d_max);
  char phase = '?';
  if (dist_left >  0) {
    if (speed > max_speed) {
      // Over max speed, slow down.
      phase = 'O';
      velocity_cmd = max<float>(0.0f, speed - dv_a);
    } else if (speed < max_speed && accel_stopping_dist < dist_left) {
      // Acceleration possible.
      phase = 'A';
      velocity_cmd = min<float>(max_speed, speed + dv_a);
    } else if (cruise_stopping_dist < dist_left) {
      // Must maintain speed, cruise phase.
      phase = 'C';
      velocity_cmd = speed;
    } else {
      // Must decelerate.
      phase = 'D';
      velocity_cmd = max<float>(0, speed - dv_d);
    }
  } else if (speed > 0.0f) {
    phase = 'X';
    velocity_cmd = max<float>(0, speed - dv_d);
  }
  if (test_mode) {
    printf("%c x:%f dist_left:%f a_dist:%f c_dist:%f v:%f cmd:%f\n",
           phase,
           x_now,
           dist_left,
           accel_stopping_dist,
           cruise_stopping_dist,
           v_now,
           velocity_cmd);
    if (fid != nullptr) {
      fprintf(fid, "%f %f %f %f %f %f\n",
              x_now,
              dist_left,
              accel_stopping_dist,
              cruise_stopping_dist,
              v_now,
              velocity_cmd);
      fflush(fid);
    }
  }
  return velocity_cmd;
}

void Navigation::TrapezoidTest() {
  if (!odom_initialized_) return;
  const float x = (odom_loc_ - starting_loc_).norm();
  const float speed = robot_vel_.norm();
  const float velocity_cmd = Run1DTOC(x, FLAGS_test_dist, speed,
    params_.linear_limits.speed, params_.linear_limits.accel, params_.linear_limits.decel, params_.dt);
  SendCommand(velocity_cmd, 0);
}

void Navigation::ObstacleTest() {
  const float speed = robot_vel_.norm();
  float free_path_length = 30.0;
  float clearance = 10;
  GetStraightFreePathLength(&free_path_length, &clearance);
  const float dist_left =
      max<float>(0.0f, free_path_length - params_.obstacle_margin);
  printf("%f\n", free_path_length);
  const float velocity_cmd = Run1DTOC(0, dist_left, speed, params_.linear_limits.speed, params_.linear_limits.accel, params_.linear_limits.decel, params_.dt);
  SendCommand(velocity_cmd, 0);
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

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
  t_point_cloud_ = time;
  PruneLatencyQueue();
}

void Navigation::Plan() {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  static const bool kVisualize = true;
  // When kEnableReducedGraph is enabled and the start/goal node location is 
  // close to a static node, connect it directly to the closest static node 
  // instead of projecting it to the closest edge. 
  const bool kEnableReducedGraph = true; 
  typedef navigation::GraphDomain Domain;
  planning_domain_.ResetDynamicStates();
  const uint64_t start_id =
      planning_domain_.AddDynamicState(robot_loc_, kEnableReducedGraph);
  const uint64_t goal_id =
      planning_domain_.AddDynamicState(nav_goal_loc_, kEnableReducedGraph);
  Domain::State start = planning_domain_.states[start_id];
  Domain::State goal = planning_domain_.states[goal_id];
  visualization::ClearVisualizationMsg(global_viz_msg_);
  GraphVisualizer graph_viz(kVisualize);
  visualization::DrawCross(goal.loc, 0.2, 0xFF0000, global_viz_msg_);
  visualization::DrawCross(start.loc, 0.2, 0x8F, global_viz_msg_);
  bool found_path = false;
  if (!params_.competence_aware) {
    found_path = AStar(start, goal, planning_domain_, &graph_viz, &plan_path_);
  } else {
    std::vector<uint64_t> start_goal_ids{start_id, goal_id};
    if (!params_.only_collect_traversal_stats) {
      if (params_.frequentist_mode || params_.bayes_mode) {
        // In frequentist mode, the probability of navigation failure for
        // each edge is stored in the graph structure hence will be published
        // to be received by the MDP solver
        PublishAllEdgesFailureInfo(params_.frequentist_mode, start_goal_ids);
      } else {
        // If not in frequentist mode, only the probability of navigation
        // failure for dynamic edges will be estimated and published since the
        // corresponding values for the static edges are already tracked by
        // the MDP solver.
        PublishDynamicEdgesFailureInfo(start_goal_ids);
      }
    }

    found_path = QueryMDPSolver(start_id, goal_id, &graph_viz, &plan_path_);
  }
      
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
  Plan();
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

  bool enforce_waypoint_dist_thresh = params_.waypoint_distance_thresh > 0;
  float sq_waypoint_dist_thresh = Sq(params_.waypoint_distance_thresh);

  if (((plan_path_[0].loc - robot_loc_).squaredNorm() < kSqCarrotDist)
      && !enforce_waypoint_dist_thresh) {
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
  UpdatePlanProgress(i1);
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

    // Return the next waypoint as the carrot if it is closer than the carrot
    // distance but farther than the waypoint_dist_thresh
    if (enforce_waypoint_dist_thresh &&
        ((v1 - robot_loc_).squaredNorm() < kSqCarrotDist) &&
        ((v1 - robot_loc_).squaredNorm() > sq_waypoint_dist_thresh)) {
      return v1;
    }

    if ((v1 - robot_loc_).squaredNorm() > kSqCarrotDist) {
      break;
    }
  }
  i1 = i0 -1;

  // If waypoint_dist_thresh is enforced, check if the goal itself should be
  // returned as the carrot only after the waypoint distance criteria has been
  // checked
  if ((plan_path_[0].loc - robot_loc_).squaredNorm() < kSqCarrotDist) {
    // Goal is within the carrot dist.
    return plan_path_[0].loc;
  }

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

void Navigation::GetFreePathLength(float curvature,
                                   float* free_path_length,
                                   float* clearance,
                                   Vector2f* obstruction) {
  if (fabs(curvature) < kEpsilon) {
    GetStraightFreePathLength(free_path_length, clearance);
    return;
  }
  const float path_radius = 1.0 / curvature;
  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * params_.robot_length - params_.base_link_offset + params_.obstacle_margin;
  const float w = 0.5 * params_.robot_width + params_.obstacle_margin;
  const Vector2f c(0, path_radius );
  const float s = ((path_radius > 0.0) ? 1.0 : -1.0);
  const Vector2f inner_front_corner(l, s * w);
  const Vector2f outer_front_corner(l, -s * w);
  const float r1 = max<float>(0.0f, fabs(path_radius) - w);
  const float r1_sq = Sq(r1);
  const float r2_sq = (inner_front_corner - c).squaredNorm();
  const float r3_sq = (outer_front_corner - c).squaredNorm();
  float angle_min = M_PI;
  *obstruction = Vector2f(-params_.max_free_path_length, 0);
  for (size_t i = 0; i < fp_point_cloud_.size(); ++i) {
    const Vector2f& p = fp_point_cloud_[i];
    if (p.x() < 0.0f) continue;
    const float r_sq = (p - c).squaredNorm();
    // printf("c:%.2f r:%.3f r1:%.3f r2:%.3f r3:%.3f\n",
    //        curvature, sqrt(r_sq), r1, sqrt(r2_sq), sqrt(r3_sq));
    if (r_sq < r1_sq || r_sq > r3_sq) continue;
    const float r = sqrt(r_sq);
    const float theta = ((curvature > 0.0f) ?
      atan2<float>(p.x(), path_radius - p.y()) :
      atan2<float>(p.x(), p.y() - path_radius));
    float alpha;
    if (r_sq < r2_sq) {
      // Point will hit the side of the robot first.
      alpha = acosf((fabs(path_radius) - w) / r);
    } else {
      // Point will hit the front of the robot first.
      alpha = asinf(l / r);
    }
    // if (theta < 0.0f) continue;
    const float path_length =
        max<float>(0.0f, fabs(path_radius) * (theta - alpha));
    if (*free_path_length > path_length) {
      *free_path_length = path_length;
      *obstruction = p;
      angle_min = theta;
    }
  }
  *free_path_length = max(0.0f, *free_path_length);
  angle_min = min<float>(angle_min, *free_path_length * fabs(curvature));
  *clearance = params_.max_clearance;

  for (const Vector2f& p : fp_point_cloud_) {
    const float theta = ((curvature > 0.0f) ?
        atan2<float>(p.x(), path_radius - p.y()) :
        atan2<float>(p.x(), p.y() - path_radius));
    if (theta < angle_min && theta > 0.0) {
      const float r = (p - c).norm();
      const float current_clearance = fabs(r - fabs(path_radius));
      if (*clearance > current_clearance) {
        *clearance = current_clearance;
      }
    }
  }
  *clearance = max(0.0f, *clearance);
}

DEFINE_double(dw, 1, "Distance weight");
DEFINE_double(cw, -0.5, "Clearance weight");
DEFINE_double(fw, -1, "Free path weight");
DEFINE_double(subopt, 1.5, "Max path increase for clearance");

PathOption GetBestOption(const vector<PathOption>& options,
                         const Vector2f& target) {
  static const bool kDebug = false;

  PathOption best_option;
  best_option.clearance = 0;
  best_option.free_path_length = 0;
  best_option.curvature = 0;

  float closest_dist = FLT_MAX;
  bool option_to_goal_exists = false;
  for (const PathOption& o : options) {
    if (o.free_path_length > 0.0f && o.dist_to_goal < 100.0f) {
      option_to_goal_exists = true;
      break;
    }
  }
  for (const PathOption& o : options) {
    if (o.free_path_length <= 0.0f) continue;
    const float d = (option_to_goal_exists ?
        (o.free_path_length + o.dist_to_goal) : GetClosestDistance(o, target));

    if (d < closest_dist) {
      closest_dist = d;
      best_option = o;
    }
  }
  const float d = (option_to_goal_exists ?
        (best_option.free_path_length + best_option.dist_to_goal) :
        GetClosestDistance(best_option, target));
  float best_cost = FLAGS_dw * d +
      FLAGS_fw * best_option.free_path_length +
      FLAGS_cw * best_option.clearance;
  if (kDebug) {
    printf("\nBest: %f %f %f %f %f = %f\n",
           best_option.curvature,
           best_option.free_path_length,
           best_option.clearance,
           best_option.dist_to_goal,
           best_option.clearance_to_goal,
           best_cost);
  }
  for (const PathOption& o : options) {
    const float d = (option_to_goal_exists ?
        (o.free_path_length + o.dist_to_goal) : GetClosestDistance(o, target));
    if (d < closest_dist * FLAGS_subopt) {
      const float cost = FLAGS_dw * d +
          FLAGS_fw * o.free_path_length +
          FLAGS_cw * o.clearance;
      if (cost < best_cost) {
        best_option = o;
        best_cost = cost;
        if (kDebug) {
          printf("Better: %f %f %f %f %f = %f\n",
                best_option.curvature,
                best_option.free_path_length,
                best_option.clearance,
                best_option.dist_to_goal,
                best_option.clearance_to_goal,
                cost);
        }
      } else if (kDebug) {
        printf("NOT Better: %f %f %f %f %f = %f\n",
              o.curvature,
              o.free_path_length,
              o.clearance,
              o.dist_to_goal,
              o.clearance_to_goal,
              cost);
      }
    } else {
      if (kDebug) {
        printf("NOT Considered: %f %f %f %f %f\n",
              o.curvature,
              o.free_path_length,
              o.clearance,
              o.dist_to_goal,
              o.clearance_to_goal);
      }
    }
  }
  // TODO: Check a window of options areound the best. If the best has
  // clearance less than ideally desired, and any have a better
  // clearance within th chosen suboptimality, prefer that.
  return best_option;
}

void Navigation::ApplyDynamicConstraints(vector<PathOption>* options_ptr) {
  vector<PathOption>& options = *options_ptr;
  const float max_domega = params_.dt * params_.angular_limits.accel;
  const float stopping_dist =
      robot_vel_.squaredNorm() / (2.0 * params_.linear_limits.decel);
  for (PathOption& o : options) {
    if (o.free_path_length < stopping_dist) {
      o.free_path_length = 0;
      o.dist_to_goal = FLT_MAX;
      o.clearance = 0;
    }
    const float omega_next = robot_vel_.x() * o.curvature;
    if (omega_next < robot_omega_ - max_domega ||
        omega_next > robot_omega_ + max_domega) {
      o.free_path_length = 0;
      o.dist_to_goal = FLT_MAX;
    }
  }
}

void Navigation::GetPathOptions(vector<PathOption>* options_ptr) {
  vector<PathOption>& options = *options_ptr;
  if (false) {
    options = {
      PathOption(-0.1),
      PathOption(0),
      PathOption(0.1),
    };
    return;
  }
  const float max_domega = params_.dt * params_.angular_limits.accel;
  const float max_dv = params_.dt * params_.linear_limits.accel;
  const float robot_speed = fabs(robot_vel_.x());
  float c_min = -FLAGS_max_curvature;
  float c_max = FLAGS_max_curvature;
  if (robot_speed > max_dv + 0.001) {
    c_min = max<float>(
        c_min, (robot_omega_ - max_domega) / (robot_speed - max_dv));
    c_max = min<float>(
        c_max, (robot_omega_ + max_domega) / (robot_speed - max_dv));
  }
  const float dc = (c_max - c_min) / static_cast<float>(
      params_.num_options - 1);
  // printf("Options: %6.2f : %6.2f : %6.2f\n", c_min, dc, c_max);
  if (false) {
    for (float c = c_min; c <= c_max; c+= dc) {
      PathOption o;
      o.curvature = c;
      options.push_back(o);
    }
  } else {
    const float dc = (2.0f * FLAGS_max_curvature) /
        static_cast<float>(params_.num_options - 1);
    for (float c = -FLAGS_max_curvature; c <= FLAGS_max_curvature; c+= dc) {
      PathOption o;
      o.curvature = c;
      options.push_back(o);
    }
  }
}

float Clearance(const Line2f& l, const vector<Vector2f>& points) {
  const Vector2f d = l.Dir();
  const float len = l.Length();
  float clearance = 10;
  for (const Vector2f& p  : points) {
    const float x = d.dot(p - l.p0);
    if (x < 0.0f || x > len) continue;
    clearance = min<float>(clearance, l.Distance(p));
  }
  return clearance;
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

template <class State, class Visualizer>
bool Navigation::QueryMDPSolver(uint64_t start_id,
                                uint64_t goal_id,
                                Visualizer* const viz,
                                std::vector<State>* path) {
  const bool kDebug = false;
  bool plan_on_static_graph = false;
  path->clear();

  // TODO(srabiee): This feature can be safely removed when kEnableReducedGraph
  // is enabled in Plan()
  if (params_.frequentist_mode && false) {
    // If the start and goal location are both close to nodes on
    // the static graph, planning will happen on the static graph.
    // This is specifically used in the frequentist competence-aware
    // mode

    // Max acceptable distance of the start and goal location to the
    // closest static state in order to use the static node instead of
    // adding a new dynamic state
    const float kMaxDistanceToStaticState = 3.0;

    uint64_t closest_static_state_id_st = 0;
    uint64_t closest_static_state_id_goal = 0;
    float dist_to_closest_static_state_st = 0;
    float dist_to_closest_static_state_goal = 0;
    bool found_st = planning_domain_.GetClosestStaticState(
        planning_domain_.KeyToState(start_id).loc,
        &closest_static_state_id_st,
        &dist_to_closest_static_state_st);
    bool found_goal = planning_domain_.GetClosestStaticState(
        planning_domain_.KeyToState(goal_id).loc,
        &closest_static_state_id_goal,
        &dist_to_closest_static_state_goal);
    if (found_st && found_goal) {
      if (dist_to_closest_static_state_st < kMaxDistanceToStaticState &&
          dist_to_closest_static_state_goal < kMaxDistanceToStaticState) {
        start_id = closest_static_state_id_st;
        goal_id = closest_static_state_id_goal;
        plan_on_static_graph = true;
      }
    }
  }

  // Prep the full map in json
  json graph_json;
  if (plan_on_static_graph) {
    graph_json = planning_domain_.GetStaticGraphInJSON();
  } else {
    graph_json = planning_domain_.GetGraphInJSON();
  }
  std::stringstream graph_ss;
  graph_ss << std::setw(4) << graph_json << std::endl;

  // Call the mdp solver service
  MDPSolver mdp_srv;
  mdp_srv.request.map_info = graph_ss.str();
  mdp_srv.request.start = start_id;
  mdp_srv.request.goal = goal_id;

  if (kDebug) {
    std::cout << "JSON map sent to the MDP solver: " << std::endl;
    std::cout << mdp_srv.request.map_info << std::endl;
    std::cout << "start_id: " << start_id << std::endl;
    std::cout << "goal_id: " << goal_id << std::endl;
  }

  if (mdp_client_.call(mdp_srv)) {
    LOG(INFO) << ("Response received from MDP solver.");
  } else {
    LOG(INFO) << ("Failed to call the MDP solver!");
    return false;
  }

  if (mdp_srv.response.plan.empty()) {
    return false;
  }

  // Populate the path
  for (auto& node_id : mdp_srv.response.plan) {
    path->push_back(planning_domain_.KeyToState(node_id));
  }
  std::reverse(path->begin(), path->end());

  if (kDebug) {
    std::cout << "Generated path: " << std::endl;
    for (auto& state : *path) {
      std::cout << state.id << ", ";
    }
    std::cout << std::endl;
  }

  for (size_t i = 0; i < path->size() - 1; i++) {
    viz->DrawEdge(path->at(i), path->at(i + 1));
  }

  return true;
}

bool Navigation::GetDynamicEdgesFailureProb(
    std::vector<graph_navigation::IntrospectivePerceptionInfo>*
        edges_failure_prob,
    const std::vector<uint64_t> &ignore_node_ids) {
  if (!initialized_) {
    return false;
  }
  // TODO(srabiee): Add the failure type number to the
  // IntrospectivePerceptionInfo.msg and use that instead of hardcoding the
  // value here
  const int kFailureTypeCount = 2;

  std::unordered_map<std::string, std::unordered_map<int, float>>
      edge_to_failure_prob_map;

  for (auto& failure : failure_data_) {
    bool is_dyanmic = false;
    navigation::GraphDomain::NavigationEdge edge;
    float closest_dist = FLT_MAX;
    if (!planning_domain_.GetClosestEdge(
            failure.location, 
            ignore_node_ids, 
            &edge, 
            &closest_dist, 
            &is_dyanmic)) {
      continue;
    }

    if (is_dyanmic) {
      // Calculate the angle between the the failure pose and the edge
      // direction. Navigation edges will be treated as directed for
      // competence-aware planning
      string edge_id;
      Eigen::Vector2f edge_dir = edge.edge.Dir();
      float edge_angle = atan2(edge_dir.y(), edge_dir.x());
      float angle_dist = math_util::AngleDist(failure.angle, edge_angle);
      if (angle_dist > M_PI_2) {
        // The failure pose is oriented in the opposite direction of the edge
        edge_id = std::to_string(edge.s1_id) + "," + std::to_string(edge.s0_id);
      } else {
        // The failure pose is oriented in the direction of the edge
        edge_id = std::to_string(edge.s0_id) + "," + std::to_string(edge.s1_id);
      }

      if (edge_to_failure_prob_map.find(edge_id) !=
          edge_to_failure_prob_map.end()) {
        if (edge_to_failure_prob_map[edge_id].find(failure.failure_type) !=
            edge_to_failure_prob_map[edge_id].end()) {
          edge_to_failure_prob_map[edge_id][failure.failure_type] =
              std::max(failure.failure_prob,
                       edge_to_failure_prob_map[edge_id][failure.failure_type]);
        } else {
          edge_to_failure_prob_map[edge_id][failure.failure_type] =
              failure.failure_prob;
        }
      } else {
        std::unordered_map<int, float> failure_type_to_prob_map;
        failure_type_to_prob_map.insert(
            make_pair(failure.failure_type, failure.failure_prob));
        edge_to_failure_prob_map.insert(
            std::make_pair(edge_id, failure_type_to_prob_map));
      }
    }
  }

  std::vector<navigation::GraphDomain::NavigationEdge> dynamic_edges;
  planning_domain_.GetDynamicEdges(&dynamic_edges);

  // Go through all dynamic edges and generate an IntrospectivePerceptionInfo
  // message for each direction of that edge and for each type of failure
  for (auto& dynamic_edge : dynamic_edges) {
    // Each navigation edge is converted to two directed edges
    std::array<std::string, 2> directed_edge_ids;
    directed_edge_ids[0] = std::to_string(dynamic_edge.s0_id) + "," +
                           std::to_string(dynamic_edge.s1_id);
    directed_edge_ids[1] = std::to_string(dynamic_edge.s1_id) + "," +
                           std::to_string(dynamic_edge.s0_id);

    // Check if failures are available for either direction of the edge
    for (size_t i = 0; i < directed_edge_ids.size(); i++) {
      uint64_t s0_id, s1_id;
      if (i == 1) {
        s0_id = dynamic_edge.s1_id;
        s1_id = dynamic_edge.s0_id;
      } else {
        s0_id = dynamic_edge.s0_id;
        s1_id = dynamic_edge.s1_id;
      }
      if (edge_to_failure_prob_map.find(directed_edge_ids[i]) !=
          edge_to_failure_prob_map.end()) {
        // Check each type of failure separately
        for (int j = 0; j < kFailureTypeCount; j++) {
          if (edge_to_failure_prob_map[directed_edge_ids[i]].find(j) !=
              edge_to_failure_prob_map[directed_edge_ids[i]].end()) {
            edges_failure_prob->push_back(
                GenerateIntrospectivePerceptionInfoMsg(
                    s0_id,
                    s1_id,
                    edge_to_failure_prob_map[directed_edge_ids[i]][j],
                    j));
          } else {
            edges_failure_prob->push_back(
                GenerateIntrospectivePerceptionInfoMsg(s0_id, s1_id, 0.0, j));
          }
        }
      } else {
        for (int j = 0; j < kFailureTypeCount; j++) {
          edges_failure_prob->push_back(
              GenerateIntrospectivePerceptionInfoMsg(s0_id, s1_id, 0.0, j));
        }
      }
    }
  }

  return true;
}

bool Navigation::GetStaticEdgesFailureProb(
    bool get_frequentist_estimates,
    std::vector<graph_navigation::IntrospectivePerceptionInfo>*
        edges_failure_prob) {
  if (!initialized_) {
    return false;
  }
  // TODO(srabiee): Add the failure type number to the
  // IntrospectivePerceptionInfo.msg and use that instead of hardcoding the
  // value here
  const int kFailureTypeCount = 2;
  std::array<float, kFailureTypeCount> failure_prob_fwd {0, 0};
  std::array<float, kFailureTypeCount> failure_prob_rev {0, 0};
  // Prob values smaller than this threshold will be set to zero
  const float kEpsilonThresh = 0.025;

  for (auto& edge : planning_domain_.static_edges) {
    // Compute the failure prob for forward direction
    for (size_t i = 0; i < kFailureTypeCount; i++) {
      failure_prob_fwd[i] = 0.0;
      if (get_frequentist_estimates) {
        if (edge.traversal_count[0] > 0) {
          failure_prob_fwd[i] = static_cast<float>(edge.failure_count_fwd[i]) /
                                static_cast<float>(edge.traversal_count[0]);
          failure_prob_fwd[i] =
              std::min(static_cast<float>(failure_prob_fwd[i]), 0.99f);
          if (failure_prob_fwd[i] < kEpsilonThresh) {
            failure_prob_fwd[i] = 0;
          }
        }
      } else {
        failure_prob_fwd[i] = edge.failure_belief_normalized[0][i];
        if (failure_prob_fwd[i] < kEpsilonThresh) {
          failure_prob_fwd[i] = 0;
        }
      }

      edges_failure_prob->push_back(GenerateIntrospectivePerceptionInfoMsg(
          edge.s0_id, edge.s1_id, failure_prob_fwd[i], i));
    }

    // Compute the failure prob for reverse direction
    for (size_t i = 0; i < kFailureTypeCount; i++) {
      failure_prob_rev[i] = 0.0;
      if (get_frequentist_estimates) {
        if (edge.traversal_count[1] > 0) {
          failure_prob_rev[i] = static_cast<float>(edge.failure_count_rev[i]) /
                                static_cast<float>(edge.traversal_count[1]);
          failure_prob_rev[i] =
              std::min(static_cast<float>(failure_prob_rev[i]), 0.99f);
          if (failure_prob_rev[i] < kEpsilonThresh) {
            failure_prob_rev[i] = 0;
          }
        }
      } else {
        failure_prob_rev[i] = edge.failure_belief_normalized[1][i];
        if (failure_prob_rev[i] < kEpsilonThresh) {
          failure_prob_rev[i] = 0;
        }
      }

      edges_failure_prob->push_back(GenerateIntrospectivePerceptionInfoMsg(
          edge.s1_id, edge.s0_id, failure_prob_rev[i], i));
    }
  }

  return true;
}

void Navigation::PublishDynamicEdgesFailureInfo(
    const std::vector<uint64_t> &ignore_node_ids) {
  std::vector<graph_navigation::IntrospectivePerceptionInfo> edges_failure_prob;
  graph_navigation::IntrospectivePerceptionInfoArray msg;
  if (GetDynamicEdgesFailureProb(&edges_failure_prob, ignore_node_ids)) {
    for (auto& detection : edges_failure_prob) {
      msg.detections.push_back(detection);
    }

    introspective_perception_pub_.publish(msg);
  }
}

void Navigation::PublishAllEdgesFailureInfo(
    bool publish_frequentist_estimates,
    const std::vector<uint64_t> &ignore_node_ids) {
  std::vector<graph_navigation::IntrospectivePerceptionInfo>
      dyn_edges_failure_prob;
  std::vector<graph_navigation::IntrospectivePerceptionInfo>
      static_edges_failure_prob;
  graph_navigation::IntrospectivePerceptionInfoArray msg;

  // Estimate failure prob. for all dynamic edges
  if (GetDynamicEdgesFailureProb(&dyn_edges_failure_prob, ignore_node_ids)) {
    for (auto& detection : dyn_edges_failure_prob) {
      msg.detections.push_back(detection);
    }
  }

  // Retrieve failure prob. for all static edges
  if (GetStaticEdgesFailureProb(publish_frequentist_estimates,
                                &static_edges_failure_prob)) {
    for (auto& detection : static_edges_failure_prob) {
      msg.detections.push_back(detection);
    }
  }

  introspective_perception_pub_.publish(msg);
}

void Navigation::UpdatePlanProgress(int plan_progress_idx) {
  const bool kDebug = false;
  size_t next_sid = std::max((plan_progress_idx - 1), 0);

  if (kDebug) {
    std::cout << "Update plan prog " << plan_progress_idx
              << "S_ID: " << plan_path_[plan_progress_idx].id << " -> "
              << plan_path_[next_sid].id << std::endl;
    std::cout << "plan path: ";
    for (size_t i = 0; i < plan_path_.size(); i++) {
      std::cout << plan_path_[i].id;
      std::cout << " ";
    }
    std::cout << std::endl;
  }

  if (!params_.lock_traversal_stats) {
    if (plan_progress_idx < static_cast<int>(plan_path_.size() - 1) &&
        (plan_progress_idx != curr_plan_progress_)) {
      planning_domain_.UpdateEdgeTraversalStats(
          plan_path_[curr_plan_progress_].id,
          plan_path_[plan_progress_idx].id,
          1);
    }
  }

  curr_plan_progress_ = plan_progress_idx;
}

DEFINE_double(tx, 0.4, "Test obstacle point - X");
DEFINE_double(ty, -0.38, "Test obstacle point - Y");

void Navigation::RunObstacleAvoidance() {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  static const bool kDebug = false;
  float curvature_cmd = 0;
  float velocity_cmd = 0;

  float max_map_speed = params_.linear_limits.speed;
  planning_domain_.GetClearanceAndSpeedFromLoc(
      robot_loc_, nullptr, &max_map_speed);

  if (false) {
    fp_point_cloud_ = {
      Vector2f(FLAGS_tx, FLAGS_ty)
    };
    for (const Vector2f& v : fp_point_cloud_) {
      visualization::DrawCross(v, 0.05, 0xFF0000, local_viz_msg_);
    }
  }

  vector<PathOption> path_options;
  GetPathOptions(&path_options);

  for (PathOption& o : path_options) {
    if (fabs(o.curvature) < kEpsilon) {
      o.free_path_length = min(params_.max_free_path_length, local_target_.x());
    } else {
      const float turn_radius = 1.0f / o.curvature;
      const Vector2f turn_center(0, turn_radius);
      const Vector2f target_radial = local_target_ - turn_center;
      const Vector2f middle_radial =
          fabs(turn_radius) * target_radial.normalized();
      const float middle_angle =
          atan2(fabs(middle_radial.x()), fabs(middle_radial.y()));
      o.free_path_length =
          min<float>(params_.max_free_path_length, middle_angle * fabs(turn_radius));
      o.free_path_length =
          min<float>(o.free_path_length, fabs(turn_radius) * M_PI_2);
    }
    // o.free_path_length = 1;
    GetFreePathLength(
        o.curvature, &o.free_path_length, &o.clearance, &o.obstruction);
    o.closest_point = GetClosestApproach(o, local_target_);
    o.clearance_to_goal =
        Clearance(Line2f(GetFinalPoint(o), local_target_), fp_point_cloud_);
    // visualization::DrawCross(GetFinalPoint(o), 0.1, 0x00C0C0, local_viz_msg_);
    // o.clearance_to_goal = max(0.0f, o.clearance_to_goal - w);
    if (o.clearance_to_goal > 0.0f) {
      o.dist_to_goal = (o.closest_point - local_target_).norm();
    } else {
      o.dist_to_goal = FLT_MAX;
    }
    if (kDebug) {
      // printf("%3.2f ", o.free_path_length);
      printf("%3.2f|%3.2f|%f\n", o.curvature, o.free_path_length, o.clearance_to_goal);
    }
  }
  if (kDebug) printf("\n");
  ApplyDynamicConstraints(&path_options);
  for (const auto& o : path_options) {
    visualization::DrawPathOption(o.curvature,
                                  o.free_path_length,
                                  o.clearance,
                                  0x0000FF,
                                  local_viz_msg_);
  }
  const PathOption best_option = GetBestOption(path_options, local_target_);
  visualization::DrawPathOption(best_option.curvature,
                                best_option.free_path_length,
                                best_option.clearance,
                                0xFF0000,
                                local_viz_msg_);

  const Vector2f closest_point = GetClosestApproach(best_option, local_target_);
  visualization::DrawCross(closest_point, 0.05, 0x00A000, local_viz_msg_);
  curvature_cmd = best_option.curvature;
  const float dist_left =
      max<float>(0.0, best_option.free_path_length - params_.obstacle_margin);


  const float speed = robot_vel_.norm();
  float max_speed = min<float>(params_.linear_limits.speed,
      sqrt(2.0f * params_.linear_limits.accel * best_option.clearance));
  max_speed = min(max_map_speed, max_speed);
  velocity_cmd = Run1DTOC(0, dist_left, speed, max_speed, params_.linear_limits.accel, params_.linear_limits.decel, params_.dt);
  SendCommand(velocity_cmd, curvature_cmd);
}

void Navigation::Halt() {
  const float kEpsSpeed = 0.01;
  const float velocity = robot_vel_.x();
  float velocity_cmd = 0;
  if (fabs(velocity) > kEpsSpeed) {
    const float dv = params_.linear_limits.decel * params_.dt;
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
  SendCommand(velocity_cmd, 0);
}

void Navigation::TurnInPlace() {
  // Scale down the max angular velocity for turning in place
  const float kAngularVelLimitScalar = 0.3;
  const float kMaxLinearSpeed = 0.1;
  const float velocity = robot_vel_.x();
  // If in airsim_compatible mode, skip a number of frames
  // to receive the updated location info before calling
  // the turn_in_place service
  const int kAirSimSkipFrames = 3;
  static int airsim_skip_counter = 0;

  float angular_cmd = 0;
  if (fabs(velocity) > kMaxLinearSpeed) {
    Halt();
    return;
  }

  const float goal_theta = atan2(local_target_.y(), local_target_.x());
  const float dv = params_.dt * params_.angular_limits.accel;

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
    angular_cmd = s * Run1DTOC(0, s * goal_theta, s * robot_omega_,
        params_.angular_limits.speed * kAngularVelLimitScalar, params_.angular_limits.accel,
        params_.angular_limits.decel, params_.dt);
  }
  // TODO: Motion profiling for omega.
  // printf("TurnInPlace: %8.3f %8.3f %8.3f\n",
  //        RadToDeg(goal_theta),
  //        RadToDeg(robot_omega_),
  //        RadToDeg(angular_cmd));
  const float curvature = Sign(goal_theta) * 10000.0;
  const float velocity_cmd = angular_cmd / curvature;
  SendCommand(velocity_cmd, curvature);

  if (params_.airsim_compatible && enabled_) {
    airsim_skip_counter++;
    if (airsim_skip_counter % kAirSimSkipFrames != 0) {
      return;
    }

    // Call the AirSim's turn-in-place service
    graph_navigation::TurnInPlace turn_in_place_srv;
    turn_in_place_srv.request.vehicle_name = "PhysXCar";
    turn_in_place_srv.request.yaw = goal_theta;

    if (turn_in_place_client_.call(turn_in_place_srv)) {
      LOG(INFO) << ("Response received from the turn_in_place service.");
    } else {
      LOG(INFO) << ("Failed to call the turn_in_place service!");
    }

    return;
  }
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
  drive_msg_.curvature = 0;
  drive_msg_.velocity = v_cmd;
  drive_msg_.header.stamp = ros::Time::now();
  ackermann_drive_pub_.publish(drive_msg_);
  twist_drive_pub_.publish(AckermannToTwist(drive_msg_));
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
  nav_aborted_ = true;
}

bool Navigation::GetClosestStaticEdgeInfo(uint64_t* s0_id, uint64_t* s1_id) {
  if (!initialized_) {
    return false;
  }
  
  navigation::GraphDomain::NavigationEdge closest_edge;
  float closest_dist = FLT_MAX;
  if (!planning_domain_.GetClosestStaticEdge(
          robot_loc_, &closest_edge, &closest_dist)) {
    return false;
  }

  // Calculate the direction of robot relative to the edge
  Eigen::Vector2f edge_dir = closest_edge.edge.Dir();
  float edge_angle = atan2(edge_dir.y(), edge_dir.x());
  float angle_dist = math_util::AngleDist(robot_angle_, edge_angle);
  if (angle_dist > M_PI_2) {
    // The robot is moving in the opposite direction of the edge
    *s0_id = closest_edge.s1_id;
    *s1_id = closest_edge.s0_id;
  } else {
    // The robot is moving in the direction of the edge
    *s0_id = closest_edge.s0_id;
    *s1_id = closest_edge.s1_id;
  }

  return true;
}

void Navigation::SaveFailureInstancesToFile(const std::string& output_path) {
  if (!params_.competence_aware) {
    return;
  }

  json json_obj;
  std::vector<json> failure_instances_json;
  for (const FailureData& failure : failure_data_) {
    failure_instances_json.push_back(failure.toJSON());
  }
  json_obj["failure_instances"] = failure_instances_json;

  std::vector<json> edge_json;
  for (const navigation::GraphDomain::NavigationEdge& e :
       planning_domain_.static_edges) {
    edge_json.push_back(e.TraversalInfoToJSON());
  }
  json_obj["edges"] = edge_json;

  std::ofstream output_file(output_path);
  output_file << std::setw(4) << json_obj << std::endl;
  output_file.close();
}

void Navigation::LoadFailureInstancesFromFile(const std::string& file) {
  const bool kDebug = false;

  if (!FileExists(file)) {
    LOG(WARNING) << "Failure logs file was not found: " << file;
    return;
  }

  std::ifstream f(file);
  json json_obj;
  f >> json_obj;
  f.close();

  // Load individual filure instances
  CHECK(json_obj["failure_instances"].is_array());
  auto const failure_instances_json = json_obj["failure_instances"];

  failure_data_.clear();

  for (const json& j : failure_instances_json) {
    FailureData failure_instance = FailureData::fromJSON(j);
    failure_data_.push_back(failure_instance);
  }
  printf("Loaded %s with %lu failure instances\n",
         file.c_str(),
         failure_instances_json.size());

  // Load logged edge traversal statistics
  CHECK(json_obj["edges"].is_array());
  auto const edges_json = json_obj["edges"];
  for (const json& j : edges_json) {
    size_t edge_id = 0;
    int edge_direction = 0;
    if(planning_domain_.GetStaticEdgeID(j["s0_id"].get<uint64_t>(),
                       j["s1_id"].get<uint64_t>(), 
                       &edge_id,
                       &edge_direction)) {
      planning_domain_.static_edges[edge_id].UpdateTraversalInfoFromJSON(j);

    }
  }

  if (kDebug) {
    planning_domain_.PrintEdgeTraversalStats();
  }
}

DEFINE_uint32(failure_data_queue_size, 200, "Maximum number of failure"     
              " instances to be stored for association with dynamic navigation" " edges.");

void Navigation::AddFailureInstance(
    const graph_navigation::IntrospectivePerceptionRawInfo& msg) {
  // The offset between the current robot location and the location
  // to log the predicted failure at (meters).
  const float kFailureLocationOffset = 3.0;

  // TODO(srabiee): turn off debug mode
  const bool kDebug = false;

  if (initialized_) {
    if (!params_.only_collect_traversal_stats) {
      // In only_collect_traversal_stats mode, experienced or predicted failures
      // do not trigger replanning
      replan_requested_ = true;
    }
    introspection::FailureData failure_data;

    introspection::ObservationType obs_type;
    if (msg.source == "ivoa") {
      obs_type = INTROSPECTION;
    } else if (msg.source == "simple_replanner") {
      obs_type = EXPERIENCE;
    } else {
      LOG(WARNING) << "Unexpected source of introspective perception msg: " << 
                      msg.source << ". Ignoring this received msg.";
      return;
    }

    failure_data.failure_prob = msg.failure_likelihood;
    failure_data.failure_type = msg.failure_type;
    failure_data.angle = robot_angle_;
    // Tag the location for the failure that has been observed at current
    // time to kFailureLocationOffset meters in front of the robot
    failure_data.location =
        robot_loc_ +
        kFailureLocationOffset * Vector2f(cos(robot_angle_), sin(robot_angle_));

    // Remove all the stored failure data before adding this new one if in
    // memoryless mode.
    if (params_.memoryless) {
      failure_data_.clear();
    }
    failure_data_.push_back(failure_data);

    // Remove enough of the oldest failure instances
    // to prevent exceeding the allocated buffer
    while (failure_data_.size() > FLAGS_failure_data_queue_size) {
      failure_data_.pop_front();
    }

    // Update the traversal stats of the current edge
    // TODO(srabiee): Prune out duplicate reports
    if ((params_.frequentist_mode || params_.bayes_mode) &&
        obs_type == EXPERIENCE && !params_.lock_traversal_stats) {
      std::array<uint64_t, 2> failure_count{0, 0};
      CHECK_LT(failure_data.failure_type, failure_count.size());
      failure_count[failure_data.failure_type] = 1;
      if (curr_plan_progress_ > 0) {
        planning_domain_.UpdateEdgeTraversalStats(
            plan_path_[curr_plan_progress_].id,
            plan_path_[curr_plan_progress_ - 1].id,
            1,
            failure_count);
      }
    }

    if (params_.bayes_mode && obs_type == INTROSPECTION &&
        !params_.lock_traversal_stats) {
      // TDOO(srabiee): The two following function calls can be implemented in
      // a single function for a more efficient solution
      float kEps = 0.02;
      uint64_t s0_id, s1_id;
      bool navigation_edge_found = GetClosestStaticEdgeInfo(&s0_id, &s1_id);
      if (navigation_edge_found) {
        // TODO(srabiee): pass the failure prob for all types of failures in
        // the introspection info msg
        kEps = std::min(kEps, 1.0f - failure_data.failure_prob);
        std::array<float, 3> observation = {kEps, kEps, kEps};
        observation[failure_data.failure_type] = failure_data.failure_prob;
        observation[2] = 1.0 - (failure_data.failure_prob + kEps);

        size_t edge_id = 0;
        int edge_direction = 0;
        planning_domain_.GetStaticEdgeID(
            s0_id, s1_id, &edge_id, &edge_direction);
        planning_domain_.static_edges[edge_id].UpdateFailureBelief(
            edge_direction, observation);

        if (kDebug) {
          planning_domain_.PrintFailureBelief();
        }
        
      }
    }
  }
}

void Navigation::Run() {
  if (!initialized_) return;
  visualization::ClearVisualizationMsg(local_viz_msg_);
  DrawRobot();
  if (!odom_initialized_) return;
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
    Halt();
    if (nav_aborted_) {
      status_msg_.status = status_msg_.ABORTED;
    } else {
      status_msg_.status = status_msg_.SUCCEEDED;
    }
    status_pub_.publish(status_msg_);
    viz_pub_.publish(local_viz_msg_);
    return;
  }
  status_msg_.status = status_msg_.ACTIVE;
  status_pub_.publish(status_msg_);
  if (!PlanStillValid() || replan_requested_) {
    Plan();
    replan_requested_ = false;
  }

  if (plan_path_.size() < 2) {
    // No path was found
    status_msg_.status = status_msg_.LOST;
    nav_aborted_ = true;
    nav_complete_ = true;
    status_pub_.publish(status_msg_);
    viz_pub_.publish(local_viz_msg_);
    return;
  }

  // Get Carrot.
  const Vector2f carrot = GetCarrot();
  carrot_pub_.publish(CarrotToNavMsgsPath(carrot));
  auto msg_copy = global_viz_msg_;
  visualization::DrawCross(carrot, 0.2, 0x10E000, msg_copy);
  visualization::DrawArc(Vector2f(0, 0),
                         params_.carrot_dist,
                         -M_PI,
                         M_PI,
                         0xE0E0E0,
                         local_viz_msg_);
  viz_pub_.publish(msg_copy);
  // Check if complete.
  nav_complete_ = (robot_loc_ - carrot).norm() < FLAGS_navigation_tolerance;
  // Run local planner.
  if (nav_complete_) {
    Halt();
    if (params_.competence_aware && params_.frequentist_mode) {
      if (curr_plan_progress_ != 0) {
        UpdatePlanProgress(0);
      }
    }
  } else {
    // TODO check if the robot needs to turn around.
    local_target_ = Rotation2Df(-robot_angle_) * (carrot - robot_loc_);
    float local_fov_rad = DegToRad(params_.local_fov);
    const float theta = atan2(local_target_.y(), local_target_.x());
    if (local_target_.squaredNorm() > Sq(params_.carrot_dist)) {
      local_target_ = params_.carrot_dist * local_target_.normalized();
    }
    visualization::DrawCross(local_target_, 0.2, 0xFF0080, local_viz_msg_);
    // printf("Local target: %8.3f, %8.3f (%6.1f\u00b0)\n",
    //        local_target_.x(),
    //        local_target_.y(),
    //        RadToDeg(theta));
    if (fabs(theta) > local_fov_rad && FLAGS_can_turn_in_place) {
      printf("TurnInPlace\n");
      TurnInPlace();
    } else {
      // printf("ObstAv\n");
      RunObstacleAvoidance();
    }
  }
  viz_pub_.publish(local_viz_msg_);
}

}  // namespace navigation
