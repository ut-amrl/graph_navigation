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
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "navigation.h"
#include "geometry_msgs/PoseStamped.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "shared/util/timer.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "astar.h"

#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "deep_cost_map_evaluator.h"
#include "linear_evaluator.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using navigation::MotionLimits;
using navigation::Odom;
using navigation::Twist;
using std::atan2;
using std::deque;
using std::max;
using std::min;
using std::swap;
using std::shared_ptr;
using std::string;
using std::vector;

using namespace math_util;
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

namespace {
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

DEFINE_int32(num_options, 41, "Number of options to consider");

// TODO(jaholtz) figure out how to handle this visualization without
// having astar contain ros dependencies
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
    // visualization::DrawLine(s1, s2, 0x606060, global_viz_msg_);
    // viz_pub_.publish(global_viz_msg_);
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
    // visualization::DrawLine(s1.loc, s2.loc, 0xC0C0C0, global_viz_msg_);
    // viz_pub_.publish(global_viz_msg_);
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
    nav_state_(NavigationState::kStopped),
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
                            const string& map_file) {
  // Initialize status message
  params_ = params;
  planning_domain_ = GraphDomain(map_file, &params_);
  initialized_ = true;
  sampler_->SetNavParams(params);

  PathEvaluatorBase* evaluator = nullptr;
  if (params_.evaluator_type == "cost_map") {
    auto cost_map_evaluator = new DeepCostMapEvaluator(params_);
    cost_map_evaluator->LoadModel();
    evaluator = (PathEvaluatorBase*) cost_map_evaluator;
  } else if (params_.evaluator_type == "linear") {
    evaluator = (PathEvaluatorBase*) new LinearEvaluator();
  } else {
    printf("Uknown evaluator type %s\n", params_.evaluator_type.c_str());
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
  nav_state_ = NavigationState::kGoto;
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  plan_path_.clear();
}


void Navigation::SetOverride(const Vector2f& loc, float angle) {
  nav_state_ = NavigationState::kOverride;
  override_target_ = loc;
}

void Navigation::Resume() {
  nav_state_ = NavigationState::kGoto;
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

void Navigation::PruneLatencyQueue() {
  if (command_history_.empty()) return;
  const double update_time = min(t_point_cloud_, t_odometry_);
  static const bool kDebug = false;
  for (size_t i = 0; i < command_history_.size(); ++i) {
    const double t_cmd = command_history_[i].time;
    if (kDebug) {
      printf("Command %d %f\n", int(i), t_cmd - update_time);
    }
    if (t_cmd < update_time - params_.dt) {
      if (kDebug) {
        printf("Erase %d %f %f\n",
            int(i),
            t_cmd - update_time,
            command_history_[i].linear.x());
      }
      command_history_.erase( command_history_.begin() + i);
      --i;
    }
  }
}

void Navigation::UpdateOdometry(const Odom& msg) {
  latest_odom_msg_ = msg;
  t_odometry_ = msg.time;
  PruneLatencyQueue();
  if (!odom_initialized_) {
    starting_loc_ = Vector2f(msg.position.x(), msg.position.y());
    odom_initialized_ = true;
  }
}

void Navigation::UpdateCommandHistory(Twist twist) {
  twist.time += params_.system_latency;
  command_history_.push_back(twist);
  if (false) {
    printf("Push %f %f\n",
          twist.linear.x(),
          command_history_.back().linear.x());
  }
}

void Navigation::ForwardPredict(double t) {
  if (command_history_.empty()) {
    robot_vel_ = Vector2f(0, 0);
    robot_omega_ = 0;
  } else {
    const Twist latest_twist = command_history_.back();
    robot_vel_ = Vector2f(latest_twist.linear.x(), latest_twist.linear.y());
    robot_omega_ = latest_twist.angular.z();
  }
  if (false) {
    for (size_t i = 0; i < command_history_.size(); ++i) {
      const auto& c = command_history_[i];
      printf("%d %f %f\n", int(i), t - c.time, c.linear.x());
    }
    printf("Predict: %f %f\n", t - t_odometry_, t - t_point_cloud_);
  }
  odom_loc_ = Vector2f(latest_odom_msg_.position.x(),
                       latest_odom_msg_.position.y());
  odom_angle_ = 2.0f * atan2f(latest_odom_msg_.orientation.z(),
                              latest_odom_msg_.orientation.w());
  using Eigen::Affine2f;
  using Eigen::Rotation2Df;
  using Eigen::Translation2f;
  Affine2f lidar_tf = Affine2f::Identity();
  for (const Twist& c : command_history_) {
    const double cmd_time = c.time;
    if (cmd_time > t) continue;
    if (cmd_time >= t_odometry_ - params_.dt) {
      const float dt = (t_odometry_ > cmd_time) ?
          min<double>(t_odometry_ - cmd_time, params_.dt) :
          min<double>(t - cmd_time, params_.dt);
      odom_loc_ += dt * (Rotation2Df(odom_angle_) * Vector2f(
          c.linear.x(), c.linear.y()));
      odom_angle_ = AngleMod(odom_angle_ + dt * c.angular.z());
    }
    if (t_point_cloud_ >= cmd_time  - params_.dt) {
      const float dt = (t_point_cloud_ > cmd_time) ?
          min<double>(t_point_cloud_ - cmd_time, params_.dt) :
          min<double>(t - cmd_time, params_.dt);
      lidar_tf =
          Translation2f(-dt * Vector2f(c.linear.x(), c.linear.y())) *
          Rotation2Df(-c.angular.z() * dt) *
          lidar_tf;
    }
  }
  fp_point_cloud_.resize(point_cloud_.size());
  for (size_t i = 0; i < point_cloud_.size(); ++i) {
    fp_point_cloud_[i] = lidar_tf * point_cloud_[i];
  }
}

void Navigation::TrapezoidTest(Vector2f& cmd_vel, float& cmd_angle_vel) {
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
  cmd_vel = {velocity_cmd, 0};
  cmd_angle_vel = 0;
  printf("x: %.3f d:%.3f v: %.3f cmd:%.3f\n", x, FLAGS_test_dist, speed, velocity_cmd);
}

void Navigation::LatencyTest(Vector2f& cmd_vel, float& cmd_angle_vel) {
  static FILE* fid = nullptr;
  if (!FLAGS_test_log_file.empty() && fid == nullptr) {
    fid = fopen(FLAGS_test_log_file.c_str(), "w");
  }
  const float kMaxSpeed = 0.75;
  const float kFrequency = 0.4;

  static double t_start_ = GetMonotonicTime();
  const double t = GetMonotonicTime() - t_start_;
  // float v_current = robot_vel_.x();
  float v_cmd = kMaxSpeed *
      sin(2.0 * M_PI * kFrequency * t);
  cmd_vel = {v_cmd, 0};
  cmd_angle_vel = 0.0;
}

void Navigation::ObstAvTest(Vector2f& cmd_vel, float& cmd_angle_vel) {
  const Vector2f kTarget(4, 0);
  local_target_ = kTarget;
  RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
}

void Navigation::ObstacleTest(Vector2f& cmd_vel, float& cmd_angle_vel) {
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
  cmd_vel = {velocity_cmd, 0};
  cmd_angle_vel = 0;
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

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
  t_point_cloud_ = time;
  PruneLatencyQueue();
}

void Navigation::ObserveImage(cv::Mat image, double time) {
  latest_image_ = image;
  t_image_ = time;
}

vector<int> Navigation::GlobalPlan(const Vector2f& initial,
                                   const Vector2f& end) {
  auto plan = Plan(initial, end);
  std::vector<int> path;
  for (auto& node : plan ) {
    path.push_back(node.id);
  }
  return path;
}

vector<GraphDomain::State> Navigation::Plan(const Vector2f& initial,
                                            const Vector2f& end) {
  vector<GraphDomain::State> path;
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  static const bool kVisualize = true;
  typedef navigation::GraphDomain Domain;
  planning_domain_.ResetDynamicStates();
  const uint64_t start_id = planning_domain_.AddDynamicState(initial);
  const uint64_t goal_id = planning_domain_.AddDynamicState(end);
  Domain::State start = planning_domain_.states[start_id];
  Domain::State goal = planning_domain_.states[goal_id];
  GraphVisualizer graph_viz(kVisualize);
  const bool found_path =
      AStar(start, goal, planning_domain_, &graph_viz, &path);
  if (found_path) {
    CHECK(path.size() > 0);
    Vector2f s1 = plan_path_[0].loc;
    for (size_t i = 1; i < plan_path_.size(); ++i) {
      Vector2f s2 = plan_path_[i].loc;
      s1 = s2;
    }
  } else {
    printf("No path found!\n");
  }
  return path;
}

void Navigation::PlannerTest() {
  if (!loc_initialized_) return;
  Plan(robot_loc_, nav_goal_loc_);
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

bool Navigation::GetCarrot(Vector2f& carrot) {
  const float kSqCarrotDist = Sq(params_.carrot_dist);
  CHECK_GE(plan_path_.size(), 2u);

  if ((plan_path_[0].loc - robot_loc_).squaredNorm() < kSqCarrotDist) {
    // Goal is within the carrot dist.
    carrot = plan_path_[0].loc;
    return true;
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
    carrot = geometry::ProjectPointOntoLineSegment(robot_loc_, v0, v1);
    return true;
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
  i1 = i0 - 1;
  // printf("i0:%d i1:%d\n", i0, i1);
  const Vector2f v0 = plan_path_[i0].loc;
  const Vector2f v1 = plan_path_[i1].loc;
  Vector2f r0, r1;
  #define V2COMP(v) v.x(), v.y()
  // printf("%f,%f %f,%f %f,%f %f\n",
  //     V2COMP(robot_loc_), V2COMP(v0), V2COMP(v1), (v0 - v1).norm());
  const int num_intersections = geometry::CircleLineIntersection<float>(
      robot_loc_, params_.carrot_dist, v0, v1, &r0, &r1);
  if (num_intersections == 0) {
    fprintf(stderr, "Error obtaining intersections:\n v0: (%f %f), v1: (%f %f), robot_loc_: (%f %f) sq_carrot_dist: (%f) closest_dist: (%f)\n",
      v0.x(), v0.y(), v1.x(), v1.y(), robot_loc_.x(), robot_loc_.y(), kSqCarrotDist, closest_dist);
    return false;
  }

  if (num_intersections == 1 || (r0 - v1).squaredNorm() < (r1 - v1).squaredNorm()) {
    carrot = r0;
  } else {
    carrot = r1;
  }
  return true;
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

DEFINE_double(tx, 0.4, "Test obstacle point - X");
DEFINE_double(ty, -0.38, "Test obstacle point - Y");

void Navigation::RunObstacleAvoidance(Vector2f& vel_cmd, float& ang_vel_cmd) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  const bool debug = FLAGS_v > 1;

  // Handling potential carrot overrides from social nav
  Vector2f local_target = local_target_;
  if (nav_state_ == NavigationState::kOverride) {
    local_target = override_target_;
  }

  sampler_->Update(robot_vel_, robot_omega_, local_target, fp_point_cloud_, latest_image_);
  evaluator_->Update(robot_loc_, robot_angle_, robot_vel_, robot_omega_, local_target, fp_point_cloud_, latest_image_);
  auto paths = sampler_->GetSamples(params_.num_options);
  if (debug) {
    printf("%lu options\n", paths.size());
    int i = 0;
    for (auto p : paths) {
      ConstantCurvatureArc arc =
          *reinterpret_cast<ConstantCurvatureArc*>(p.get());
      printf("%3d: %7.5f %7.3f %7.3f\n",
          i++, arc.curvature, arc.length, arc.curvature);
    }
  }
  if (paths.size() == 0) {
    // No options, just stop.
    Halt(vel_cmd, ang_vel_cmd);
    if (debug) printf("No paths found\n");
    return;
  }
  auto best_path = evaluator_->FindBest(paths);
  if (best_path == nullptr) {
    if (debug) printf("No best path found\n");
    // No valid path found!
    Halt(vel_cmd, ang_vel_cmd);
    return;
  }
  ang_vel_cmd = 0;
  vel_cmd = {0, 0};

  float max_map_speed = params_.linear_limits.max_speed;
  planning_domain_.GetClearanceAndSpeedFromLoc(
      robot_loc_, nullptr, &max_map_speed);
  auto linear_limits = params_.linear_limits;
  linear_limits.max_speed = min(max_map_speed, params_.linear_limits.max_speed);
  best_path->GetControls(linear_limits,
                         params_.angular_limits,
                         params_.dt, robot_vel_,
                         robot_omega_,
                         vel_cmd,
                         ang_vel_cmd);
  last_options_ = paths;
  best_option_ = best_path;
}

void Navigation::Halt(Vector2f& cmd_vel, float& angular_vel_cmd) {
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
  cmd_vel = {velocity_cmd, 0};
  //TODO: motion profiling for omega
  angular_vel_cmd = 0;
}

void Navigation::TurnInPlace(Vector2f& cmd_vel, float& cmd_angle_vel) {
  static const bool kDebug = false;
  const float kMaxLinearSpeed = 0.1;
  const float velocity = robot_vel_.x();
  cmd_angle_vel = 0;
  if (fabs(velocity) > kMaxLinearSpeed) {
    Halt(cmd_vel, cmd_angle_vel);
    return;
  }
  float dTheta = 0;
  if (nav_state_ == NavigationState::kGoto) {
    dTheta = atan2(local_target_.y(), local_target_.x());
  } else if (nav_state_ == NavigationState::kOverride) {
    dTheta = atan2(override_target_.y(), override_target_.x());
  } else if (nav_state_ == NavigationState::kTurnInPlace) {
    dTheta = AngleDiff(nav_goal_angle_, robot_angle_);
  }
  if (kDebug) printf("dTheta: %f robot_angle: %f\n", RadToDeg(dTheta), RadToDeg(robot_angle_));

  
  const float s = Sign(dTheta);
  if (robot_omega_ * dTheta < 0.0f) {
    if (kDebug) printf("Wrong way\n");
    const float dv = params_.dt * params_.angular_limits.max_acceleration;
    // Turning the wrong way!
    if (fabs(robot_omega_) < dv) {
      cmd_angle_vel = 0;
    } else {
      cmd_angle_vel = robot_omega_ - Sign(robot_omega_) * dv;
    }
  } else {
    cmd_angle_vel = s * Run1DTimeOptimalControl(
        params_.angular_limits,
        0,
        s * robot_omega_,
        s * dTheta,
        0,
        params_.dt);
  }
  cmd_vel = {0, 0};
}

void Navigation::Pause() {
  nav_state_ = NavigationState::kPaused;
}

void Navigation::SetMaxVel(const float vel) {
  params_.linear_limits.max_speed = vel;
}

void Navigation::SetMaxAccel(const float accel) {
  params_.linear_limits.max_acceleration = accel;
  return;
}

void Navigation::SetMaxDecel(const float decel) {
  params_.linear_limits.max_deceleration = decel;
  return;
}

void Navigation::SetAngAccel(const float accel) {
  params_.angular_limits.max_acceleration = accel;
  return;
}

void Navigation::SetAngVel(const float vel) {
  params_.angular_limits.max_speed = vel;
  return;
}

void Navigation::SetObstacleMargin(const float margin) {
  params_.obstacle_margin = margin;
  return;
}

void Navigation::SetClearanceWeight(const float weight) {
  LinearEvaluator* evaluator =
      dynamic_cast<LinearEvaluator*>(evaluator_.get());
  evaluator->SetClearanceWeight(weight);
  return;
}

void Navigation::SetCarrotDist(const float carrot_dist) {
  params_.carrot_dist = carrot_dist;
  return;
}

Eigen::Vector2f Navigation::GetTarget() {
  return local_target_;
}

Eigen::Vector2f Navigation::GetOverrideTarget() {
  return override_target_;
}

Eigen::Vector2f Navigation::GetVelocity() {
  return robot_vel_;
}

float Navigation::GetAngularVelocity() {
  return robot_omega_;
}

string Navigation::GetNavStatus() {
  switch (nav_state_) {
    case NavigationState::kStopped: {
      return "Stopped";
    } break;
    case NavigationState::kPaused: {
      return "Paused";
    } break;
    case NavigationState::kGoto: {
      return "Goto";
    } break;
    case NavigationState::kOverride: {
      return "Override";
    } break;
    case NavigationState::kTurnInPlace: {
      return "TurnInPlace";
    } break;
    default: {
      return "Unknown";
    } break;
  }
}

vector<Vector2f> Navigation::GetPredictedCloud() {
  return fp_point_cloud_;
}

float Navigation::GetCarrotDist() {
  return params_.carrot_dist;
}

float Navigation::GetObstacleMargin() {
  return params_.obstacle_margin;
}

float Navigation::GetRobotWidth() {
  return params_.robot_width;
}

float Navigation::GetRobotLength() {
  return params_.robot_length;
}

vector<std::shared_ptr<PathRolloutBase>> Navigation::GetLastPathOptions() {
  return last_options_;
}

const cv::Mat& Navigation::GetVisualizationImage() {
  if (params_.evaluator_type == "cost_map") {
    return dynamic_cast<DeepCostMapEvaluator*>(evaluator_.get())->latest_vis_image_;
  } else {
    std::cerr << "No visualization image for linear evaluator" << std::endl;
    exit(1);
  }
}

std::shared_ptr<PathRolloutBase> Navigation::GetOption() {
  return best_option_;
}

vector<GraphDomain::State> Navigation::GetPlanPath() {
  return plan_path_;
}

bool Navigation::Run(const double& time,
                     Vector2f& cmd_vel,
                     float& cmd_angle_vel) {
  const bool kDebug = FLAGS_v > 0;
  if (!initialized_) {
    if (kDebug) printf("Parameters and maps not initialized\n");
    return false;
  }
  if (!odom_initialized_) {
    if (kDebug) printf("Odometry not initialized\n");
    return false;
  }

  ForwardPredict(time + params_.system_latency);
  if (FLAGS_test_toc) {
    TrapezoidTest(cmd_vel, cmd_angle_vel);
    return true;
  } else if (FLAGS_test_obstacle) {
    ObstacleTest(cmd_vel, cmd_angle_vel);
    return true;
  } else if (FLAGS_test_avoidance) {
    ObstAvTest(cmd_vel, cmd_angle_vel);
    return true;
  } else if (FLAGS_test_planner) {
    PlannerTest();
    return true;
  } else if (FLAGS_test_latency) {
    LatencyTest(cmd_vel, cmd_angle_vel);
    return true;
  }

  // Before swithcing states we need to update the local target.
  if (nav_state_ == NavigationState::kGoto ||
      nav_state_ == NavigationState::kOverride) {
    // Recompute global plan as necessary.
    if (!PlanStillValid()) {
      if (kDebug) printf("Replanning\n");
      plan_path_ = Plan(robot_loc_, nav_goal_loc_);
    }
    if (nav_state_ == NavigationState::kGoto) {
      // Get Carrot and check if done
      Vector2f carrot(0, 0);
      bool foundCarrot = GetCarrot(carrot);
      if (!foundCarrot) {
        Halt(cmd_vel, cmd_angle_vel);
        return false;
      }
      // Local Navigation
      local_target_ = Rotation2Df(-robot_angle_) * (carrot - robot_loc_);
    }
  }

  // Switch between navigation states.
  NavigationState prev_state = nav_state_;
  do {
    prev_state = nav_state_;
    if (nav_state_ == NavigationState::kGoto &&
        local_target_.squaredNorm() < Sq(params_.target_dist_tolerance) &&
        robot_vel_.squaredNorm() < Sq(params_.target_dist_tolerance)) {
      nav_state_ = NavigationState::kTurnInPlace;
    } else if (nav_state_ == NavigationState::kTurnInPlace &&
          AngleDist(robot_angle_, nav_goal_angle_) < 
          params_.target_angle_tolerance) {
      nav_state_ = NavigationState::kStopped;
    }
  } while (prev_state != nav_state_);

  
  switch (nav_state_) {
    case NavigationState::kStopped: {
      if (kDebug) printf("\nNav complete\n");
    } break;
    case NavigationState::kPaused: {
      if (kDebug) printf("\nNav paused\n");
    } break;
    case NavigationState::kGoto: {
      if (kDebug) printf("\nNav Goto\n");
    } break;
    case NavigationState::kTurnInPlace: {
      if (kDebug) printf("\nNav TurnInPlace\n");
    } break;
    case NavigationState::kOverride: {
      if (kDebug) printf("\nNav override\n");
    } break;
    default: {
      fprintf(stderr, "ERROR: Unknown nav state %d\n", 
          static_cast<int>(nav_state_));
    }
  }

  if (nav_state_ == NavigationState::kPaused ||
      nav_state_ == NavigationState::kStopped) {
    Halt(cmd_vel, cmd_angle_vel);
    return true;
  } else if (nav_state_ == NavigationState::kGoto ||
      nav_state_ == NavigationState::kOverride) {
    Vector2f local_target(0, 0);
    if (nav_state_ == NavigationState::kGoto) {
      // Local Navigation
      local_target = local_target_;
    } else {
      // Running NavigationState::kOverride .
      local_target = override_target_;
    }
    const float theta = atan2(local_target.y(), local_target.x());
    if (local_target.squaredNorm() > Sq(params_.carrot_dist)) {
      local_target = params_.carrot_dist * local_target.normalized();
    }
    if (!FLAGS_no_local) {
      if (fabs(theta) > params_.local_fov) {
        if (kDebug) printf("TurnInPlace\n");
        TurnInPlace(cmd_vel, cmd_angle_vel);
      } else {
        if (kDebug) printf("ObstAv\n");
        RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
      }
    }
  } else if (nav_state_ == NavigationState::kTurnInPlace) {
    if (kDebug) printf("Reached Goal: TurnInPlace\n");
    TurnInPlace(cmd_vel, cmd_angle_vel);
  }

  return true;
}

}  // namespace navigation
