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
#include <unordered_map>
#include <chrono>
#include <iostream>
#include <fstream>

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
#include "simple_queue.h"

#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "deep_cost_map_evaluator.h"
#include "linear_evaluator.h"
#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/Pose2Df.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

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
using std::unordered_set;
using std::unordered_map;
using std::set;

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
    evaluator_(nullptr),
    costmap_(60, 60, 0.5, -15, -15),
    global_costmap_(200, 200, 0.5, -50, -50),
    intermediate_path_found_(false),
    intermediate_goal_(0, 0){ //parameters are (x/y size in cells, resolution, bottom left x/y origin coordinates)
  sampler_ = std::unique_ptr<PathRolloutSamplerBase>(new AckermannSampler());
}

void Navigation::Initialize(const NavigationParameters& params,
                            const string& map_file) {
  // Initialize status message
  params_ = params;
  int local_costmap_size = 2*static_cast<int>(std::round(params_.local_costmap_radius/params_.local_costmap_resolution));
  costmap_ = costmap_2d::Costmap2D(local_costmap_size, local_costmap_size, params_.local_costmap_resolution, -params.local_costmap_radius, -params.local_costmap_radius);
  int global_costmap_size = 2*static_cast<int>(std::round(params_.global_costmap_radius/params_.global_costmap_resolution));
  global_costmap_ = costmap_2d::Costmap2D(global_costmap_size, global_costmap_size, params_.global_costmap_resolution, -params.global_costmap_radius, -params.global_costmap_radius);
  planning_domain_ = GraphDomain(map_file, &params_);

  LoadVectorMap(map_file);

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

void Navigation::LoadVectorMap(const string& map_file){ //Assume map is given as MAP.navigation.json

  std::string vector_map_file = map_file;

  // Find the position of ".navigation.json"
  size_t found = vector_map_file.find(".navigation.json");
    
  // Replace ".navigation.json" with ".vectormap.json"
  if (found != std::string::npos) {
      vector_map_file.replace(found, std::string(".navigation.json").length(), ".vectormap.json");
  }

  // Output the modified string
  std::cout << "Loading vectormap file: " << vector_map_file << std::endl;

  int x_max = global_costmap_.getSizeInCellsX(); 
  int y_max = global_costmap_.getSizeInCellsY(); 
  global_costmap_.resetMap(0, 0, x_max, y_max);
  global_costmap_obstacles_.clear();

  std::ifstream i(vector_map_file);
  json j;
  i >> j;
  i.close();

  unordered_set<uint64_t> inflation_cells;

  for (const auto& line : j) {
    // Access specific fields in each dictionary
    Vector2f p0(line["p0"]["x"], line["p0"]["y"]);
    Vector2f p1(line["p1"]["x"], line["p1"]["y"]);

    float length = (p0 - p1).norm();
    for (float i = 0; i < length; i += params_.global_costmap_resolution){
      Vector2f costmap_point = p0 + i*(p1 - p0)/length;
      uint32_t unsigned_mx, unsigned_my;
      bool in_map = global_costmap_.worldToMap(costmap_point.x(), costmap_point.y(), unsigned_mx, unsigned_my);
      if(in_map){
        int cell_inflation_size = std::ceil(params_.global_costmap_inflation_size/global_costmap_.getResolution());
        int mx = static_cast<int>(unsigned_mx);
        int my = static_cast<int>(unsigned_my);
        for (int j = -cell_inflation_size; j <= cell_inflation_size; j++){
          for (int k = -cell_inflation_size; k <= cell_inflation_size; k++){
            if((sqrt(pow(j, 2) + pow(k, 2)) <= cell_inflation_size) && (mx + j >= 0) && (mx + j < x_max) 
            && (my + k >= 0) && (my + k < y_max)){
              inflation_cells.insert(global_costmap_.getIndex(mx + j, my + k));
              // global_costmap_.setCost(mx + j, my + k, costmap_2d::LETHAL_OBSTACLE);
            }
          }
        }
      }
    }
  }

  for (const auto& index : inflation_cells) {
    uint32_t mx = 0;
    uint32_t my = 0;
    global_costmap_.indexToCells(index, mx, my);
    global_costmap_.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);

    double wx, wy;
    global_costmap_.mapToWorld(mx, my, wx, wy);
    global_costmap_obstacles_.push_back(Vector2f(wx, wy));
  }

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

void Navigation::ResetNavGoals() {
  nav_state_ = NavigationState::kStopped;
  nav_goal_loc_ = robot_loc_;
  nav_goal_angle_ = robot_angle_;
  local_target_.setZero();
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
  LoadVectorMap(map_path);
  planning_domain_.Load(map_path);
  plan_path_.clear();
  prev_obstacles_.clear();
  costmap_obstacles_.clear();
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

  SimpleQueue<uint32_t, float> intermediate_queue; //<Location stored as <index, cost>
  unordered_map<uint32_t, uint32_t> parent;
  unordered_map<uint32_t, int> cost;

  uint32_t mx = 0, my = 0;
  costmap_.worldToMap(0, 0, mx, my);
  uint32_t robot_index = costmap_.getIndex(mx, my);
  intermediate_queue.Push(robot_index, 0);
  cost[robot_index] = 0;

  global_plan_path_ = path;
  Vector2f intermediate_goal_global = end;
  GetGlobalCarrot(intermediate_goal_global);


  if(path.size() == 0){
    return path;
  }

  Vector2f intermediate_goal_local = intermediate_goal_global - robot_loc_;

  int goal_map_x, goal_map_y;

  costmap_.worldToMapEnforceBounds(intermediate_goal_local.x(), intermediate_goal_local.y(), goal_map_x, goal_map_y);
  uint32_t goal_index = costmap_.getIndex(goal_map_x, goal_map_y);
  double goal_relative_x, goal_relative_y;
  costmap_.mapToWorld(goal_map_x, goal_map_y, goal_relative_x, goal_relative_y);
  intermediate_goal_ = Vector2f(goal_relative_x, goal_relative_y) + robot_loc_;

  unordered_set<uint32_t> visited;

  while(!intermediate_queue.Empty()){
    uint32_t current_index = intermediate_queue.Pop();
    visited.insert(current_index);

    if(current_index == goal_index){
      break;
    }

    costmap_.indexToCells(current_index, mx, my);
    vector<int> neighbors {-1, 0, 1, 0};
    for(size_t i = 0; i < neighbors.size(); i++){
      int new_row = mx + neighbors[i];
      int new_col = my + neighbors[(i+1)%4];
      uint32_t neighbor_index = costmap_.getIndex(new_row, new_col);
      
      if(new_row >= 0 && new_row < static_cast<int>(costmap_.getSizeInCellsX()) && new_col >= 0 
      && new_col < static_cast<int>(costmap_.getSizeInCellsY()) && (costmap_.getCost(new_row, new_col) != costmap_2d::LETHAL_OBSTACLE)
      && (cost.count(neighbor_index) == 0 || cost[current_index] + 1 < cost[neighbor_index])){
        double wx, wy;
        costmap_.mapToWorld(new_row, new_col, wx, wy);
        wx += robot_loc_.x();
        wy += robot_loc_.y();
        uint32_t global_mx, global_my;
        bool in_global_map = global_costmap_.worldToMap(wx, wy, global_mx, global_my);
        if(!in_global_map || global_costmap_.getCost(global_mx, global_my) != costmap_2d::LETHAL_OBSTACLE){
          cost[neighbor_index] = cost[current_index] + 1;
          parent[neighbor_index] = current_index;
          float heuristic_cost = (Vector2f(goal_map_x, goal_map_y) - Vector2f(new_row, new_col)).norm();
          intermediate_queue.Push(neighbor_index, -(cost[neighbor_index] + heuristic_cost));
        }
      }

    }
  }

  vector<Vector2f> intermediate_vector_path;
  vector<GraphDomain::State> intermediate_path;

  if(cost.count(goal_index) != 0){

    uint32_t row, col;
    costmap_.indexToCells(robot_index, row, col);
    double wx, wy;
    costmap_.mapToWorld(row, col, wx, wy);
    Vector2f robot_location(wx, wy);

    uint32_t current_index = goal_index;
    while(parent.count(current_index) > 0){
      costmap_.indexToCells(current_index, row, col);
      costmap_.mapToWorld(row, col, wx, wy);
      intermediate_vector_path.push_back(Vector2f(wx, wy) - robot_location);
      current_index = parent[current_index];
    }
    reverse(intermediate_vector_path.begin(), intermediate_vector_path.end());
    planning_domain_.ResetDynamicStates();
  
    for(size_t i = 0; i < intermediate_vector_path.size(); i++){
      Vector2f map_frame_position = intermediate_vector_path[i] + robot_loc_;
      const uint64_t path_id = planning_domain_.AddDynamicState(map_frame_position);
      intermediate_path.push_back(planning_domain_.states[path_id]);
    }
    reverse(intermediate_path.begin(), intermediate_path.end());
    intermediate_path_found_ = true;
    return intermediate_path;
  }
  else{
    intermediate_path_found_ = false;
    printf("No intermediate planner path found\n");
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

bool Navigation::IntermediatePlanStillValid(){

  if (plan_path_.size() < 2 || !intermediate_path_found_) return false;

  // TODO: Add parameter for when to look for new goal or use different heuristic
  if ((nav_goal_loc_ - plan_path_[0].loc).norm() > sqrt(2 * params_.local_costmap_resolution) / 2 
  && (robot_loc_ - plan_path_[0].loc).norm() < 1){
    return false;
  }

  // TODO: add parameter for distance between intermediate path and global carrot to replan
  Vector2f global_carrot;
  GetGlobalCarrot(global_carrot);
  if((intermediate_goal_ - global_carrot).norm() > 2){
    return false;
  }

  for (size_t i = 0; i < plan_path_.size(); i++){
    uint32_t mx, my;
    Vector2f relative_path_location = plan_path_[i].loc - robot_loc_;
    bool in_map = costmap_.worldToMap(relative_path_location.x(), relative_path_location.y(), mx, my);
    if(in_map && costmap_.getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE){
      return false;
    }
  }
  return true;
}

bool Navigation::GetGlobalCarrot(Vector2f& carrot) {
  return GetCarrot(carrot, true);
}

bool Navigation::GetIntermediateCarrot(Vector2f& carrot) {
  return GetCarrot(carrot, false);
}

bool Navigation::GetCarrot(Vector2f& carrot, bool global) {
  float carrot_dist = params_.intermediate_carrot_dist;
  vector<GraphDomain::State> plan_path = plan_path_;
  if(global){
    plan_path = global_plan_path_;
    carrot_dist = params_.carrot_dist;
  }
  // const float kSqCarrotDist = Sq(params_.carrot_dist);
  const float kSqCarrotDist = Sq(carrot_dist);

  CHECK_GE(plan_path.size(), 2u);

  if ((plan_path[0].loc - robot_loc_).squaredNorm() < kSqCarrotDist) {
    // Goal is within the carrot dist.
    carrot = plan_path[0].loc;
    return true;
  }

  // Find closest line segment in plan to current location
  float closest_dist = FLT_MAX;
  int i0 = 0, i1 = 1;
  for (size_t i = 0; i + 1 < plan_path.size(); ++i) {
    const Vector2f v0 = plan_path[i].loc;
    const Vector2f v1 = plan_path[i + 1].loc;
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
    const Vector2f v0 = plan_path[i0].loc;
    const Vector2f v1 = plan_path[i1].loc;
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
    const Vector2f v1 = plan_path[i - 1].loc;
    if ((v1 - robot_loc_).squaredNorm() > kSqCarrotDist) {
      break;
    }
  }
  i1 = i0 - 1;
  // printf("i0:%d i1:%d\n", i0, i1);
  const Vector2f v0 = plan_path[i0].loc;
  const Vector2f v1 = plan_path[i1].loc;
  Vector2f r0, r1;
  #define V2COMP(v) v.x(), v.y()
  // printf("%f,%f %f,%f %f,%f %f\n",
  //     V2COMP(robot_loc_), V2COMP(v0), V2COMP(v1), (v0 - v1).norm());
  const int num_intersections = geometry::CircleLineIntersection<float>(
      robot_loc_, carrot_dist, v0, v1, &r0, &r1);
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

uint8_t Navigation::GetNavStatusUint8() {
  return static_cast<uint8_t>(nav_state_);
}

vector<Vector2f> Navigation::GetPredictedCloud() {
  return fp_point_cloud_;
}

float Navigation::GetCarrotDist() {
  return params_.intermediate_carrot_dist;
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

vector<GraphDomain::State> Navigation::GetGlobalPath() {
  return global_plan_path_;
}

vector<Eigen::Vector2f> Navigation::GetCostmapObstacles(){
  return costmap_obstacles_;
}

vector<Eigen::Vector2f> Navigation::GetGlobalCostmapObstacles(){
  return global_costmap_obstacles_;
}

Eigen::Vector2f Navigation::GetIntermediateGoal(){
  return intermediate_goal_;
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
  
  int cell_inflation_size = std::ceil(params_.local_costmap_inflation_size/costmap_.getResolution());

  int x_max = costmap_.getSizeInCellsX(); 
  int y_max = costmap_.getSizeInCellsY(); 

  // Reset map to empty from previous iteration
  costmap_.resetMap(0, 0, x_max, y_max);
  costmap_obstacles_.clear();

  unordered_set<uint64_t> inflation_cells;
  unordered_set<uint64_t> obstacle_cells;

  // Map from costmap index to real coordinates relative to robot
  unordered_map<uint64_t, float> index_to_x_coord;
  unordered_map<uint64_t, float> index_to_y_coord;

  uint32_t robot_mx, robot_my;
  costmap_.worldToMap(0, 0, robot_mx, robot_my);
  uint32_t robot_index = costmap_.getIndex(robot_mx, robot_my);
  uint32_t robot_row, robot_col;
  costmap_.indexToCells(robot_index, robot_row, robot_col);
  double robot_wx, robot_wy;
  costmap_.mapToWorld(robot_row, robot_col, robot_wx, robot_wy);
  Vector2f robot_location(robot_wx, robot_wy);

  // Add new points to costmap
  for (size_t i = 0; i < point_cloud_.size(); i++){
    uint32_t unsigned_mx, unsigned_my;
    Vector2f relative_location_map_frame = Rotation2Df(robot_angle_) * point_cloud_[i];
    bool in_map = costmap_.worldToMap(relative_location_map_frame.x(), relative_location_map_frame.y(), unsigned_mx, unsigned_my); 

    //TODO: change max distance based on lidar to base link transformation
    if (in_map && relative_location_map_frame.norm() < (params_.range_max - params_.robot_length) && relative_location_map_frame.norm() > params_.range_min){
      uint32_t index = costmap_.getIndex(unsigned_mx, unsigned_my);
      double wx, wy;
      costmap_.mapToWorld(unsigned_mx, unsigned_my, wx, wy);
      obstacle_cells.insert(index);

      index_to_x_coord[index] = wx - robot_location.x();
      index_to_y_coord[index] = wy - robot_location.y();
    }
  }

  // Point sorting function
  struct PointComparison {
    const costmap_2d::Costmap2D costmap;
    const int robot_mx;
    const int robot_my;


    PointComparison(const costmap_2d::Costmap2D map, const int robot_x, const int robot_y) : costmap(map), robot_mx(robot_x), robot_my(robot_y) {}

    bool operator()(int a, int b) const {
      uint32_t row_a, col_a;
      costmap.indexToCells(a, row_a, col_a);
      uint32_t row_b, col_b;
      costmap.indexToCells(b, row_b, col_b);

      double angle_a = atan2(row_a - robot_mx, col_a);
      double angle_b = atan2(row_b - robot_my, col_b);

      return angle_a < angle_b;
    }
  };
  PointComparison pointComparison(costmap_, robot_mx, robot_my);

  // Define set of points to exclude from new costmap
  set<int, PointComparison> sorted_obstacle_cells(pointComparison);
  unordered_set<uint32_t> empty_cells;

  for (const auto& index : obstacle_cells) {
    sorted_obstacle_cells.insert(index);
  }

  auto iter = sorted_obstacle_cells.begin();

  while (iter != sorted_obstacle_cells.end() && iter != std::prev(sorted_obstacle_cells.end())) {
    costmap_2d::MapLocation point_a;
    costmap_.indexToCells(*iter, point_a.x, point_a.y);
    costmap_2d::MapLocation point_b;
    costmap_.indexToCells(*std::next(iter), point_b.x, point_b.y);
    costmap_2d::MapLocation robot_point;
    robot_point.x = robot_mx;
    robot_point.y = robot_my;
    robot_point = robot_point;
    vector<costmap_2d::MapLocation> polygon = {point_a, point_b, robot_point};
    vector<costmap_2d::MapLocation> fill_cells;
    costmap_.convexFillCells(polygon, fill_cells);

    for (size_t i = 0; i < fill_cells.size(); i++){
      empty_cells.insert(costmap_.getIndex(fill_cells[i].x, fill_cells[i].y));
    }

    iter++;
  }


  // Add old obstacles that are still in map to new costmap
  for (const auto& point : prev_obstacles_) {
    uint32_t unsigned_mx, unsigned_my;

    Vector2f new_relative_point = point + prev_robot_loc_ - robot_loc_;
    bool in_map = costmap_.worldToMap(new_relative_point.x(), new_relative_point.y(), unsigned_mx, unsigned_my);
    uint32_t index = costmap_.getIndex(unsigned_mx, unsigned_my);
    if (in_map && empty_cells.count(index) == 0){
      obstacle_cells.insert(index);
      index_to_x_coord[index] = new_relative_point.x();
      index_to_y_coord[index] = new_relative_point.y();
    }
  }

  for (const auto& index : obstacle_cells) {
    uint32_t unsigned_mx, unsigned_my;
    costmap_.indexToCells(index, unsigned_mx, unsigned_my);
    int mx = static_cast<int>(unsigned_mx);
    int my = static_cast<int>(unsigned_my);
    for (int j = -cell_inflation_size; j <= cell_inflation_size; j++){
      for (int k = -cell_inflation_size; k <= cell_inflation_size; k++){
        if((sqrt(pow(j, 2) + pow(k, 2)) <= cell_inflation_size) && (mx + j >= 0) && (mx + j < x_max) 
        && (my + k >= 0) && (my + k < y_max)){
          inflation_cells.insert(costmap_.getIndex(mx + j, my + k));
        }
      }
    }
  }


  for (const auto& index : inflation_cells) {
    uint32_t mx = 0;
    uint32_t my = 0;
    costmap_.indexToCells(index, mx, my);
    costmap_.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);

    double wx, wy;
    costmap_.mapToWorld(mx, my, wx, wy);
    costmap_obstacles_.push_back(Vector2f(wx, wy) + robot_loc_);
  }

  // Record last seen locations of points
  prev_robot_loc_ = robot_loc_;
  prev_obstacles_.clear();

  for (const auto& pair : index_to_x_coord) {
    uint32_t key = pair.first;
    double x = pair.second;
    double y = index_to_y_coord[key];
    prev_obstacles_.push_back(Vector2f(x, y));
  }


  // Before switching states we need to update the local target.

  if (nav_state_ == NavigationState::kGoto ||
      nav_state_ == NavigationState::kOverride) {
    // Recompute global plan as necessary.
    if (!PlanStillValid() || !IntermediatePlanStillValid()) {
      if (kDebug) printf("Replanning\n");
      plan_path_ = Plan(robot_loc_, nav_goal_loc_);
    }
    if (nav_state_ == NavigationState::kGoto) {
      // Get Carrot and check if done
      Vector2f carrot(0, 0);
      bool foundCarrot = GetIntermediateCarrot(carrot);
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
        robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance)) {
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
    if (local_target.squaredNorm() > params_.intermediate_carrot_dist) {
      local_target = params_.intermediate_carrot_dist * local_target.normalized();
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

  // viz_pub_.publish(local_viz_msg_);
  // viz_pub_.publish(global_viz_msg_);

  return true;
}

}  // namespace navigation
