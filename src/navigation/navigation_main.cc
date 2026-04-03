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
\brief   Main entry point for ROS2 Navigation implementation
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <signal.h>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <limits>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Transform.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// AMRL includes
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/nav_status_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"

// Generated service includes
#include "graph_navigation/srv/graph_nav.hpp"

// Internal includes
#include "config_reader/config_reader.h"
#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "omnidirectional_motion_primitives.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/util/helpers.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "navigation.h"

// System includes
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using namespace std::chrono_literals;

// Command line flags
DEFINE_string(robot_config, "", "Robot config file (required)");
DEFINE_string(maps_dir, "", "Directory containing AMRL maps");
DEFINE_string(map, "UT_Campus", "Name of navigation map file");
DEFINE_string(debug_file, "", "Path to debug log file (.log or .txt). Empty disables logging");

// NavigationParameters
CONFIG_FLOAT(dt, "NavigationParameters.dt");
CONFIG_FLOAT(max_linear_accel, "NavigationParameters.linear_limits.max_acceleration");
CONFIG_FLOAT(max_linear_decel, "NavigationParameters.linear_limits.max_deceleration");
CONFIG_FLOAT(max_linear_speed, "NavigationParameters.linear_limits.max_speed");
CONFIG_FLOAT(max_angular_accel, "NavigationParameters.angular_limits.max_acceleration");
CONFIG_FLOAT(max_angular_decel, "NavigationParameters.angular_limits.max_deceleration");
CONFIG_FLOAT(max_angular_speed, "NavigationParameters.angular_limits.max_speed");
CONFIG_FLOAT(actuation_latency, "NavigationParameters.actuation_latency");
CONFIG_FLOAT(obstacle_margin, "NavigationParameters.obstacle_margin");
CONFIG_INT(num_options, "NavigationParameters.num_options");
CONFIG_FLOAT(robot_width, "NavigationParameters.robot_width");
CONFIG_FLOAT(robot_length, "NavigationParameters.robot_length");
CONFIG_FLOAT(geometric_center_offset_x, "NavigationParameters.geometric_center_offset.x");
CONFIG_FLOAT(geometric_center_offset_y, "NavigationParameters.geometric_center_offset.y");
CONFIG_FLOAT(max_rollout_length, "NavigationParameters.max_rollout_length");
CONFIG_FLOAT(max_lookahead_fpl, "NavigationParameters.max_lookahead_fpl");
CONFIG_FLOAT(clearance_band, "NavigationParameters.clearance_band");
CONFIG_FLOAT(lidar_fov_half_angle, "NavigationParameters.lidar_fov_half_angle");
CONFIG_BOOL(can_traverse_stairs, "NavigationParameters.can_traverse_stairs");
CONFIG_FLOAT(target_dist_tolerance, "NavigationParameters.target_dist_tolerance");
CONFIG_FLOAT(nudge_dist_tolerance, "NavigationParameters.nudge_dist_tolerance");
CONFIG_FLOAT(target_vel_tolerance, "NavigationParameters.target_vel_tolerance");
CONFIG_FLOAT(target_angle_tolerance, "NavigationParameters.target_angle_tolerance");
CONFIG_FLOAT(target_omega_tolerance, "NavigationParameters.target_omega_tolerance");
CONFIG_STRING(evaluator_type, "NavigationParameters.evaluator_type");
CONFIG_FLOAT(carrot_dist, "NavigationParameters.carrot_dist");
CONFIG_STRING(motion_primitives_mode, "NavigationParameters.motion_primitives_mode");
CONFIG_BOOL(do_ang_toc, "NavigationParameters.do_ang_toc");
CONFIG_FLOAT(max_plan_deviation, "NavigationParameters.max_plan_deviation");
CONFIG_FLOAT(laser_height, "NavigationParameters.laser_height");
CONFIG_FLOAT(stuck_meta_override_obstacle_margin, "NavigationParameters.stuck_meta_control.override_obstacle_margin");
CONFIG_FLOAT(stuck_meta_stuck_timeout_sec, "NavigationParameters.stuck_meta_control.stuck_timeout_sec");
CONFIG_FLOAT(stuck_meta_improve_eps, "NavigationParameters.stuck_meta_control.improve_eps");

// Command Mapping
CONFIG_BOOL(apply_custom_cmd_map, "CommandMapping.apply_custom_cmd_map");
CONFIG_FLOAT(cmd_map_x_slope_pos, "CommandMapping.linear_models.x.slope_pos");
CONFIG_FLOAT(cmd_map_x_intercept_pos, "CommandMapping.linear_models.x.intercept_pos");
CONFIG_FLOAT(cmd_map_x_slope_neg, "CommandMapping.linear_models.x.slope_neg");
CONFIG_FLOAT(cmd_map_x_intercept_neg, "CommandMapping.linear_models.x.intercept_neg");
CONFIG_FLOAT(cmd_map_y_slope_pos, "CommandMapping.linear_models.y.slope_pos");
CONFIG_FLOAT(cmd_map_y_intercept_pos, "CommandMapping.linear_models.y.intercept_pos");
CONFIG_FLOAT(cmd_map_y_slope_neg, "CommandMapping.linear_models.y.slope_neg");
CONFIG_FLOAT(cmd_map_y_intercept_neg, "CommandMapping.linear_models.y.intercept_neg");
CONFIG_FLOAT(cmd_map_r_slope_pos, "CommandMapping.linear_models.r.slope_pos");
CONFIG_FLOAT(cmd_map_r_intercept_pos, "CommandMapping.linear_models.r.intercept_pos");
CONFIG_FLOAT(cmd_map_r_slope_neg, "CommandMapping.linear_models.r.slope_neg");
CONFIG_FLOAT(cmd_map_r_intercept_neg, "CommandMapping.linear_models.r.intercept_neg");

// ROS Topics
CONFIG_STRINGLIST(laser_topics, "ROSTopics.laser_topics");
CONFIG_STRING(odom_topic, "ROSTopics.odom_topic");
CONFIG_STRING(localization_topic, "ROSTopics.localization_topic");
CONFIG_STRING(ackermann_drive_topic, "ROSTopics.ackermann_drive_topic");
CONFIG_STRING(nav_status_topic, "ROSTopics.nav_status_topic");
CONFIG_STRING(visualization_topic, "ROSTopics.visualization_topic");
CONFIG_STRING(visualization_local_topic, "ROSTopics.visualization_local_topic");
CONFIG_STRING(fp_pcl_topic, "ROSTopics.fp_pcl_topic");
CONFIG_STRING(path_topic, "ROSTopics.path_topic");
CONFIG_STRING(carrot_topic, "ROSTopics.carrot_topic");
CONFIG_STRING(goto_topic, "ROSTopics.goto_topic");
CONFIG_STRING(goto_amrl_topic, "ROSTopics.goto_amrl_topic");
CONFIG_STRING(reset_nav_goals_topic, "ROSTopics.reset_nav_goals_topic");
CONFIG_STRING(halt_topic, "ROSTopics.halt_topic");
CONFIG_STRING(twist_drive_topic, "ROSTopics.twist_drive_topic");
CONFIG_STRING(current_map_topic, "ROSTopics.current_map_topic");
CONFIG_STRING(robot_geometry_topic, "ROSTopics.robot_geometry_topic");
CONFIG_STRING(dynamic_nav_graph_topic, "ROSTopics.dynamic_nav_graph_topic");

// ROS Frames
CONFIG_STRING(map_frame, "ROSFrames.map_frame");
CONFIG_STRING(robot_frame, "ROSFrames.robot_frame");

class NavigationNode : public rclcpp::Node, public std::enable_shared_from_this<NavigationNode> {
   public:
    NavigationNode()
        : Node("navigation"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          run_(true),
          received_odom_(false),
          received_laser_(false) {
        // Initialize maps directory
        if (FLAGS_maps_dir.empty()) {
            try {
                FLAGS_maps_dir = ament_index_cpp::get_package_share_directory("amrl_maps");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Could not find amrl_maps package: %s", e.what());
                FLAGS_maps_dir = "./maps";
            }
        }

        // Initialize configuration
        config_reader::ConfigReader reader({FLAGS_robot_config});
        LoadConfig(&params_);

        // Save baseline (original Lua config). We will always reset to THIS after reaching the active goal.
        config_params_ = params_;
        
        // Initialize dynamic graph tracking
        has_dynamic_graph_ = false;

        // Load map
        std::string map_path = navigation::GetMapPath(FLAGS_maps_dir, FLAGS_map);
        current_map_name_ = FLAGS_map;
        current_map_path_ = map_path;
        if (!FileExists(map_path)) {
            RCLCPP_ERROR(this->get_logger(), "Could not find navigation map file at %s", map_path.c_str());
            throw std::runtime_error("Map file not found");
        }

        // Initialize navigation
        navigation_.Initialize(params_, map_path);

        // Initialize visualization messages
        local_viz_msg_ = visualization::NewVisualizationMessage(CONFIG_robot_frame, "navigation_local");
        global_viz_msg_ = visualization::NewVisualizationMessage(CONFIG_map_frame, "navigation_global");

        // Create publishers
        ackermann_drive_pub_ =
            this->create_publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>(CONFIG_ackermann_drive_topic, 1);
        twist_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(CONFIG_twist_drive_topic, 1);
        status_pub_ = this->create_publisher<amrl_msgs::msg::NavStatusMsg>(CONFIG_nav_status_topic, 1);
        viz_pub_ = this->create_publisher<amrl_msgs::msg::VisualizationMsg>(CONFIG_visualization_topic, 10);
        viz_local_pub_ = this->create_publisher<amrl_msgs::msg::VisualizationMsg>(CONFIG_visualization_local_topic, 10);
        fp_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(CONFIG_fp_pcl_topic, 1);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(CONFIG_path_topic, 1);
        carrot_pub_ = this->create_publisher<nav_msgs::msg::Path>(CONFIG_carrot_topic, 1);

        // Create service
        nav_service_ = this->create_service<graph_navigation::srv::GraphNav>(
            "GraphNav",
            std::bind(&NavigationNode::PlanServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            CONFIG_odom_topic, 1, std::bind(&NavigationNode::OdometryCallback, this, std::placeholders::_1));
        localization_sub_ = this->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            CONFIG_localization_topic, 1,
            std::bind(&NavigationNode::LocalizationCallback, this, std::placeholders::_1));
        for (size_t i = 0; i < CONFIG_laser_topics.size(); ++i) {
            auto laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                CONFIG_laser_topics[i], 1, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    LaserCallback(msg, CONFIG_laser_topics[i]);
                });
            laser_subs_.push_back(laser_sub);
        }
        goto_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            CONFIG_goto_topic, 1, std::bind(&NavigationNode::GoToCallback, this, std::placeholders::_1));
        goto_amrl_sub_ = this->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            CONFIG_goto_amrl_topic, 1, std::bind(&NavigationNode::GoToCallbackAMRL, this, std::placeholders::_1));
        reset_nav_goals_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            CONFIG_reset_nav_goals_topic, 1,
            std::bind(&NavigationNode::ResetNavGoalsCallback, this, std::placeholders::_1));
        halt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            CONFIG_halt_topic, 1, std::bind(&NavigationNode::HaltCallback, this, std::placeholders::_1));
        current_map_sub_ = this->create_subscription<std_msgs::msg::String>(
            CONFIG_current_map_topic, 1, std::bind(&NavigationNode::CurrentMapCallback, this, std::placeholders::_1));
        robot_geom_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            CONFIG_robot_geometry_topic, 1, std::bind(&NavigationNode::RobotGeomCallback, this, std::placeholders::_1));
        
        auto graph_qos = rclcpp::QoS(1).reliable().transient_local();
        dynamic_nav_graph_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            CONFIG_dynamic_nav_graph_topic, graph_qos, std::bind(&NavigationNode::DynamicNavGraphCallback, this, std::placeholders::_1));

        // Create timer for main loop (respects use_sim_time parameter)
        timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::duration<double>(params_.dt),
                                      std::bind(&NavigationNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
    }

    ~NavigationNode() { run_ = false; }

   private:
    // ROS2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Publishers
    rclcpp::Publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>::SharedPtr ackermann_drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_drive_pub_;
    rclcpp::Publisher<amrl_msgs::msg::NavStatusMsg>::SharedPtr status_pub_;
    rclcpp::Publisher<amrl_msgs::msg::VisualizationMsg>::SharedPtr viz_pub_;
    rclcpp::Publisher<amrl_msgs::msg::VisualizationMsg>::SharedPtr viz_local_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr fp_pcl_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr carrot_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goto_sub_;
    rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr goto_amrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_nav_goals_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_map_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr robot_geom_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_nav_graph_sub_;

    // Service
    rclcpp::Service<graph_navigation::srv::GraphNav>::SharedPtr nav_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Navigation components
    navigation::Navigation navigation_;
    navigation::NavigationParameters params_;

    // Snapshot of the ORIGINAL config (Lua) params. Used for meta-reset when goal completes.
    navigation::NavigationParameters config_params_;

    // State variables
    bool run_;
    bool received_odom_;
    bool received_laser_;
    navigation::Odom odom_;
    std::vector<Eigen::Vector2f> point_cloud_;
    std::string current_map_name_;
    std::string current_map_path_;
    
    // Dynamic navigation graph tracking
    visualization_msgs::msg::MarkerArray last_dynamic_graph_;
    bool has_dynamic_graph_;

    // --- "Stuck" meta-controller state ---
    struct StuckMetaState {
        bool initialized = false;
        bool override_active = false;

        // Final fallback latch: we already retargeted the goal to best_loc_map.
        bool final_goal_applied = false;

        // For sim-time jumps/backwards detection
        double last_time = 0.0;

        // Lowest distance-to-goal observed since we started tracking (for this goal/meta reset).
        float best_dist = std::numeric_limits<float>::infinity();

        // Robot pose (MAP frame) at which best_dist was achieved.
        Eigen::Vector2f best_loc_map = Eigen::Vector2f(0.0f, 0.0f);

        // time when best_dist was last improved
        double best_time = 0.0;
    };

    StuckMetaState stuck_meta_;

    // Track nav_state across ticks so we detect transitions that happen BETWEEN callbacks too.
    navigation::NavigationState last_nav_state_ = navigation::NavigationState::kStopped;

    // Pending geometry update buffer
    struct PendingGeomUpdate {
        float width = 0.0f;
        float length = 0.0f;
        float offset_x = 0.0f;
        float offset_y = 0.0f;
        float margin = 0.0f;
        bool do_ang_toc = false;
    };

    std::mutex geom_update_mutex_;
    bool geom_update_pending_ = false;
    PendingGeomUpdate pending_geom_update_;

    // Visualization
    amrl_msgs::msg::VisualizationMsg local_viz_msg_;
    amrl_msgs::msg::VisualizationMsg global_viz_msg_;

    // Laser processing
    struct LaserCache {
        double time = 0.0;
        float dtheta = 0.0f;
        float angle_min = 0.0f;
        std::vector<Eigen::Vector3f> rays;
        Eigen::Affine3f frame_tf = Eigen::Affine3f::Identity();
    };
    std::unordered_map<std::string, LaserCache> laser_caches_;

    // Callback functions
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        received_odom_ = true;
        odom_ = OdomHandler(*msg);
        navigation_.UpdateOdometry(odom_);
    }

    void LocalizationCallback(const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
        navigation_.UpdateLocation(Eigen::Vector2f(msg->pose.x, msg->pose.y), msg->pose.theta);
    }

    void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic) {
        if (!received_laser_) {
            point_cloud_.clear();
            received_laser_ = true;
        }
        LaserHandler(*msg, topic);
        navigation_.ObservePointCloud(point_cloud_, rclcpp::Time(msg->header.stamp).seconds());
    }

    void GoToCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const Eigen::Vector2f loc(msg->pose.position.x, msg->pose.position.y);
        const float angle = 2.0 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "Goal: (%f,%f) %f°", loc.x(), loc.y(), angle);
        navigation_.SetNavGoal(loc, angle);
    }

    void GoToCallbackAMRL(const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
        const Eigen::Vector2f loc(msg->pose.x, msg->pose.y);
        RCLCPP_INFO(this->get_logger(), "Goal: (%f,%f) %f°", loc.x(), loc.y(), msg->pose.theta);
        navigation_.SetNavGoal(loc, msg->pose.theta);
    }

    void ResetNavGoalsCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Resetting all nav goals");
        navigation_.ResetNavGoals();
    }

    void HaltCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        navigation_.nav_state_ = navigation::NavigationState::kStopped;
    }

    void CurrentMapCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (current_map_name_ != msg->data) {
            RCLCPP_INFO(this->get_logger(), "Current map changed to: %s", msg->data.c_str());
            current_map_name_ = msg->data;
            current_map_path_ = navigation::GetMapPath(FLAGS_maps_dir, msg->data);
            navigation_.UpdateMap(current_map_path_);
            ReapplyDynamicGraphIfActive();
        }
    }

    void DynamicNavGraphCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        last_dynamic_graph_ = *msg;
        has_dynamic_graph_ = true;
        navigation_.UpdateDynamicNavGraph(*msg);
    }
    
    void ReapplyDynamicGraphIfActive() {
        if (has_dynamic_graph_) {
            navigation_.UpdateDynamicNavGraph(last_dynamic_graph_);
        }
    }

    void RobotGeomCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
            RCLCPP_WARN(this->get_logger(),
                        "Robot geometry update ignored: expected 6 floats "
                        "[width,length,offset_x,offset_y,margin,do_ang_toc], got %zu",
                        msg->data.size());
            return;
        }

        // Sentinel value for "do not modify" parameters
        constexpr float kNoModify = -99.0f;

        // Apply "do not change" sentinel values (kNoModify) - use current values as defaults
        const float width = (msg->data[0] == kNoModify) ? navigation_.params_.robot_width : msg->data[0];
        const float length = (msg->data[1] == kNoModify) ? navigation_.params_.robot_length : msg->data[1];
        const float offset_x =
            (msg->data[2] == kNoModify) ? navigation_.params_.geometric_center_offset.x : msg->data[2];
        const float offset_y =
            (msg->data[3] == kNoModify) ? navigation_.params_.geometric_center_offset.y : msg->data[3];
        const float margin = (msg->data[4] == kNoModify) ? navigation_.params_.obstacle_margin : msg->data[4];
        const float do_ang_toc_raw = msg->data[5];
        const bool do_ang_toc =
            (do_ang_toc_raw == kNoModify) ? navigation_.params_.do_ang_toc : (do_ang_toc_raw != 0.0f);

        PendingGeomUpdate u;
        u.width = width;
        u.length = length;
        u.offset_x = offset_x;
        u.offset_y = offset_y;
        u.margin = margin;
        u.do_ang_toc = do_ang_toc;

        {
            std::lock_guard<std::mutex> lock(geom_update_mutex_);
            pending_geom_update_ = u;  // last-write-wins
            geom_update_pending_ = true;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Queued robot geometry update: width=%.3f length=%.3f offset=(%.3f,%.3f) margin=%.3f do_ang_toc=%s",
                    u.width, u.length, u.offset_x, u.offset_y, u.margin, u.do_ang_toc ? "true" : "false");
    }

    void PlanServiceCallback(const std::shared_ptr<graph_navigation::srv::GraphNav::Request> request,
                             std::shared_ptr<graph_navigation::srv::GraphNav::Response> response) {
        const Eigen::Vector2f start(request->start.x, request->start.y);
        const Eigen::Vector2f end(request->end.x, request->end.y);
        auto plan_states = navigation_.Plan(start, end);
        std::vector<int> plan;
        for (auto& node : plan_states) {
            plan.push_back(node.id);
        }
        response->plan = plan;
    }

    void ApplyPendingGeometryUpdate() {
        // If meta override is active, ignore external geometry updates until reset-on-stop.
        if (stuck_meta_.override_active) {
            std::lock_guard<std::mutex> lock(geom_update_mutex_);
            geom_update_pending_ = false;
            return;
        }

        PendingGeomUpdate u;
        {
            std::lock_guard<std::mutex> lock(geom_update_mutex_);
            if (!geom_update_pending_) return;
            u = pending_geom_update_;
            geom_update_pending_ = false;
        }

        // Keep the node's params_ consistent too (future-proof; avoids mixed usage).
        params_.robot_width = u.width;
        params_.robot_length = u.length;
        params_.geometric_center_offset.x = u.offset_x;
        params_.geometric_center_offset.y = u.offset_y;
        params_.obstacle_margin = u.margin;
        params_.do_ang_toc = u.do_ang_toc;

        // Apply to Navigation (propagates to sampler/evaluator, clears stale plan/samples).
        navigation_.UpdateGeometryParams(u.width, u.length, u.offset_x, u.offset_y, u.margin, u.do_ang_toc);

        // IMPORTANT: Re-load the current map so the global planning domain can rebuild any geometry-dependent caches.
        if (!current_map_path_.empty()) {
            navigation_.UpdateMap(current_map_path_);
            ReapplyDynamicGraphIfActive();
        }
    }

    void ApplyGeometryUpdateImmediate(float width, float length, float offset_x, float offset_y, float margin,
                                      bool do_ang_toc) {
        // Determine what changed BEFORE mutating params_.
        const bool geom_changed = (params_.robot_width != width) || (params_.robot_length != length) ||
                                  (params_.geometric_center_offset.x != offset_x) ||
                                  (params_.geometric_center_offset.y != offset_y) ||
                                  (params_.obstacle_margin != margin);

        const bool toc_changed = (params_.do_ang_toc != do_ang_toc);

        if (!geom_changed && !toc_changed) return;

        // Keep node params_ consistent.
        params_.robot_width = width;
        params_.robot_length = length;
        params_.geometric_center_offset.x = offset_x;
        params_.geometric_center_offset.y = offset_y;
        params_.obstacle_margin = margin;
        params_.do_ang_toc = do_ang_toc;

        // Apply into Navigation (propagates to sampler/evaluator, clears stale local planner artifacts).
        navigation_.UpdateGeometryParams(width, length, offset_x, offset_y, margin, do_ang_toc);

        // Only reload the map if geometry (footprint / margin) changed. (do_ang_toc alone does NOT require reload)
        if (geom_changed && !current_map_path_.empty()) {
            navigation_.UpdateMap(current_map_path_);
            ReapplyDynamicGraphIfActive();
        }
    }

    void ResetStuckMetaState() {
        stuck_meta_ = StuckMetaState();  // resets initialized/best/override/timestamps
    }

    void RestoreOriginalConfigGeometry() {
        // Drop any queued external geometry update so it doesn't re-apply AFTER we restore baseline.
        {
            std::lock_guard<std::mutex> lock(geom_update_mutex_);
            geom_update_pending_ = false;
        }

        ApplyGeometryUpdateImmediate(config_params_.robot_width, config_params_.robot_length,
                                     config_params_.geometric_center_offset.x, config_params_.geometric_center_offset.y,
                                     config_params_.obstacle_margin, config_params_.do_ang_toc);
    }

    void MetaGeometryController(double now_sec, bool nav_succeeded) {
        const navigation::NavigationState cur_state = navigation_.nav_state_;
        const navigation::NavigationState prev_state = last_nav_state_;

        // --- Reset meta + restore baseline geometry when we ENTER Stopped. ---
        if (cur_state == navigation::NavigationState::kStopped) {
            if (prev_state != navigation::NavigationState::kStopped) {
                ResetStuckMetaState();
                RestoreOriginalConfigGeometry();
            }
            return;
        }

        // If we START navigating from Stopped, reset meta tracking.
        if (prev_state == navigation::NavigationState::kStopped) {
            ResetStuckMetaState();
        }

        // Only evaluate stuck logic if we produced a navigation output and are actively navigating.
        if (!nav_succeeded || cur_state != navigation::NavigationState::kGoto) return;

        // If we've already declared the original goal unreachable and retargeted,
        // do nothing else until we enter kStopped (reset happens there).
        if (stuck_meta_.final_goal_applied) {
            stuck_meta_.last_time = now_sec;
            return;
        }

        constexpr double kTimeBackwardsEps = 1e-3;
        constexpr float kMarginEps = 1e-4f;

        const float override_margin = params_.stuck_meta_override_obstacle_margin;
        const double timeout_sec = params_.stuck_meta_stuck_timeout_sec;
        const float improve_eps = params_.stuck_meta_improve_eps;
        const char* toc_str = params_.do_ang_toc ? "true" : "false";

        const Eigen::Vector2f robot_fp = navigation_.robot_loc_fp_;  // MAP frame
        const Eigen::Vector2f goal_map = navigation_.nav_goal_loc_;  // MAP frame
        const float dist_to_goal = (goal_map - robot_fp).norm();
        if (!std::isfinite(dist_to_goal)) return;

        auto DropPendingGeomUpdate = [&]() {
            std::lock_guard<std::mutex> lock(geom_update_mutex_);
            geom_update_pending_ = false;
        };

        auto ApplyOverride = [&]() {
            stuck_meta_.override_active = true;
            ApplyGeometryUpdateImmediate(params_.robot_width, params_.robot_length, params_.geometric_center_offset.x,
                                         params_.geometric_center_offset.y, override_margin, params_.do_ang_toc);
        };

        auto EnsureBestInitialized = [&]() {
            if (!stuck_meta_.initialized) {
                stuck_meta_.initialized = true;
                stuck_meta_.best_dist = dist_to_goal;
                stuck_meta_.best_loc_map = robot_fp;  // MAP frame
            }
        };

        // --- Near-goal: goal is effectively inside inflated footprint -> immediately relax margin once. ---
        if (!stuck_meta_.override_active) {
            const float max_body_dim =
                (params_.robot_width > params_.robot_length) ? params_.robot_width : params_.robot_length;
            const float trigger_dist = max_body_dim + 2.0f * params_.obstacle_margin;
            if (dist_to_goal < trigger_dist) {
                ApplyOverride();
                EnsureBestInitialized();
                stuck_meta_.best_time = now_sec;  // fresh timeout window (prevents instant "unreachable")
                stuck_meta_.last_time = now_sec;

                RCLCPP_WARN(this->get_logger(),
                            "[meta] Goal near; applying override: obstacle_margin=%.2f, do_ang_toc=%s", override_margin,
                            toc_str);
                return;
            }
        }

        // Pause the stuck timer while not in obstacle-avoidance mode BEFORE we apply the override.
        // Once override_active is true, we intentionally keep counting time even if OA toggles off, so stage-2 can
        // trigger.
        if (!navigation_.in_obstacle_avoidance_mode_ && !stuck_meta_.override_active) {
            if (now_sec + kTimeBackwardsEps < stuck_meta_.last_time) {
                ResetStuckMetaState();
            } else if (stuck_meta_.initialized && stuck_meta_.last_time > 0.0) {
                const double pause_dt = now_sec - stuck_meta_.last_time;
                if (pause_dt > 0.0) stuck_meta_.best_time += pause_dt;
            }
            stuck_meta_.last_time = now_sec;
            return;
        }

        // Handle time going backwards (sim time reset/jump).
        if (now_sec + kTimeBackwardsEps < stuck_meta_.last_time) {
            ResetStuckMetaState();
        }
        stuck_meta_.last_time = now_sec;

        // --- "lowest it ever was" rule ---
        // Update best-ever distance if uninitialized or improved by >= eps.
        if (!stuck_meta_.initialized || dist_to_goal < stuck_meta_.best_dist - improve_eps) {
            stuck_meta_.initialized = true;
            stuck_meta_.best_dist = dist_to_goal;
            stuck_meta_.best_loc_map = robot_fp;
            stuck_meta_.best_time = now_sec;
            return;
        }

        const double wait_time = now_sec - stuck_meta_.best_time;
        if (wait_time < timeout_sec) {
            RCLCPP_INFO(this->get_logger(),
                        "[meta] No new best goal distance for %.2fs / %.1fs (best=%.3f, current=%.3f)", wait_time,
                        timeout_sec, stuck_meta_.best_dist, dist_to_goal);
            return;
        }

        // --- Timed out without improvement: stage-1 override OR stage-2 "unreachable goal" fallback. ---
        const bool at_override_margin = (std::fabs(params_.obstacle_margin - override_margin) <= kMarginEps);
        if (at_override_margin) {
            stuck_meta_.override_active = true;  // keep suppressing external geom updates until stop
            stuck_meta_.final_goal_applied = true;

            Eigen::Vector2f fallback_goal = stuck_meta_.best_loc_map;
            if (!std::isfinite(fallback_goal.x()) || !std::isfinite(fallback_goal.y())) {
                fallback_goal = robot_fp;
            }

            const float keep_goal_angle = navigation_.nav_goal_angle_;
            const Eigen::Vector2f original_goal = navigation_.nav_goal_loc_;

            DropPendingGeomUpdate();
            navigation_.SetNavGoal(fallback_goal, keep_goal_angle);

            RCLCPP_ERROR(this->get_logger(),
                         "[meta] Goal unreachable after override (margin=%.2f). Retargeting to closest point: "
                         "best_dist=%.3f best_loc=(%.3f, %.3f) original_goal=(%.3f, %.3f)",
                         override_margin, stuck_meta_.best_dist, fallback_goal.x(), fallback_goal.y(),
                         original_goal.x(), original_goal.y());
            return;
        }

        // Stage-1: apply reduced obstacle margin and give it a fresh timeout window.
        ApplyOverride();
        stuck_meta_.best_time = now_sec;

        RCLCPP_WARN(this->get_logger(),
                    "[meta] Stuck: no NEW best goal distance for %.1fs (best=%.3f, current=%.3f). "
                    "Applying override: obstacle_margin=%.2f, do_ang_toc=%s",
                    timeout_sec, stuck_meta_.best_dist, dist_to_goal, override_margin, toc_str);
    }

    void TimerCallback() {
        if (!run_) return;

        ApplyPendingGeometryUpdate();

        // const auto timer_start = std::chrono::steady_clock::now();

        // Clear visualization messages
        visualization::ClearVisualizationMsg(local_viz_msg_);
        visualization::ClearVisualizationMsg(global_viz_msg_);
        received_laser_ = false;  // Reset for new tick so next laser clears/rebuilds the point cloud from latest data

        Eigen::Vector2f cmd_vel(0, 0);
        float cmd_angle_vel(0);
        const double cmd_plan_start_time = this->get_clock()->now().seconds();
        bool nav_succeeded = navigation_.Run(cmd_plan_start_time, cmd_vel, cmd_angle_vel);

        // Meta-controller (stuck detection + param override/reset).
        MetaGeometryController(cmd_plan_start_time, nav_succeeded);

        // Update last-nav-state AFTER meta logic so transitions are detected correctly next tick.
        last_nav_state_ = navigation_.nav_state_;

        PublishNavStatus();

        // Always publish visualizations (they may be empty/cleared if navigation failed)
        PublishForwardPredictedPCL(navigation_.fp_point_cloud_);
        DrawRobot();
        if (static_cast<uint8_t>(navigation_.nav_state_) !=
            static_cast<uint8_t>(navigation::NavigationState::kStopped)) {
            DrawTarget();
            if (navigation_.in_obstacle_avoidance_mode_) {
                DrawYawTarget();
            }
            DrawPathOptions();
        }
        PublishPath();
        local_viz_msg_.header.stamp = this->get_clock()->now();
        global_viz_msg_.header.stamp = this->get_clock()->now();
        viz_local_pub_->publish(local_viz_msg_);
        viz_pub_->publish(global_viz_msg_);

        if (nav_succeeded) {
            // Send commands only if navigation succeeded
            SendCommand(cmd_vel, cmd_angle_vel, cmd_plan_start_time);
        }

        // const auto timer_end = std::chrono::steady_clock::now();
        // const double total_ms = std::chrono::duration<double, std::milli>(timer_end - timer_start).count();
        // std::string timer_msg = std::string("[") + std::to_string(static_cast<int>(navigation_.nav_state_)) +
        //                         "] TimerCallback took " + std::to_string(total_ms) + " ms";
        // navigation::navigation_debug::DebugLog(timer_msg);
    }

    // Helper functions
    navigation::Odom OdomHandler(const nav_msgs::msg::Odometry& msg) {
        navigation::Odom odom;
        odom.time = rclcpp::Time(msg.header.stamp).seconds();
        odom.orientation = {
            static_cast<float>(msg.pose.pose.orientation.w), static_cast<float>(msg.pose.pose.orientation.x),
            static_cast<float>(msg.pose.pose.orientation.y), static_cast<float>(msg.pose.pose.orientation.z)};
        odom.position = {static_cast<float>(msg.pose.pose.position.x), static_cast<float>(msg.pose.pose.position.y),
                         static_cast<float>(msg.pose.pose.position.z)};
        return odom;
    }

    void LaserHandler(const sensor_msgs::msg::LaserScan& msg, const std::string& topic) {
        auto& cache = laser_caches_[topic];

        if (cache.dtheta != msg.angle_increment || cache.angle_min != msg.angle_min ||
            cache.rays.size() != msg.ranges.size()) {
            cache.dtheta = msg.angle_increment;
            cache.angle_min = msg.angle_min;
            cache.rays.resize(msg.ranges.size());
            for (size_t i = 0; i < cache.rays.size(); ++i) {
                const float a = cache.angle_min + static_cast<float>(i) * cache.dtheta;
                cache.rays[i] = Eigen::Vector3f(cos(a), sin(a), 0.0f);
            }
        }

        // Lookup TF transform
        RetrieveTransform(msg.header, cache.frame_tf);

        size_t start_idx = point_cloud_.size();
        point_cloud_.resize(start_idx + cache.rays.size());

        for (size_t i = 0; i < cache.rays.size(); ++i) {
            const float r =
                ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max) ? msg.ranges[i] : msg.range_max);
            point_cloud_[start_idx + i] = (cache.frame_tf * (r * cache.rays[i])).head<2>();
        }
    }

    void RetrieveTransform(const std_msgs::msg::Header& msg, Eigen::Affine3f& frame_tf) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform(CONFIG_robot_frame, msg.frame_id, tf2::TimePointZero);

            tf2::Transform tf_transform;
            tf2::fromMsg(transform_stamped.transform, tf_transform);

            // Convert to Eigen
            auto translation = tf_transform.getOrigin();
            auto rotation = tf_transform.getRotation();

            frame_tf = Eigen::Translation3f(translation.x(), translation.y(), translation.z()) *
                       Eigen::Quaternionf(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to retrieve transform: %s", ex.what());
            // Use identity transform as fallback
            frame_tf = Eigen::Affine3f::Identity();
        }
    }

    void PublishNavStatus() {
        auto status = std::make_unique<amrl_msgs::msg::NavStatusMsg>();
        status->header.stamp = this->get_clock()->now();
        status->status = static_cast<uint8_t>(navigation_.nav_state_);
        status_pub_->publish(std::move(status));
    }

    void SendCommand(const Eigen::Vector2f& vel, float ang_vel, double cmd_plan_start_time) {
        // Determine commanded values first to avoid use-after-move on unique_ptr
        double cmd_lin_x = 0.0;
        double cmd_lin_y = 0.0;
        double cmd_ang_z = 0.0;
        cmd_lin_x = vel.x();
        cmd_lin_y = vel.y();
        cmd_ang_z = ang_vel;

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = cmd_lin_x;
        twist_msg->linear.y = cmd_lin_y;
        twist_msg->angular.z = cmd_ang_z;
        twist_drive_pub_->publish(std::move(twist_msg));

        // Convert to Ackermann if needed
        auto ackermann_msg = std::make_unique<amrl_msgs::msg::AckermannCurvatureDriveMsg>();
        ackermann_msg->header.stamp = this->get_clock()->now();
        ackermann_msg->velocity = cmd_lin_x;
        if (fabs(ackermann_msg->velocity) < 1e-6) {
            ackermann_msg->curvature = 0;
        } else {
            ackermann_msg->curvature = cmd_ang_z / ackermann_msg->velocity;
        }
        ackermann_drive_pub_->publish(std::move(ackermann_msg));

        // Update command history
        navigation::Twist twist;
        twist.cmd_plan_start_time = cmd_plan_start_time;
        twist.cmd_exec_start_time = cmd_plan_start_time + navigation_.params_.actuation_latency;
        twist.linear = {static_cast<float>(cmd_lin_x), static_cast<float>(cmd_lin_y), 0.0f};
        twist.angular = {0.0f, 0.0f, static_cast<float>(cmd_ang_z)};
        navigation_.UpdateCommandHistory(twist);
    }

    void PublishForwardPredictedPCL(const std::vector<Eigen::Vector2f>& pcl) {
        auto fp_pcl_msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        fp_pcl_msg->points.resize(pcl.size());
        for (size_t i = 0; i < pcl.size(); ++i) {
            fp_pcl_msg->points[i].x = pcl[i].x();
            fp_pcl_msg->points[i].y = pcl[i].y();
            fp_pcl_msg->points[i].z = navigation_.params_.laser_height;
        }
        fp_pcl_msg->header.stamp = this->get_clock()->now();
        fp_pcl_msg->header.frame_id = CONFIG_robot_frame;
        fp_pcl_pub_->publish(std::move(fp_pcl_msg));
    }

    void PublishPath() {
        const auto path = navigation_.plan_path_;
        if (path.size() >= 2) {
            // Publish full planned path as nav_msgs::Path
            auto path_msg = std::make_unique<nav_msgs::msg::Path>();
            path_msg->header.stamp = this->get_clock()->now();
            path_msg->header.frame_id = CONFIG_map_frame;

            // Convert each waypoint to a pose in the path
            for (size_t i = 0; i < path.size(); i++) {
                geometry_msgs::msg::PoseStamped pose_plan;
                pose_plan.pose.position.x = path[i].loc.x();
                pose_plan.pose.position.y = path[i].loc.y();
                pose_plan.pose.orientation.w = 1.0;  // Default orientation (no rotation)
                pose_plan.header.stamp = this->get_clock()->now();
                pose_plan.header.frame_id = CONFIG_map_frame;
                path_msg->poses.push_back(pose_plan);
            }
            path_pub_->publish(std::move(path_msg));

            // Draw green lines connecting consecutive waypoints for visualization
            for (size_t i = 1; i < path.size(); i++) {
                visualization::DrawLine(path[i - 1].loc, path[i].loc, 0x007F00, global_viz_msg_);
            }

            // Publish current carrot (intermediate target) point
            Eigen::Vector2f carrot;
            if (navigation_.GetCarrot(carrot)) {
                auto carrot_msg = std::make_unique<nav_msgs::msg::Path>();
                carrot_msg->header.stamp = this->get_clock()->now();
                carrot_msg->header.frame_id = CONFIG_map_frame;

                // Single pose representing the carrot point
                geometry_msgs::msg::PoseStamped carrot_pose;
                carrot_pose.pose.position.x = carrot.x();
                carrot_pose.pose.position.y = carrot.y();
                carrot_pose.pose.orientation.w = 1.0;  // Default orientation
                carrot_pose.header.stamp = this->get_clock()->now();
                carrot_pose.header.frame_id = CONFIG_map_frame;
                carrot_msg->poses.push_back(carrot_pose);

                carrot_pub_->publish(std::move(carrot_msg));
            }
        }
    }

    void DrawTarget() {
        const float carrot_dist = navigation_.params_.carrot_dist;
        // navigation_.local_target_ is in the predicted base frame at actuation time.
        // For visualization, compute the equivalent local target in the CURRENT base frame.
        const Eigen::Affine2f T_map_base_pred =
            Eigen::Translation2f(navigation_.robot_loc_fp_) * Eigen::Rotation2Df(navigation_.robot_angle_fp_);
        const Eigen::Vector2f target_map = T_map_base_pred * navigation_.local_target_;
        const Eigen::Rotation2Df R_now_inv(-navigation_.robot_angle_);
        const Eigen::Vector2f target = R_now_inv * (target_map - navigation_.robot_loc_);

        // Draw carrot distance circle (light gray)
        visualization::DrawArc(Eigen::Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);

        // Draw local target point (magenta cross)
        visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);

        // Compute final goal position in current base frame for visualization
        const Eigen::Vector2f goal_in_local = R_now_inv * (navigation_.nav_goal_loc_ - navigation_.robot_loc_);

        // Draw target distance tolerance circle around final goal: red if robot within target distance, light gray
        // otherwise
        const float goal_dist2 = (navigation_.nav_goal_loc_ - navigation_.robot_loc_fp_).squaredNorm();
        const float target_dist_tolerance = navigation_.params_.target_dist_tolerance;
        const bool within_target_dist = (goal_dist2 <= target_dist_tolerance * target_dist_tolerance);
        const uint32_t target_dist_color =
            within_target_dist ? 0xFF0000 : 0xE0E0E0;  // red if within, light gray otherwise
        visualization::DrawArc(goal_in_local, target_dist_tolerance, -M_PI, M_PI, target_dist_color, local_viz_msg_);

        // Draw nudge circle around final goal: red if robot within nudge distance, light gray otherwise
        const float nudge_dist_tolerance = navigation_.params_.nudge_dist_tolerance;
        const bool within_nudge = (goal_dist2 <= nudge_dist_tolerance * nudge_dist_tolerance);
        const uint32_t nudge_color = within_nudge ? 0xFF0000 : 0xE0E0E0;  // red if within, light gray otherwise
        visualization::DrawArc(goal_in_local, nudge_dist_tolerance, -M_PI, M_PI, nudge_color, local_viz_msg_);

        // Draw lidar FOV cone boundaries (dark yellow)
        const float fov_length = 2.0f;  // Length of FOV lines in meters
        const float fov_half_angle = CONFIG_lidar_fov_half_angle;
        Eigen::Vector2f fov_left(fov_length * cos(fov_half_angle), fov_length * sin(fov_half_angle));
        Eigen::Vector2f fov_right(fov_length * cos(-fov_half_angle), fov_length * sin(-fov_half_angle));
        visualization::DrawLine(Eigen::Vector2f(0, 0), fov_left, 0xFFCC00, local_viz_msg_);
        visualization::DrawLine(Eigen::Vector2f(0, 0), fov_right, 0xFFCC00, local_viz_msg_);
    }

    void DrawRobot() {
        const float kRobotLength = navigation_.params_.robot_length;
        const float kRobotWidth = navigation_.params_.robot_width;
        const float kBaseLinkOffsetX = navigation_.params_.geometric_center_offset.x;
        const float kBaseLinkOffsetY = navigation_.params_.geometric_center_offset.y;
        const float kObstacleMargin = navigation_.params_.obstacle_margin;

        // Draw robot with margin (light gray outline showing safety buffer)
        {
            const float x_min = kBaseLinkOffsetX - 0.5f * kRobotLength - kObstacleMargin;
            const float x_max = kBaseLinkOffsetX + 0.5f * kRobotLength + kObstacleMargin;
            const float y_min = kBaseLinkOffsetY - 0.5f * kRobotWidth - kObstacleMargin;
            const float y_max = kBaseLinkOffsetY + 0.5f * kRobotWidth + kObstacleMargin;
            visualization::DrawLine(Eigen::Vector2f(x_min, y_max), Eigen::Vector2f(x_min, y_min), 0xC0C0C0,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_max, y_max), Eigen::Vector2f(x_max, y_min), 0xC0C0C0,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_min, y_max), Eigen::Vector2f(x_max, y_max), 0xC0C0C0,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_min, y_min), Eigen::Vector2f(x_max, y_min), 0xC0C0C0,
                                    local_viz_msg_);
        }

        // Draw actual robot footprint (black outline)
        {
            const float x_min = kBaseLinkOffsetX - 0.5f * kRobotLength;
            const float x_max = kBaseLinkOffsetX + 0.5f * kRobotLength;
            const float y_min = kBaseLinkOffsetY - 0.5f * kRobotWidth;
            const float y_max = kBaseLinkOffsetY + 0.5f * kRobotWidth;
            visualization::DrawLine(Eigen::Vector2f(x_min, y_max), Eigen::Vector2f(x_min, y_min), 0x000000,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_max, y_max), Eigen::Vector2f(x_max, y_min), 0x000000,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_min, y_max), Eigen::Vector2f(x_max, y_max), 0x000000,
                                    local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(x_min, y_min), Eigen::Vector2f(x_max, y_min), 0x000000,
                                    local_viz_msg_);
        }

        // Draw forward-predicted robot footprint (fp) in local (base) frame (blue outline)
        {
            const float x_min_fp = kBaseLinkOffsetX - 0.5f * kRobotLength;
            const float x_max_fp = kBaseLinkOffsetX + 0.5f * kRobotLength;
            const float y_min_fp = kBaseLinkOffsetY - 0.5f * kRobotWidth;
            const float y_max_fp = kBaseLinkOffsetY + 0.5f * kRobotWidth;

            // Predicted pose in MAP: (t_fp, R_fp)
            const Eigen::Rotation2Df R_fp(navigation_.robot_angle_fp_);
            const Eigen::Vector2f t_fp = navigation_.robot_loc_fp_;

            // Corners in MAP
            const Eigen::Vector2f p1m = t_fp + R_fp * Eigen::Vector2f(x_min_fp, y_max_fp);
            const Eigen::Vector2f p2m = t_fp + R_fp * Eigen::Vector2f(x_min_fp, y_min_fp);
            const Eigen::Vector2f p3m = t_fp + R_fp * Eigen::Vector2f(x_max_fp, y_min_fp);
            const Eigen::Vector2f p4m = t_fp + R_fp * Eigen::Vector2f(x_max_fp, y_max_fp);

            // Transform MAP -> current BASE frame (local)
            const Eigen::Rotation2Df R_now_inv(-navigation_.robot_angle_);
            const Eigen::Vector2f t_now = navigation_.robot_loc_;
            auto ToLocal = [&](const Eigen::Vector2f& pm) { return R_now_inv * (pm - t_now); };

            const Eigen::Vector2f p1 = ToLocal(p1m);
            const Eigen::Vector2f p2 = ToLocal(p2m);
            const Eigen::Vector2f p3 = ToLocal(p3m);
            const Eigen::Vector2f p4 = ToLocal(p4m);

            // Draw FP robot in LOCAL (base_link) frame
            visualization::DrawLine(p1, p2, 0x66CCFF, local_viz_msg_);
            visualization::DrawLine(p2, p3, 0x66CCFF, local_viz_msg_);
            visualization::DrawLine(p3, p4, 0x66CCFF, local_viz_msg_);
            visualization::DrawLine(p4, p1, 0x66CCFF, local_viz_msg_);
        }

        // Draw base_link marker at origin (small cross)
        visualization::DrawCross(Eigen::Vector2f(0, 0), 0.05, 0x000000, local_viz_msg_);
    }

    void DrawYawTarget() {
        // Only draw if the Navigation has initialized the setpoint
        if (!navigation_.yaw_align_sp_init_) return;

        // Arrow in current BASE frame that points along the MAP-frame yaw setpoint
        const float L = 0.9f;  // arrow length in meters
        const Eigen::Rotation2Df R_now_inv(-navigation_.robot_angle_);
        const Eigen::Rotation2Df R_sp(navigation_.yaw_align_sp_map_);

        const Eigen::Vector2f tip_local = R_now_inv * (R_sp * Eigen::Vector2f(L, 0.0f));

        // Shaft
        visualization::DrawLine(Eigen::Vector2f(0, 0), tip_local, 0x00A0FF, local_viz_msg_);

        // Arrowhead
        const float theta = std::atan2(tip_local.y(), tip_local.x());
        const float head_back = 0.18f * L;
        const float head_side = 0.10f * L;
        const Eigen::Vector2f back = tip_local - head_back * Eigen::Vector2f(std::cos(theta), std::sin(theta));
        const Eigen::Vector2f left = back + head_side * Eigen::Vector2f(std::cos(theta + 0.8f), std::sin(theta + 0.8f));
        const Eigen::Vector2f right =
            back + head_side * Eigen::Vector2f(std::cos(theta - 0.8f), std::sin(theta - 0.8f));
        visualization::DrawLine(tip_local, left, 0x00A0FF, local_viz_msg_);
        visualization::DrawLine(tip_local, right, 0x00A0FF, local_viz_msg_);
    }

    void DrawPathOptions() {
        std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> path_rollouts = navigation_.sampled_paths_;
        std::shared_ptr<motion_primitives::PathRolloutBase> best_option = navigation_.best_option_;

        // If no paths are available, don't draw anything (messages are already cleared)
        if (path_rollouts.empty() && best_option == nullptr) {
            return;
        }

        // Draw path options that participate in optimization (Length > 0) in light blue
        constexpr uint32_t kCandidatePathColor = 0x80A0FF;  // Light blue for non-winning paths
        for (const auto& rollout : path_rollouts) {
            if (rollout->Length() <= 0.0f) continue;  // Skip zero-length unusable paths

            // Handle constant curvature arc paths
            const auto* arc = dynamic_cast<const motion_primitives::ConstantCurvatureArcPath*>(rollout.get());
            if (arc) {
                // Draw arc path
                visualization::DrawPathOption(arc->curvature, arc->Length(), arc->Clearance(), kCandidatePathColor,
                                              false, local_viz_msg_);
            }
            // Handle omnidirectional straight-line paths
            const auto* omni = dynamic_cast<const motion_primitives::OmnidirectionalMovePath*>(rollout.get());
            if (omni) {
                // Draw straight line from origin to endpoint
                Eigen::Vector2f endpoint = omni->EndPoint().translation;
                visualization::DrawLine(Eigen::Vector2f(0, 0), endpoint, kCandidatePathColor, local_viz_msg_);
            }
        }

        // Highlight the selected best path option in red
        if (best_option != nullptr) {
            // Handle best arc path
            const auto* best_arc = dynamic_cast<const motion_primitives::ConstantCurvatureArcPath*>(best_option.get());
            if (best_arc) {
                // Draw selected arc (red: 0xFF0000)
                visualization::DrawPathOption(best_arc->curvature, best_arc->Length(), best_arc->Clearance(), 0xFF0000,
                                              true, local_viz_msg_);
            }
            // Handle best omnidirectional path
            const auto* best_omni = dynamic_cast<const motion_primitives::OmnidirectionalMovePath*>(best_option.get());
            if (best_omni) {
                // Draw selected straight path (red: 0xFF0000)
                const Eigen::Vector2f endpoint = best_omni->EndPoint().translation;
                visualization::DrawLine(Eigen::Vector2f(0, 0), endpoint, 0xFF0000, local_viz_msg_);

                // Draw clearance corridor: inflated robot body extent + clearance on each side
                // Clearance is computed w.r.t. inflated body, so corridor shows inflated body + extra clearance
                const Eigen::Vector2f dir_lateral(-best_omni->direction.y(), best_omni->direction.x());
                const motion_primitives::OffsetRect robot_inflated = {
                    Eigen::Vector2f(navigation_.params_.geometric_center_offset.x,
                                    navigation_.params_.geometric_center_offset.y),
                    0.5f * navigation_.params_.robot_length + navigation_.params_.obstacle_margin,
                    0.5f * navigation_.params_.robot_width + navigation_.params_.obstacle_margin};
                const float clearance = best_omni->Clearance();
                // Corridor edge = inflated body extent (from base_link) + clearance
                const Eigen::Vector2f offset_left = (clearance + robot_inflated.support(dir_lateral)) * dir_lateral;
                const Eigen::Vector2f offset_right =
                    (clearance + robot_inflated.support(-dir_lateral)) * (-dir_lateral);
                constexpr uint32_t kCorridorColor = 0xFF8080;  // Light red
                visualization::DrawLine(offset_left, endpoint + offset_left, kCorridorColor, local_viz_msg_);
                visualization::DrawLine(offset_right, endpoint + offset_right, kCorridorColor, local_viz_msg_);
            }
        }
    }

    void LoadConfig(navigation::NavigationParameters* params) {
        config_reader::ConfigReader reader({FLAGS_robot_config});
        params->dt = CONFIG_dt;
        params->linear_limits =
            navigation::MotionLimits(CONFIG_max_linear_accel, CONFIG_max_linear_decel, CONFIG_max_linear_speed);
        params->angular_limits =
            navigation::MotionLimits(CONFIG_max_angular_accel, CONFIG_max_angular_decel, CONFIG_max_angular_speed);
        params->actuation_latency = CONFIG_actuation_latency;
        params->obstacle_margin = CONFIG_obstacle_margin;
        params->num_options = CONFIG_num_options;
        params->robot_width = CONFIG_robot_width;
        params->robot_length = CONFIG_robot_length;
        params->geometric_center_offset.x = CONFIG_geometric_center_offset_x;
        params->geometric_center_offset.y = CONFIG_geometric_center_offset_y;
        params->max_rollout_length = CONFIG_max_rollout_length;
        params->max_lookahead_fpl = CONFIG_max_lookahead_fpl;
        params->clearance_band = CONFIG_clearance_band;
        params->lidar_fov_half_angle = CONFIG_lidar_fov_half_angle;
        params->can_traverse_stairs = CONFIG_can_traverse_stairs;
        params->target_dist_tolerance = CONFIG_target_dist_tolerance;
        params->nudge_dist_tolerance = CONFIG_nudge_dist_tolerance;
        params->target_vel_tolerance = CONFIG_target_vel_tolerance;
        params->target_angle_tolerance = CONFIG_target_angle_tolerance;
        params->target_omega_tolerance = CONFIG_target_omega_tolerance;
        params->evaluator_type = CONFIG_evaluator_type;
        params->carrot_dist = CONFIG_carrot_dist;
        params->motion_primitives_mode = CONFIG_motion_primitives_mode;
        params->do_ang_toc = CONFIG_do_ang_toc;
        params->max_plan_deviation = CONFIG_max_plan_deviation;
        params->laser_height = CONFIG_laser_height;
        params->stuck_meta_override_obstacle_margin = CONFIG_stuck_meta_override_obstacle_margin;
        params->stuck_meta_stuck_timeout_sec = CONFIG_stuck_meta_stuck_timeout_sec;
        params->stuck_meta_improve_eps = CONFIG_stuck_meta_improve_eps;
        params->apply_custom_cmd_map = CONFIG_apply_custom_cmd_map;
        params->cmd_map_x_slope_pos = CONFIG_cmd_map_x_slope_pos;
        params->cmd_map_x_intercept_pos = CONFIG_cmd_map_x_intercept_pos;
        params->cmd_map_x_slope_neg = CONFIG_cmd_map_x_slope_neg;
        params->cmd_map_x_intercept_neg = CONFIG_cmd_map_x_intercept_neg;
        params->cmd_map_y_slope_pos = CONFIG_cmd_map_y_slope_pos;
        params->cmd_map_y_intercept_pos = CONFIG_cmd_map_y_intercept_pos;
        params->cmd_map_y_slope_neg = CONFIG_cmd_map_y_slope_neg;
        params->cmd_map_y_intercept_neg = CONFIG_cmd_map_y_intercept_neg;
        params->cmd_map_r_slope_pos = CONFIG_cmd_map_r_slope_pos;
        params->cmd_map_r_intercept_pos = CONFIG_cmd_map_r_intercept_pos;
        params->cmd_map_r_slope_neg = CONFIG_cmd_map_r_slope_neg;
        params->cmd_map_r_intercept_neg = CONFIG_cmd_map_r_intercept_neg;
    }
};

void SignalHandler(int) {
    RCLCPP_INFO(rclcpp::get_logger("navigation"), "Shutting down navigation node");
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    // Initialize gflags and glog
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Check if robot config was provided
    if (FLAGS_robot_config.empty()) {
        fprintf(stderr, "ERROR: --robot_config flag is required. Please specify a robot config file path.\n");
        fprintf(stderr, "Usage: %s --robot_config=<path_to_config_file> [other options]\n", argv[0]);
        exit(1);
    }

    // Set up signal handler
    signal(SIGINT, SignalHandler);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<NavigationNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("navigation"), "[main] Exception in navigation node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
