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

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
DEFINE_string(robot_config, "config/navigation.lua", "Robot config file");
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
CONFIG_FLOAT(base_link_offset_x, "NavigationParameters.base_link_offset_x");
CONFIG_FLOAT(base_link_offset_y, "NavigationParameters.base_link_offset_y");
CONFIG_FLOAT(max_free_path_length, "NavigationParameters.max_free_path_length");
CONFIG_FLOAT(max_clearance, "NavigationParameters.max_clearance");
CONFIG_FLOAT(local_half_fov, "NavigationParameters.local_half_fov");
CONFIG_FLOAT(center_threshold, "NavigationParameters.center_threshold");
CONFIG_BOOL(can_traverse_stairs, "NavigationParameters.can_traverse_stairs");
CONFIG_FLOAT(target_dist_tolerance, "NavigationParameters.target_dist_tolerance");
CONFIG_FLOAT(target_vel_tolerance, "NavigationParameters.target_vel_tolerance");
CONFIG_FLOAT(target_angle_tolerance, "NavigationParameters.target_angle_tolerance");
CONFIG_FLOAT(target_omega_tolerance, "NavigationParameters.target_omega_tolerance");
CONFIG_STRING(evaluator_type, "NavigationParameters.evaluator_type");
CONFIG_FLOAT(carrot_dist, "NavigationParameters.carrot_dist");
CONFIG_STRING(motion_primitives_mode, "NavigationParameters.motion_primitives_mode");
CONFIG_BOOL(do_ang_toc, "NavigationParameters.do_ang_toc");

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

// ROS Topics and Frames
CONFIG_STRINGLIST(laser_topics, "ROSTopics.laser_topics");
CONFIG_STRING(laser_frame, "ROSTopics.laser_frame");
CONFIG_STRING(odom_topic, "ROSTopics.odom_topic");
CONFIG_STRING(localization_topic, "ROSTopics.localization_topic");
CONFIG_STRING(ackermann_drive_topic, "ROSTopics.ackermann_drive_topic");
CONFIG_STRING(nav_status_topic, "ROSTopics.nav_status_topic");
CONFIG_STRING(visualization_topic, "ROSTopics.visualization_topic");
CONFIG_STRING(fp_pcl_topic, "ROSTopics.fp_pcl_topic");
CONFIG_STRING(path_topic, "ROSTopics.path_topic");
CONFIG_STRING(carrot_topic, "ROSTopics.carrot_topic");
CONFIG_STRING(goto_topic, "ROSTopics.goto_topic");
CONFIG_STRING(goto_amrl_topic, "ROSTopics.goto_amrl_topic");
CONFIG_STRING(reset_nav_goals_topic, "ROSTopics.reset_nav_goals_topic");
CONFIG_STRING(halt_topic, "ROSTopics.halt_topic");
CONFIG_STRING(twist_drive_topic, "ROSTopics.twist_drive_topic");

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

        // Load map
        std::string map_path = navigation::GetMapPath(FLAGS_maps_dir, FLAGS_map);
        if (!FileExists(map_path)) {
            RCLCPP_ERROR(this->get_logger(), "Could not find navigation map file at %s", map_path.c_str());
            throw std::runtime_error("Map file not found");
        }

        // Initialize navigation
        navigation_.Initialize(params_, map_path);

        // Initialize visualization messages
        local_viz_msg_ = visualization::NewVisualizationMessage(CONFIG_laser_frame, "navigation_local");
        global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");

        // Create publishers
        ackermann_drive_pub_ =
            this->create_publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>(CONFIG_ackermann_drive_topic, 1);
        twist_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(CONFIG_twist_drive_topic, 1);
        status_pub_ = this->create_publisher<amrl_msgs::msg::NavStatusMsg>(CONFIG_nav_status_topic, 1);
        viz_pub_ = this->create_publisher<amrl_msgs::msg::VisualizationMsg>(CONFIG_visualization_topic, 1);
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

        // Create timer for main loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(params_.dt),
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

    // Service
    rclcpp::Service<graph_navigation::srv::GraphNav>::SharedPtr nav_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Navigation components
    navigation::Navigation navigation_;
    navigation::NavigationParameters params_;

    // State variables
    bool run_;
    bool received_odom_;
    bool received_laser_;
    navigation::Odom odom_;
    std::vector<Eigen::Vector2f> point_cloud_;

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
        static std::string map = "";
        navigation_.UpdateLocation(Eigen::Vector2f(msg->pose.x, msg->pose.y), msg->pose.theta);
        if (map != msg->map) {
            map = msg->map;
            navigation_.UpdateMap(navigation::GetMapPath(FLAGS_maps_dir, msg->map));
        }
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

    void TimerCallback() {
        if (!run_) return;
        const double timer_callback_start_time = this->get_clock()->now().seconds();
        std::string start_msg = "TimerCallback started at timestamp: " + std::to_string(timer_callback_start_time);
        navigation::navigation_debug::DebugLog(start_msg);
        const auto timer_start = std::chrono::steady_clock::now();

        // Clear visualization messages
        visualization::ClearVisualizationMsg(local_viz_msg_);
        visualization::ClearVisualizationMsg(global_viz_msg_);
        received_laser_ = false;  // ?? why is this here? why happening at each callback?

        Eigen::Vector2f cmd_vel(0, 0);
        float cmd_angle_vel(0);
        const double cmd_plan_start_time = this->get_clock()->now().seconds();
        bool nav_succeeded = navigation_.Run(cmd_plan_start_time, cmd_vel, cmd_angle_vel);

        PublishNavStatus();
        if (nav_succeeded) {
            // Publish visualizations
            PublishForwardPredictedPCL(navigation_.fp_point_cloud_);
            DrawRobot();
            if (static_cast<uint8_t>(navigation_.nav_state_) !=
                static_cast<uint8_t>(navigation::NavigationState::kStopped)) {
                DrawTarget();
                DrawPathOptions();
            }
            PublishPath();
            local_viz_msg_.header.stamp = this->get_clock()->now();
            global_viz_msg_.header.stamp = this->get_clock()->now();
            viz_pub_->publish(local_viz_msg_);
            viz_pub_->publish(global_viz_msg_);

            // Send commands
            SendCommand(cmd_vel, cmd_angle_vel, cmd_plan_start_time);
        }

        const auto timer_end = std::chrono::steady_clock::now();
        const double total_ms = std::chrono::duration<double, std::milli>(timer_end - timer_start).count();

        // Log end-to-end TimerCallback duration
        std::string timer_msg = std::string("[") + std::to_string(static_cast<int>(navigation_.nav_state_)) +
                                "] TimerCallback took " + std::to_string(total_ms) + " ms";
        navigation::navigation_debug::DebugLog(timer_msg);
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
                tf_buffer_.lookupTransform(CONFIG_laser_frame, msg.frame_id, tf2::TimePointZero);

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
            fp_pcl_msg->points[i].z =
                0.324;  // ?? is this laser height above ground? if so, should be a config parameter
        }
        fp_pcl_msg->header.stamp = this->get_clock()->now();
        fp_pcl_pub_->publish(std::move(fp_pcl_msg));
    }

    void PublishPath() {
        const auto path = navigation_.plan_path_;
        if (path.size() >= 2) {  // ?? is it due to (start, end) atleast
            // Publish full planned path as nav_msgs::Path
            auto path_msg = std::make_unique<nav_msgs::msg::Path>();
            path_msg->header.stamp = this->get_clock()->now();
            path_msg->header.frame_id = "map";

            // Convert each waypoint to a pose in the path
            for (size_t i = 0; i < path.size(); i++) {
                geometry_msgs::msg::PoseStamped pose_plan;
                pose_plan.pose.position.x = path[i].loc.x();
                pose_plan.pose.position.y = path[i].loc.y();
                pose_plan.pose.orientation.w = 1.0;  // Default orientation (no rotation)
                pose_plan.header.stamp = this->get_clock()->now();
                pose_plan.header.frame_id = "map";
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
                carrot_msg->header.frame_id = "map";

                // Single pose representing the carrot point
                geometry_msgs::msg::PoseStamped carrot_pose;
                carrot_pose.pose.position.x = carrot.x();
                carrot_pose.pose.position.y = carrot.y();
                carrot_pose.pose.orientation.w = 1.0;  // Default orientation
                carrot_pose.header.stamp = this->get_clock()->now();
                carrot_pose.header.frame_id = "map";
                carrot_msg->poses.push_back(carrot_pose);

                carrot_pub_->publish(std::move(carrot_msg));
            }
        }
    }

    void DrawTarget() {
        // ?? BUG: i think for viz we do NOT need fp
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

        // Draw FOV cone boundaries (dark yellow)
        const float fov_length = 2.0f;  // Length of FOV lines in meters
        const float fov_half_angle = CONFIG_local_half_fov;
        Eigen::Vector2f fov_left(fov_length * cos(fov_half_angle), fov_length * sin(fov_half_angle));
        Eigen::Vector2f fov_right(fov_length * cos(-fov_half_angle), fov_length * sin(-fov_half_angle));
        visualization::DrawLine(Eigen::Vector2f(0, 0), fov_left, 0xFFCC00, local_viz_msg_);
        visualization::DrawLine(Eigen::Vector2f(0, 0), fov_right, 0xFFCC00, local_viz_msg_);
    }

    void DrawRobot() {
        const float kRobotLength = navigation_.params_.robot_length;
        const float kRobotWidth = navigation_.params_.robot_width;
        const float kBaseLinkOffsetX = navigation_.params_.base_link_offset_x;
        const float kBaseLinkOffsetY = navigation_.params_.base_link_offset_y;
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
    }

    void DrawPathOptions() {
        std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> path_rollouts = navigation_.sampled_paths_;
        std::shared_ptr<motion_primitives::PathRolloutBase> best_option = navigation_.best_option_;

        // Draw all sampled path options in blue
        for (const auto& rollout : path_rollouts) {
            // Handle constant curvature arc paths
            const auto* arc = dynamic_cast<const motion_primitives::ConstantCurvatureArcPath*>(rollout.get());
            if (arc) {
                // Draw arc path (blue: 0x0000FF)
                visualization::DrawPathOption(arc->curvature, arc->Length(), arc->Clearance(), 0x0000FF, false,
                                              local_viz_msg_);
            }
            // Handle omnidirectional straight-line paths
            const auto* omni = dynamic_cast<const motion_primitives::OmnidirectionalMovePath*>(rollout.get());
            if (omni) {
                // Draw straight line from origin to endpoint (blue: 0x0000FF)
                Eigen::Vector2f endpoint = omni->EndPoint().translation;
                visualization::DrawLine(Eigen::Vector2f(0, 0), endpoint, 0x0000FF, local_viz_msg_);
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
                Eigen::Vector2f endpoint = best_omni->EndPoint().translation;
                visualization::DrawLine(Eigen::Vector2f(0, 0), endpoint, 0xFF0000, local_viz_msg_);

                // Draw clearance boundaries showing minimum distance to obstacles (red: 0xFF0000)
                const float clearance = best_omni->Clearance();
                // Calculate perpendicular vector for clearance boundaries
                Eigen::Vector2f perp(-best_omni->direction.y(), best_omni->direction.x());
                Eigen::Vector2f clearance_offset = clearance * perp;
                // Draw parallel lines showing clearance boundaries
                visualization::DrawLine(clearance_offset, endpoint + clearance_offset, 0xFF0000, local_viz_msg_);
                visualization::DrawLine(-clearance_offset, endpoint - clearance_offset, 0xFF0000, local_viz_msg_);
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
        params->base_link_offset_x = CONFIG_base_link_offset_x;
        params->base_link_offset_y = CONFIG_base_link_offset_y;
        params->max_free_path_length = CONFIG_max_free_path_length;
        params->max_clearance = CONFIG_max_clearance;
        params->local_half_fov = CONFIG_local_half_fov;
        params->center_threshold = CONFIG_center_threshold;
        params->can_traverse_stairs = CONFIG_can_traverse_stairs;
        params->target_dist_tolerance = CONFIG_target_dist_tolerance;
        params->target_vel_tolerance = CONFIG_target_vel_tolerance;
        params->target_angle_tolerance = CONFIG_target_angle_tolerance;
        params->target_omega_tolerance = CONFIG_target_omega_tolerance;
        params->evaluator_type = CONFIG_evaluator_type;
        params->carrot_dist = CONFIG_carrot_dist;
        params->motion_primitives_mode = CONFIG_motion_primitives_mode;
        params->do_ang_toc = CONFIG_do_ang_toc;
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
