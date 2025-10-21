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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");
DEFINE_bool(no_joystick, true, "Whether to use a joystick or not");

// Configuration parameters
CONFIG_STRINGLIST(laser_topics, "NavigationParameters.laser_topics");
CONFIG_STRING(laser_frame, "NavigationParameters.laser_frame");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(localization_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");
CONFIG_FLOAT(dt, "NavigationParameters.dt");
CONFIG_FLOAT(max_linear_accel, "NavigationParameters.max_linear_accel");
CONFIG_FLOAT(max_linear_decel, "NavigationParameters.max_linear_decel");
CONFIG_FLOAT(max_linear_speed, "NavigationParameters.max_linear_speed");
CONFIG_FLOAT(max_angular_accel, "NavigationParameters.max_angular_accel");
CONFIG_FLOAT(max_angular_decel, "NavigationParameters.max_angular_decel");
CONFIG_FLOAT(max_angular_speed, "NavigationParameters.max_angular_speed");
CONFIG_FLOAT(system_latency, "NavigationParameters.system_latency");
CONFIG_FLOAT(obstacle_margin, "NavigationParameters.obstacle_margin");
CONFIG_INT(num_options, "NavigationParameters.num_options");
CONFIG_FLOAT(robot_width, "NavigationParameters.robot_width");
CONFIG_FLOAT(robot_length, "NavigationParameters.robot_length");
CONFIG_FLOAT(base_link_offset, "NavigationParameters.base_link_offset");
CONFIG_FLOAT(max_free_path_length, "NavigationParameters.max_free_path_length");
CONFIG_FLOAT(max_clearance, "NavigationParameters.max_clearance");
CONFIG_BOOL(use_map_speed, "NavigationParameters.use_map_speed");
CONFIG_FLOAT(target_dist_tolerance, "NavigationParameters.target_dist_tolerance");
CONFIG_FLOAT(target_vel_tolerance, "NavigationParameters.target_vel_tolerance");
CONFIG_FLOAT(target_angle_tolerance, "NavigationParameters.target_angle_tolerance");
CONFIG_FLOAT(local_fov, "NavigationParameters.local_fov");
CONFIG_BOOL(can_traverse_stairs, "NavigationParameters.can_traverse_stairs");
CONFIG_STRING(evaluator_type, "NavigationParameters.evaluator_type");
CONFIG_FLOAT(carrot_dist, "NavigationParameters.carrot_dist");
CONFIG_FLOAT(recovery_carrot_dist, "NavigationParameters.recovery_carrot_dist");
CONFIG_STRING(ackermann_drive_topic, "NavigationParameters.ackermann_drive_topic");
CONFIG_STRING(nav_status_topic, "NavigationParameters.nav_status_topic");
CONFIG_STRING(visualization_topic, "NavigationParameters.visualization_topic");
CONFIG_STRING(fp_pcl_topic, "NavigationParameters.fp_pcl_topic");
CONFIG_STRING(path_topic, "NavigationParameters.path_topic");
CONFIG_STRING(carrot_topic, "NavigationParameters.carrot_topic");
CONFIG_STRING(goto_topic, "NavigationParameters.goto_topic");
CONFIG_STRING(goto_amrl_topic, "NavigationParameters.goto_amrl_topic");
CONFIG_STRING(reset_nav_goals_topic, "NavigationParameters.reset_nav_goals_topic");
CONFIG_STRING(halt_topic, "NavigationParameters.halt_topic");
CONFIG_STRING(override_topic, "NavigationParameters.override_topic");

class NavigationNode : public rclcpp::Node, public std::enable_shared_from_this<NavigationNode> {
   public:
    NavigationNode() : Node("navigation"),
                       tf_buffer_(this->get_clock()),
                       tf_listener_(tf_buffer_),
                       run_(true),
                       enabled_(false),
                       received_odom_(false),
                       received_laser_(false),
                       current_angle_(0.0),
                       goal_angle_(0.0) {
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
        ackermann_drive_pub_ = this->create_publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>(
            CONFIG_ackermann_drive_topic, 1);
        twist_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            FLAGS_twist_drive_topic, 1);
        status_pub_ = this->create_publisher<amrl_msgs::msg::NavStatusMsg>(
            CONFIG_nav_status_topic, 1);
        viz_pub_ = this->create_publisher<amrl_msgs::msg::VisualizationMsg>(
            CONFIG_visualization_topic, 1);
        fp_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            CONFIG_fp_pcl_topic, 1);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(CONFIG_path_topic, 1);
        carrot_pub_ = this->create_publisher<nav_msgs::msg::Path>(CONFIG_carrot_topic, 1);

        // Create service
        nav_service_ = this->create_service<graph_navigation::srv::GraphNav>(
            "GraphNav", std::bind(&NavigationNode::PlanServiceCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            CONFIG_odom_topic, 1, std::bind(&NavigationNode::OdometryCallback, this, std::placeholders::_1));

        localization_sub_ = this->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            CONFIG_localization_topic, 1, std::bind(&NavigationNode::LocalizationCallback, this, std::placeholders::_1));

        // Create laser subscribers
        for (size_t i = 0; i < CONFIG_laser_topics.size(); ++i) {
            auto laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                CONFIG_laser_topics[i], 1,
                [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    LaserCallback(msg, CONFIG_laser_topics[i]);
                });
            laser_subs_.push_back(laser_sub);
        }

        goto_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            CONFIG_goto_topic, 1, std::bind(&NavigationNode::GoToCallback, this, std::placeholders::_1));

        goto_amrl_sub_ = this->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            CONFIG_goto_amrl_topic, 1, std::bind(&NavigationNode::GoToCallbackAMRL, this, std::placeholders::_1));

        reset_nav_goals_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            CONFIG_reset_nav_goals_topic, 1, std::bind(&NavigationNode::ResetNavGoalsCallback, this, std::placeholders::_1));

        enabler_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            CONFIG_enable_topic, 1, std::bind(&NavigationNode::EnablerCallback, this, std::placeholders::_1));

        halt_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            CONFIG_halt_topic, 1, std::bind(&NavigationNode::HaltCallback, this, std::placeholders::_1));

        override_sub_ = this->create_subscription<amrl_msgs::msg::Pose2Df>(
            CONFIG_override_topic, 1, std::bind(&NavigationNode::OverrideCallback, this, std::placeholders::_1));

        // Initialize visualization markers
        InitSimulatorVizMarkers();

        // Create timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(params_.dt),
            std::bind(&NavigationNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
    }

    ~NavigationNode() {
        run_ = false;
    }

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabler_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_sub_;
    rclcpp::Subscription<amrl_msgs::msg::Pose2Df>::SharedPtr override_sub_;

    // Service
    rclcpp::Service<graph_navigation::srv::GraphNav>::SharedPtr nav_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Navigation components
    navigation::Navigation navigation_;
    navigation::NavigationParameters params_;

    // State variables
    bool run_;
    bool enabled_;
    bool received_odom_;
    bool received_laser_;
    Eigen::Vector2f goal_{0, 0};
    Eigen::Vector2f current_loc_{0, 0};
    Eigen::Vector2f current_vel_{0, 0};
    float current_angle_;
    float goal_angle_;
    navigation::Odom odom_;
    std::vector<Eigen::Vector2f> point_cloud_;

    // Visualization
    amrl_msgs::msg::VisualizationMsg local_viz_msg_;
    amrl_msgs::msg::VisualizationMsg global_viz_msg_;
    visualization_msgs::msg::Marker line_list_marker_;
    visualization_msgs::msg::Marker pose_marker_;
    visualization_msgs::msg::Marker target_marker_;

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
    void EnablerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        enabled_ = msg->data;
    }

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
        current_loc_ = Eigen::Vector2f(msg->pose.x, msg->pose.y);
        current_angle_ = msg->pose.theta;
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
        navigation_.Resume();
    }

    void GoToCallbackAMRL(const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
        const Eigen::Vector2f loc(msg->pose.x, msg->pose.y);
        RCLCPP_INFO(this->get_logger(), "Goal: (%f,%f) %f°", loc.x(), loc.y(), msg->pose.theta);
        navigation_.SetNavGoal(loc, msg->pose.theta);
        navigation_.Resume();
    }

    void ResetNavGoalsCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Resetting all nav goals");
        navigation_.ResetNavGoals();
    }

    void HaltCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        navigation_.Pause();
    }

    void OverrideCallback(const amrl_msgs::msg::Pose2Df::SharedPtr msg) {
        const Eigen::Vector2f loc(msg->x, msg->y);
        navigation_.SetOverride(loc, msg->theta);
    }

    void PlanServiceCallback(const std::shared_ptr<graph_navigation::srv::GraphNav::Request> request,
                             std::shared_ptr<graph_navigation::srv::GraphNav::Response> response) {
        const Eigen::Vector2f start(request->start.x, request->start.y);
        const Eigen::Vector2f end(request->end.x, request->end.y);
        const std::vector<int> plan = navigation_.GlobalPlan(start, end);
        response->plan = plan;
    }

    void TimerCallback() {
        if (!run_) return;

        // Clear visualization messages
        visualization::ClearVisualizationMsg(local_viz_msg_);
        visualization::ClearVisualizationMsg(global_viz_msg_);
        received_laser_ = false;

        // Run navigation
        Eigen::Vector2f cmd_vel(0, 0);
        float cmd_angle_vel(0);

        bool nav_succeeded = navigation_.Run(this->get_clock()->now().seconds(), cmd_vel, cmd_angle_vel);

        // Publish status
        PublishNavStatus();

        if (nav_succeeded) {
            // Publish visualizations and commands
            PublishForwardPredictedPCL(navigation_.GetPredictedCloud());
            DrawRobot();

            if (navigation_.GetNavStatusUint8() != static_cast<uint8_t>(navigation::NavigationState::kStopped)) {
                DrawTarget();
                DrawPathOptions();
            }

            PublishVisualizationMarkers();
            PublishPath();

            // Update timestamps
            local_viz_msg_.header.stamp = this->get_clock()->now();
            global_viz_msg_.header.stamp = this->get_clock()->now();

            // Publish visualization messages
            viz_pub_->publish(local_viz_msg_);
            viz_pub_->publish(global_viz_msg_);

            // Send commands
            SendCommand(cmd_vel, cmd_angle_vel);
        }
    }

    // Helper functions
    navigation::Odom OdomHandler(const nav_msgs::msg::Odometry& msg) {
        navigation::Odom odom;
        odom.time = rclcpp::Time(msg.header.stamp).seconds();
        odom.orientation = {static_cast<float>(msg.pose.pose.orientation.w),
                            static_cast<float>(msg.pose.pose.orientation.x),
                            static_cast<float>(msg.pose.pose.orientation.y),
                            static_cast<float>(msg.pose.pose.orientation.z)};
        odom.position = {static_cast<float>(msg.pose.pose.position.x),
                         static_cast<float>(msg.pose.pose.position.y),
                         static_cast<float>(msg.pose.pose.position.z)};
        return odom;
    }

    void LaserHandler(const sensor_msgs::msg::LaserScan& msg, const std::string& topic) {
        auto& cache = laser_caches_[topic];

        if (cache.dtheta != msg.angle_increment ||
            cache.angle_min != msg.angle_min ||
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
            const float r = ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max) ? msg.ranges[i] : msg.range_max);
            point_cloud_[start_idx + i] = (cache.frame_tf * (r * cache.rays[i])).head<2>();
        }
    }

    void RetrieveTransform(const std_msgs::msg::Header& msg, Eigen::Affine3f& frame_tf) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                CONFIG_laser_frame, msg.frame_id, tf2::TimePointZero);

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
        status->status = navigation_.GetNavStatusUint8();
        status_pub_->publish(std::move(status));
    }

    void SendCommand(const Eigen::Vector2f& vel, float ang_vel) {
        // Determine commanded values first to avoid use-after-move on unique_ptr
        double cmd_lin_x = 0.0;
        double cmd_lin_y = 0.0;
        double cmd_ang_z = 0.0;
        if (FLAGS_no_joystick || enabled_) {
            cmd_lin_x = vel.x();
            cmd_lin_y = vel.y();
            cmd_ang_z = ang_vel;
        }

        // Minimal one-time debug to help trace potential crashes here
        static int send_cmd_dbg_printed = 0;
        if (send_cmd_dbg_printed == 0) {
            RCLCPP_INFO(this->get_logger(), "SendCommand: vx=%.3f vy=%.3f wz=%.3f", cmd_lin_x, cmd_lin_y, cmd_ang_z);
            send_cmd_dbg_printed = 1;
        }

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
        twist.time = this->get_clock()->now().seconds();
        twist.linear = {static_cast<float>(cmd_lin_x),
                        static_cast<float>(cmd_lin_y),
                        0.0f};
        twist.angular = {0.0f,
                         0.0f,
                         static_cast<float>(cmd_ang_z)};
        navigation_.UpdateCommandHistory(twist);
    }

    void PublishForwardPredictedPCL(const std::vector<Eigen::Vector2f>& pcl) {
        auto fp_pcl_msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        fp_pcl_msg->points.resize(pcl.size());
        for (size_t i = 0; i < pcl.size(); ++i) {
            fp_pcl_msg->points[i].x = pcl[i].x();
            fp_pcl_msg->points[i].y = pcl[i].y();
            fp_pcl_msg->points[i].z = 0.324;
        }
        fp_pcl_msg->header.stamp = this->get_clock()->now();
        fp_pcl_pub_->publish(std::move(fp_pcl_msg));
    }

    void PublishPath() {
        const auto path = navigation_.GetPlanPath();
        if (path.size() >= 2) {
            auto path_msg = std::make_unique<nav_msgs::msg::Path>();
            path_msg->header.stamp = this->get_clock()->now();
            path_msg->header.frame_id = "map";

            for (size_t i = 0; i < path.size(); i++) {
                geometry_msgs::msg::PoseStamped pose_plan;
                pose_plan.pose.position.x = path[i].loc.x();
                pose_plan.pose.position.y = path[i].loc.y();
                pose_plan.pose.orientation.w = 1.0;
                pose_plan.header.stamp = this->get_clock()->now();
                pose_plan.header.frame_id = "map";
                path_msg->poses.push_back(pose_plan);
            }
            path_pub_->publish(std::move(path_msg));

            // Draw path visualization
            for (size_t i = 1; i < path.size(); i++) {
                visualization::DrawLine(path[i - 1].loc, path[i].loc, 0x007F00, global_viz_msg_);
            }

            // Draw global path
            const auto global_path = navigation_.GetGlobalPath();
            for (size_t i = 1; i < global_path.size(); i++) {
                visualization::DrawLine(global_path[i - 1].loc, global_path[i].loc, 0xA86032, global_viz_msg_);
            }

            // Draw carrot
            Eigen::Vector2f carrot;
            if (navigation_.GetLocalCarrot(carrot)) {
                auto carrot_msg = std::make_unique<nav_msgs::msg::Path>();
                carrot_msg->header.stamp = this->get_clock()->now();
                carrot_msg->header.frame_id = "map";

                geometry_msgs::msg::PoseStamped carrot_pose;
                carrot_pose.pose.position.x = carrot.x();
                carrot_pose.pose.position.y = carrot.y();
                carrot_pose.pose.orientation.w = 1.0;
                carrot_pose.header.stamp = this->get_clock()->now();
                carrot_pose.header.frame_id = "map";
                carrot_msg->poses.push_back(carrot_pose);

                carrot_pub_->publish(std::move(carrot_msg));
            }
        }
    }

    void DrawTarget() {
        const float carrot_dist = navigation_.GetCarrotDist();
        const Eigen::Vector2f target = navigation_.GetTarget();

        visualization::DrawArc(Eigen::Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0, local_viz_msg_);
        visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);
    }

    void DrawRobot() {
        const float kRobotLength = navigation_.GetRobotLength();
        const float kRobotWidth = navigation_.GetRobotWidth();
        const float kRearAxleOffset = 0.0;
        const float kObstacleMargin = navigation_.GetObstacleMargin();

        // Draw robot with margin
        {
            const float l1 = -0.5 * kRobotLength - kRearAxleOffset - kObstacleMargin;
            const float l2 = 0.5 * kRobotLength - kRearAxleOffset + kObstacleMargin;
            const float w = 0.5 * kRobotWidth + kObstacleMargin;
            visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l1, -w), 0xC0C0C0, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l2, w), Eigen::Vector2f(l2, -w), 0xC0C0C0, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l2, w), 0xC0C0C0, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l1, -w), Eigen::Vector2f(l2, -w), 0xC0C0C0, local_viz_msg_);
        }

        // Draw actual robot
        {
            const float l1 = -0.5 * kRobotLength - kRearAxleOffset;
            const float l2 = 0.5 * kRobotLength - kRearAxleOffset;
            const float w = 0.5 * kRobotWidth;
            visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l1, -w), 0x000000, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l2, w), Eigen::Vector2f(l2, -w), 0x000000, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l2, w), 0x000000, local_viz_msg_);
            visualization::DrawLine(Eigen::Vector2f(l1, -w), Eigen::Vector2f(l2, -w), 0x000000, local_viz_msg_);
        }
    }

    void DrawPathOptions() {
        std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> path_rollouts =
            navigation_.GetLastPathOptions();
        std::shared_ptr<motion_primitives::PathRolloutBase> best_option = navigation_.GetOption();

        for (const auto& rollout : path_rollouts) {
            const auto* arc = dynamic_cast<const motion_primitives::ConstantCurvatureArc*>(rollout.get());
            if (arc) {
                visualization::DrawPathOption(arc->curvature, arc->Length(), arc->Clearance(),
                                              0x0000FF, false, local_viz_msg_);
            }
        }

        if (best_option != nullptr) {
            const auto* best_arc = dynamic_cast<const motion_primitives::ConstantCurvatureArc*>(best_option.get());
            if (best_arc) {
                visualization::DrawPathOption(best_arc->curvature, best_arc->Length(), best_arc->Clearance(),
                                              0xFF0000, true, local_viz_msg_);
            }
        }
    }

    void PublishVisualizationMarkers() {
        // This would publish the visualization markers - implementation depends on visualization system
        // For now, we'll skip this as it requires the full visualization marker setup
    }

    void InitSimulatorVizMarkers() {
        // Initialize visualization markers - simplified for now
        RCLCPP_INFO(this->get_logger(), "Visualization markers initialized");
    }

    void LoadConfig(navigation::NavigationParameters* params) {
        config_reader::ConfigReader reader({FLAGS_robot_config});
        params->dt = CONFIG_dt;
        params->linear_limits = navigation::MotionLimits(
            CONFIG_max_linear_accel, CONFIG_max_linear_decel, CONFIG_max_linear_speed);
        params->angular_limits = navigation::MotionLimits(
            CONFIG_max_angular_accel, CONFIG_max_angular_decel, CONFIG_max_angular_speed);
        params->system_latency = CONFIG_system_latency;
        params->obstacle_margin = CONFIG_obstacle_margin;
        params->num_options = CONFIG_num_options;
        params->robot_width = CONFIG_robot_width;
        params->robot_length = CONFIG_robot_length;
        params->base_link_offset = CONFIG_base_link_offset;
        params->max_free_path_length = CONFIG_max_free_path_length;
        params->max_clearance = CONFIG_max_clearance;
        params->use_map_speed = CONFIG_use_map_speed;
        params->can_traverse_stairs = CONFIG_can_traverse_stairs;
        params->target_dist_tolerance = CONFIG_target_dist_tolerance;
        params->target_vel_tolerance = CONFIG_target_vel_tolerance;
        params->target_angle_tolerance = CONFIG_target_angle_tolerance;
        params->local_fov = CONFIG_local_fov;
        params->evaluator_type = CONFIG_evaluator_type;
        params->carrot_dist = CONFIG_carrot_dist;
        params->recovery_carrot_dist = CONFIG_recovery_carrot_dist;
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
