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
\brief   Main entry point for ROS2 social navigation
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================

#include <signal.h>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
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
#include <ament_index_cpp/get_package_share_directory.hpp>

// AMRL includes
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/msg/human_state_array_msg.hpp"
#include "amrl_msgs/msg/human_state_msg.hpp"
#include "amrl_msgs/srv/social_pips_srv.hpp"

// Generated service includes
#include "graph_navigation/srv/graph_nav.hpp"
#include "graph_navigation/srv/social_nav.hpp"

// Internal includes
#include "constant_curvature_arcs.h"
#include "motion_primitives.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/math/line2d.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "social_nav.h"

// System includes
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using namespace std::chrono_literals;

// Found in Config File
CONFIG_STRING(laser_topic, "NavigationParameters.laser_topic");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(loc_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");
CONFIG_FLOAT(laser_loc_x, "NavigationParameters.laser_loc.x");
CONFIG_FLOAT(laser_loc_y, "NavigationParameters.laser_loc.y");
CONFIG_FLOAT(dt, "NavigationParameters.dt");
CONFIG_FLOAT(max_linear_accel, "NavigationParameters.max_linear_accel");
CONFIG_FLOAT(max_linear_decel, "NavigationParameters.max_linear_decel");
CONFIG_FLOAT(max_linear_speed, "NavigationParameters.max_linear_speed");
CONFIG_FLOAT(max_angular_accel, "NavigationParameters.max_angular_accel");
CONFIG_FLOAT(max_angular_decel, "NavigationParameters.max_angular_decel");
CONFIG_FLOAT(max_angular_speed, "NavigationParameters.max_angular_speed");
CONFIG_FLOAT(carrot_dist, "NavigationParameters.carrot_dist");
CONFIG_FLOAT(system_latency, "NavigationParameters.system_latency");
CONFIG_FLOAT(obstacle_margin, "NavigationParameters.obstacle_margin");
CONFIG_INT(num_options, "NavigationParameters.num_options");
CONFIG_FLOAT(robot_width, "NavigationParameters.robot_width");
CONFIG_FLOAT(robot_length, "NavigationParameters.robot_length");
CONFIG_FLOAT(base_link_offset, "NavigationParameters.base_link_offset");
CONFIG_FLOAT(max_free_path_length, "NavigationParameters.max_free_path_length");
CONFIG_FLOAT(max_clearance, "NavigationParameters.max_clearance");
CONFIG_BOOL(can_traverse_stairs, "NavigationParameters.can_traverse_stairs");
CONFIG_FLOAT(intermediate_goal_dist, "NavigationParameters.intermediate_goal_dist");
CONFIG_BOOL(use_map_speed, "NavigationParameters.use_map_speed");
CONFIG_FLOAT(target_dist_tolerance, "NavigationParameters.target_dist_tolerance");
CONFIG_FLOAT(target_vel_tolerance, "NavigationParameters.target_vel_tolerance");
CONFIG_FLOAT(target_angle_tolerance, "NavigationParameters.target_angle_tolerance");
CONFIG_FLOAT(local_fov, "NavigationParameters.local_fov");
CONFIG_BOOL(use_kinect, "NavigationParameters.use_kinect");
CONFIG_STRING(model_path, "NavigationParameters.model_path");
CONFIG_STRING(evaluator_type, "NavigationParameters.evaluator_type");
CONFIG_FLOAT(local_costmap_resolution, "NavigationParameters.local_costmap_resolution");
CONFIG_FLOAT(max_inflation_radius, "NavigationParameters.max_inflation_radius");
CONFIG_FLOAT(local_costmap_size, "NavigationParameters.local_costmap_size");
CONFIG_FLOAT(min_inflation_radius, "NavigationParameters.min_inflation_radius");
CONFIG_FLOAT(global_costmap_resolution, "NavigationParameters.global_costmap_resolution");
CONFIG_FLOAT(global_costmap_size_x, "NavigationParameters.global_costmap_size_x");
CONFIG_FLOAT(global_costmap_size_y, "NavigationParameters.global_costmap_size_y");
CONFIG_FLOAT(global_costmap_origin_x, "NavigationParameters.global_costmap_origin_x");
CONFIG_FLOAT(global_costmap_origin_y, "NavigationParameters.global_costmap_origin_y");
CONFIG_FLOAT(lidar_range_min, "NavigationParameters.lidar_range_min");
CONFIG_FLOAT(lidar_range_max, "NavigationParameters.lidar_range_max");
CONFIG_FLOAT(replan_dist, "NavigationParameters.replan_dist");
CONFIG_FLOAT(object_lifespan, "NavigationParameters.object_lifespan");
CONFIG_FLOAT(inflation_coeff, "NavigationParameters.inflation_coeff");
CONFIG_FLOAT(distance_weight, "NavigationParameters.distance_weight");
CONFIG_FLOAT(recovery_carrot_dist, "NavigationParameters.recovery_carrot_dist");

// Add these macros after the other CONFIG_STRING macros
CONFIG_STRING(ackermann_drive_topic, "NavigationParameters.ackermann_drive_topic");
CONFIG_STRING(visualization_topic, "NavigationParameters.visualization_topic");
CONFIG_STRING(visualization_marker_topic, "NavigationParameters.visualization_marker_topic");
CONFIG_STRING(simulator_visualization_topic, "NavigationParameters.simulator_visualization_topic");
CONFIG_STRING(human_viz_topic, "NavigationParameters.human_viz_topic");
CONFIG_STRING(human_states_topic, "NavigationParameters.human_states_topic");
CONFIG_STRING(set_goal_topic, "NavigationParameters.set_goal_topic");
CONFIG_STRING(robot_cmd_vel_topic, "NavigationParameters.robot_cmd_vel_topic");
CONFIG_STRING(graph_nav_service, "NavigationParameters.graph_nav_service");
CONFIG_STRING(social_nav_service, "NavigationParameters.social_nav_service");
CONFIG_STRING(social_pips_service, "NavigationParameters.social_pips_service");

// Command Line Flags
DEFINE_bool(service_mode, false, "Listen to a service instead of topics.");
DEFINE_bool(social_mode, true, "Enable social navigation behaviors.");
DEFINE_bool(bag_mode, false, "Run in bag playback mode.");
DEFINE_string(topic_prefix, "", "Prefix for robot id.");
DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");
DEFINE_string(maps_dir, "", "Directory containing AMRL maps");
DEFINE_string(map, "GDC1", "Name of navigation map file");
DEFINE_string(robot_config, "config/gym_nav.lua", "Path to config file");
DEFINE_double(dt, 0.025, "Delta T");

class SocialNavigationNode : public rclcpp::Node, public std::enable_shared_from_this<SocialNavigationNode> {
   public:
    SocialNavigationNode() : Node("social_navigation"),
                             run_(true),
                             enabled_(true),
                             r_loc_(false),
                             r_odom_(false),
                             r_humans_(false),
                             r_laser_(false),
                             goal_set_(false),
                             current_action_(0),
                             follow_target_(0),
                             current_angle_(0.0),
                             current_angular_vel_(0.0),
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

        // Initialize social navigation
        navigation_ = std::make_unique<navigation::SocialNav>();

        // Load map
        std::string map_path = navigation::GetMapPath(FLAGS_maps_dir, FLAGS_map);
        if (!FileExists(map_path)) {
            RCLCPP_ERROR(this->get_logger(), "Could not find navigation map file at %s", map_path.c_str());
            throw std::runtime_error("Map file not found");
        }

        // Load configuration
        navigation::NavigationParameters params;
        LoadConfig(&params);
        navigation_->GetGraphNav()->Initialize(params, map_path);

        if (FLAGS_bag_mode) {
            // Load additional map for visualization if needed
            // map_.Load(map_path);
            DrawMap();
        }

        // Initialize visualization messages
        local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
        global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");

        // Create publishers
        ackermann_drive_pub_ = this->create_publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>(
            CONFIG_ackermann_drive_topic, 1);
        twist_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            FLAGS_twist_drive_topic, 1);
        viz_pub_ = this->create_publisher<amrl_msgs::msg::VisualizationMsg>(
            CONFIG_visualization_topic, 1);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            CONFIG_visualization_marker_topic, 1);
        map_lines_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            CONFIG_simulator_visualization_topic, 6);
        pose_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            CONFIG_simulator_visualization_topic, 6);
        human_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            CONFIG_human_viz_topic, 6);

        // Create services
        plan_service_ = this->create_service<graph_navigation::srv::GraphNav>(
            CONFIG_graph_nav_service, std::bind(&SocialNavigationNode::PlanService, this,
                                                std::placeholders::_1, std::placeholders::_2));
        social_service_ = this->create_service<graph_navigation::srv::SocialNav>(
            CONFIG_social_nav_service, std::bind(&SocialNavigationNode::SocialService, this,
                                                 std::placeholders::_1, std::placeholders::_2));

        // Create service client for PIPS
        pips_client_ = this->create_client<amrl_msgs::srv::SocialPipsSrv>(CONFIG_social_pips_service);

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            FLAGS_topic_prefix + CONFIG_odom_topic, 1,
            std::bind(&SocialNavigationNode::OdometryCallback, this, std::placeholders::_1));

        localization_sub_ = this->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            FLAGS_topic_prefix + CONFIG_loc_topic, 1,
            std::bind(&SocialNavigationNode::LocalizationCallback, this, std::placeholders::_1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            FLAGS_topic_prefix + CONFIG_laser_topic, 1,
            std::bind(&SocialNavigationNode::LaserCallback, this, std::placeholders::_1));

        human_sub_ = this->create_subscription<amrl_msgs::msg::HumanStateArrayMsg>(
            CONFIG_human_states_topic, 1,
            std::bind(&SocialNavigationNode::HumanCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<amrl_msgs::msg::Pose2Df>(
            CONFIG_set_goal_topic, 1,
            std::bind(&SocialNavigationNode::GoalCallback, this, std::placeholders::_1));

        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            CONFIG_robot_cmd_vel_topic, 1,
            std::bind(&SocialNavigationNode::VelocityCallback, this, std::placeholders::_1));

        // Initialize visualization markers
        InitSimulatorVizMarkers();

        // Create timer for main loop (only if not in service mode)
        if (!FLAGS_service_mode) {
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(FLAGS_dt),
                std::bind(&SocialNavigationNode::TimerCallback, this));
        }

        RCLCPP_INFO(this->get_logger(), "Social navigation node initialized");
    }

    ~SocialNavigationNode() {
        run_ = false;
    }

   private:
    // Navigation components
    std::unique_ptr<navigation::SocialNav> navigation_;
    vector_map::VectorMap map_;

    // Publishers
    rclcpp::Publisher<amrl_msgs::msg::AckermannCurvatureDriveMsg>::SharedPtr ackermann_drive_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_drive_pub_;
    rclcpp::Publisher<amrl_msgs::msg::VisualizationMsg>::SharedPtr viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_lines_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr human_marker_publisher_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr localization_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<amrl_msgs::msg::HumanStateArrayMsg>::SharedPtr human_sub_;
    rclcpp::Subscription<amrl_msgs::msg::Pose2Df>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

    // Services
    rclcpp::Service<graph_navigation::srv::GraphNav>::SharedPtr plan_service_;
    rclcpp::Service<graph_navigation::srv::SocialNav>::SharedPtr social_service_;
    rclcpp::Client<amrl_msgs::srv::SocialPipsSrv>::SharedPtr pips_client_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    bool run_;
    bool enabled_;
    bool r_loc_;
    bool r_odom_;
    bool r_humans_;
    bool r_laser_;
    bool goal_set_;
    int current_action_;
    int follow_target_;

    std::vector<Eigen::Vector2f> point_cloud_;
    Eigen::Vector2f current_loc_{0, 0};
    float current_angle_;
    Eigen::Vector2f current_vel_{0, 0};
    float current_angular_vel_;
    navigation::Odom odom_;
    Eigen::Vector2f goal_{0, 0};
    float goal_angle_;
    std::vector<navigation::Human> humans_;

    // Visualization
    amrl_msgs::msg::VisualizationMsg local_viz_msg_;
    amrl_msgs::msg::VisualizationMsg global_viz_msg_;
    visualization_msgs::msg::Marker line_list_marker_;
    visualization_msgs::msg::Marker pose_marker_;
    visualization_msgs::msg::Marker human_marker_;
    visualization_msgs::msg::MarkerArray human_array_;
    visualization_msgs::msg::Marker target_marker_;

    // Callback functions
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        r_odom_ = true;
        odom_ = OdomHandler(*msg);
    }

    void LocalizationCallback(const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
        r_loc_ = true;
        current_loc_ = Eigen::Vector2f(msg->pose.x, msg->pose.y);
        current_angle_ = msg->pose.theta;
    }

    void LaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        r_laser_ = true;
        LaserHandler(*msg);
    }

    void HumanCallback(const amrl_msgs::msg::HumanStateArrayMsg::SharedPtr msg) {
        r_humans_ = true;
        humans_.clear();
        for (const auto& human_msg : msg->human_states) {
            navigation::Human human;
            human.pose = Eigen::Vector2f(human_msg.pose.x, human_msg.pose.y);
            human.vel = Eigen::Vector2f(human_msg.translational_velocity.x, human_msg.translational_velocity.y);
            human.id = human_msg.id;
            // If you need rotational_velocity, access human_msg.rotational_velocity
            humans_.push_back(human);
        }
    }

    void GoalCallback(const amrl_msgs::msg::Pose2Df::SharedPtr msg) {
        goal_ = Eigen::Vector2f(msg->x, msg->y);
        goal_angle_ = msg->theta;
        goal_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal set: (%f, %f) %f°", goal_.x(), goal_.y(), goal_angle_);
    }

    void VelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_vel_ = Eigen::Vector2f(msg->linear.x, msg->linear.y);
        current_angular_vel_ = msg->angular.z;
    }

    void TimerCallback() {
        if (!run_) return;

        if (goal_set_) {
            if (FLAGS_bag_mode) {
                RunBagfile();
            } else {
                RunSocial();
            }
        }

        PublishVisualizationMarkers();
        viz_pub_->publish(local_viz_msg_);
        viz_pub_->publish(global_viz_msg_);
    }

    // Service callbacks
    void PlanService(const std::shared_ptr<graph_navigation::srv::GraphNav::Request> request,
                     std::shared_ptr<graph_navigation::srv::GraphNav::Response> response) {
        const Eigen::Vector2f start(request->start.x, request->start.y);
        const Eigen::Vector2f end(request->end.x, request->end.y);
        const std::vector<int> plan = navigation_->GetGraphNav()->GlobalPlan(start, end);
        response->plan = plan;
    }

    void SocialService(const std::shared_ptr<graph_navigation::srv::SocialNav::Request> request,
                       std::shared_ptr<graph_navigation::srv::SocialNav::Response> response) {
        visualization::ClearVisualizationMsg(local_viz_msg_);
        navigation::SocialAction action = navigation::SocialAction::GoAlone;

        if (request->action == 1) {
            action = navigation::SocialAction::Halt;
        } else if (request->action == 2) {
            action = navigation::SocialAction::Follow;
        } else if (request->action == 3) {
            action = navigation::SocialAction::Pass;
        }

        navigation::Odom odom = OdomHandler(request->odom);
        odom.time += FLAGS_dt;

        Eigen::Vector2f cmd_vel;
        float cmd_angle_vel;
        navigation_->SetNavGoal({request->goal_pose.x, request->goal_pose.y}, request->goal_pose.theta);
        LaserHandler(request->laser);
        navigation_->Run(this->get_clock()->now().seconds(),
                         action,
                         {request->loc.x, request->loc.y},
                         request->loc.theta,
                         odom,
                         point_cloud_,
                         ToHumans(request->human_poses, request->human_vels),
                         cmd_vel, cmd_angle_vel);

        auto twist = GetTwistMsg(cmd_vel, cmd_angle_vel);
        auto ackermann = TwistToAckermann(twist);
        response->cmd_vel = ackermann->velocity;
        response->cmd_curve = ackermann->curvature;
        response->target_id = navigation_->GetTargetId();

        const Eigen::Vector2f local_target = navigation_->GetLocalTarget();
        response->local_target.x = local_target.x();
        response->local_target.y = local_target.y();

        SendCommand(cmd_vel, cmd_angle_vel);
        DrawRobot();
        DrawPathOptions();
        DrawTarget();
        viz_pub_->publish(local_viz_msg_);
        viz_pub_->publish(global_viz_msg_);
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

    void LaserHandler(const sensor_msgs::msg::LaserScan& msg) {
        point_cloud_.clear();
        for (size_t i = 0; i < msg.ranges.size(); ++i) {
            const float angle = msg.angle_min + static_cast<float>(i) * msg.angle_increment;
            const float range = ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max) ? msg.ranges[i] : msg.range_max);
            const float x = range * cos(angle) + CONFIG_laser_loc_x;
            const float y = range * sin(angle) + CONFIG_laser_loc_y;
            point_cloud_.emplace_back(x, y);
        }
    }

    std::vector<navigation::Human> ToHumans(const std::vector<geometry_msgs::msg::Pose2D>& poses,
                                            const std::vector<geometry_msgs::msg::Pose2D>& vels) {
        std::vector<navigation::Human> humans;
        for (size_t i = 0; i < poses.size() && i < vels.size(); ++i) {
            navigation::Human human;
            human.pose = Eigen::Vector2f(poses[i].x, poses[i].y);
            // If heading is needed, store as a separate variable or ignore
            human.vel = Eigen::Vector2f(vels[i].x, vels[i].y);
            human.id = static_cast<int>(i);
            humans.push_back(human);
        }
        return humans;
    }

    std::unique_ptr<geometry_msgs::msg::TwistStamped> GetTwistMsg(const Eigen::Vector2f& vel, float ang_vel) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->get_clock()->now();
        twist_msg->twist.linear.x = vel.x();
        twist_msg->twist.linear.y = vel.y();
        twist_msg->twist.angular.z = ang_vel;
        return twist_msg;
    }

    std::unique_ptr<amrl_msgs::msg::AckermannCurvatureDriveMsg> TwistToAckermann(
        const std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist) {
        auto ackermann_msg = std::make_unique<amrl_msgs::msg::AckermannCurvatureDriveMsg>();
        ackermann_msg->header = twist->header;
        ackermann_msg->velocity = twist->twist.linear.x;
        if (fabs(ackermann_msg->velocity) < 1e-6) {
            ackermann_msg->curvature = 0;
        } else {
            ackermann_msg->curvature = twist->twist.angular.z / ackermann_msg->velocity;
        }
        return ackermann_msg;
    }

    void SendCommand(const Eigen::Vector2f& vel, float ang_vel) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = vel.x();
        twist_msg->linear.y = vel.y();
        twist_msg->angular.z = ang_vel;
        twist_drive_pub_->publish(std::move(twist_msg));

        auto ackermann_msg = std::make_unique<amrl_msgs::msg::AckermannCurvatureDriveMsg>();
        ackermann_msg->header.stamp = this->get_clock()->now();
        ackermann_msg->velocity = vel.x();
        if (fabs(ackermann_msg->velocity) < 1e-6) {
            ackermann_msg->curvature = 0;
        } else {
            ackermann_msg->curvature = ang_vel / ackermann_msg->velocity;
        }
        ackermann_drive_pub_->publish(std::move(ackermann_msg));
    }

    template<typename T>
    void FillRequest(T& req) {
        req.action = current_action_;
        req.goal_pose.x = goal_.x();
        req.goal_pose.y = goal_.y();
        req.goal_pose.theta = goal_angle_;
        req.loc.x = current_loc_.x();
        req.loc.y = current_loc_.y();
        req.loc.theta = current_angle_;
        req.time = this->get_clock()->now().seconds();

        // Convert humans to request format
        req.human_poses.clear();
        req.human_vels.clear();
        for (const auto& human : humans_) {
            geometry_msgs::msg::Pose2D pose;
            pose.x = human.pose.x();
            pose.y = human.pose.y();
            pose.theta = 0.0;  // No heading in Pose2D
            req.human_poses.push_back(pose);

            geometry_msgs::msg::Pose2D vel;
            vel.x = human.vel.x();
            vel.y = human.vel.y();
            vel.theta = 0.0;
            req.human_vels.push_back(vel);
        }
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
        navigation::SocialAction action = navigation::SocialAction::GoAlone;

        if (FLAGS_social_mode) {
            // Note: PIPS service call would need proper service definition
            // For now, using default action
            current_action_ = 0;
            action = navigation::SocialAction::GoAlone;
        }

        Eigen::Vector2f cmd_vel;
        float cmd_angle_vel;
        navigation_->SetNavGoal({goal_.x(), goal_.y()}, goal_angle_);
        navigation_->Run(this->get_clock()->now().seconds(),
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
        viz_pub_->publish(local_viz_msg_);
        viz_pub_->publish(global_viz_msg_);
    }

    void RunBagfile() {
        if (Synced()) {
            RunSocial();
        }
    }

    void DrawRobot() {
        // Implementation would be similar to navigation_main.cc
        const float robot_width = 0.44f;
        const float robot_length = 0.5f;
        const float rear_axle_offset = 0.0f;

        const float l1 = -0.5f * robot_length - rear_axle_offset;
        const float l2 = 0.5f * robot_length - rear_axle_offset;
        const float w = 0.5f * robot_width;

        visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l1, -w), 0x000000, local_viz_msg_);
        visualization::DrawLine(Eigen::Vector2f(l2, w), Eigen::Vector2f(l2, -w), 0x000000, local_viz_msg_);
        visualization::DrawLine(Eigen::Vector2f(l1, w), Eigen::Vector2f(l2, w), 0x000000, local_viz_msg_);
        visualization::DrawLine(Eigen::Vector2f(l1, -w), Eigen::Vector2f(l2, -w), 0x000000, local_viz_msg_);
    }

    void DrawPathOptions() {
        // Draw the path options from the social navigation planner
        auto path_rollouts = navigation_->GetGraphNav()->GetLastPathOptions();
        auto best_option = navigation_->GetGraphNav()->GetOption();

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

    void DrawTarget() {
        const Eigen::Vector2f target = navigation_->GetLocalTarget();
        visualization::DrawCross(target, 0.2, 0xFF0080, local_viz_msg_);
    }

    void DrawMap() {
        // Draw map lines for visualization if needed
        RCLCPP_INFO(this->get_logger(), "Map visualization initialized");
    }

    void InitSimulatorVizMarkers() {
        // Initialize visualization markers
        RCLCPP_INFO(this->get_logger(), "Visualization markers initialized");
    }

    void PublishVisualizationMarkers() {
        // Publish visualization markers for humans and other objects
        human_array_.markers.clear();
        for (size_t i = 0; i < humans_.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "humans";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = humans_[i].pose.x();
            marker.pose.position.y = humans_[i].pose.y();
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1.8;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

            human_array_.markers.push_back(marker);
        }
        human_marker_publisher_->publish(human_array_);
    }

    void LoadConfig(navigation::NavigationParameters* params) {
        config_reader::ConfigReader reader({FLAGS_robot_config});
        params->dt = CONFIG_dt;
        params->linear_limits = navigation::MotionLimits(
            CONFIG_max_linear_accel, CONFIG_max_linear_decel, CONFIG_max_linear_speed);
        params->angular_limits = navigation::MotionLimits(
            CONFIG_max_angular_accel, CONFIG_max_angular_decel, CONFIG_max_angular_speed);
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
        params->intermediate_goal_dist = CONFIG_intermediate_goal_dist;
        params->use_map_speed = CONFIG_use_map_speed;
        params->target_dist_tolerance = CONFIG_target_dist_tolerance;
        params->target_vel_tolerance = CONFIG_target_vel_tolerance;
        params->target_angle_tolerance = CONFIG_target_angle_tolerance;
        params->local_fov = CONFIG_local_fov;
        params->use_kinect = CONFIG_use_kinect;
        params->model_path = CONFIG_model_path;
        params->evaluator_type = CONFIG_evaluator_type;
        params->local_costmap_resolution = CONFIG_local_costmap_resolution;
        params->max_inflation_radius = CONFIG_max_inflation_radius;
        params->local_costmap_size = CONFIG_local_costmap_size;
        params->min_inflation_radius = CONFIG_min_inflation_radius;
        params->global_costmap_resolution = CONFIG_global_costmap_resolution;
        params->global_costmap_size_x = CONFIG_global_costmap_size_x;
        params->global_costmap_size_y = CONFIG_global_costmap_size_y;
        params->global_costmap_origin_x = CONFIG_global_costmap_origin_x;
        params->global_costmap_origin_y = CONFIG_global_costmap_origin_y;
        params->lidar_range_min = CONFIG_lidar_range_min;
        params->lidar_range_max = CONFIG_lidar_range_max;
        params->replan_dist = CONFIG_replan_dist;
        params->object_lifespan = CONFIG_object_lifespan;
        params->inflation_coeff = CONFIG_inflation_coeff;
        params->distance_weight = CONFIG_distance_weight;
        params->recovery_carrot_dist = CONFIG_recovery_carrot_dist;
    }
};

void SignalHandler(int) {
    RCLCPP_INFO(rclcpp::get_logger("social_navigation"), "Shutting down social navigation node");
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
        auto node = std::make_shared<SocialNavigationNode>();
        if (FLAGS_service_mode) {
            rclcpp::spin(node);
        } else {
            rclcpp::spin(node);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("social_navigation"), "Exception in social navigation node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
