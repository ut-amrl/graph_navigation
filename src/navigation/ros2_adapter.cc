#ifndef ROS1

#include "ros_adapter.h"

// ROS 2 includes
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/gps_array_msg.hpp"
#include "amrl_msgs/msg/gps_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/mission_status_msg.hpp"
#include "amrl_msgs/msg/nav_status_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "amrl_msgs/srv/graph_nav_gps_srv.hpp"
#include "foxglove_msgs/msg/geo_json.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Navigation includes
#include "config_reader/config_reader.h"
#include "constant_curvature_arcs.h"
#include "motion_primitives.h"
#include "navigation.h"
#include "navigation_flags.h"
#include "navigation_types.h"
#include "shared/math/geometry.h"
#include "shared/math/gps_util.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/ros/ros_macros.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "visualization/ros_visualization.h"
#include "visualization/visualization.h"

// Built in Libraries
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Third Party Libraries
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "gflags/gflags.h"

using namespace std;
using namespace std::chrono;
using rclcpp::Node;
using rclcpp::Time;
using namespace Eigen;

using ros_helpers::InitRosHeader;

// Aliases for custom datatypes
using gps_util::GPSPoint;
using motion_primitives::ConstantCurvatureArc;
using motion_primitives::PathRolloutBase;
using navigation::PathOption;

// Aliases for ROS2 messages
using AckermannCurvatureDriveMsg = amrl_msgs::msg::AckermannCurvatureDriveMsg;
using GPSArrayMsg = amrl_msgs::msg::GPSArrayMsg;
using GPSMsg = amrl_msgs::msg::GPSMsg;
using Localization2DMsg = amrl_msgs::msg::Localization2DMsg;
using MissionStatusMsg = amrl_msgs::msg::MissionStatusMsg;
using NavStatusMsg = amrl_msgs::msg::NavStatusMsg;
using Pose2Df = amrl_msgs::msg::Pose2Df;
using VisualizationMsg = amrl_msgs::msg::VisualizationMsg;
using GraphNavGPSSrv = amrl_msgs::srv::GraphNavGPSSrv;

using geometry::kEpsilon;

using foxglove_msgs::msg::GeoJSON;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud;
using std_msgs::msg::Bool;
using std_msgs::msg::Empty;
using std_msgs::msg::Float64MultiArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using std::string;
using std::unordered_map;
using std::vector;

// Configuration macros from config_reader (assumed to be defined)
CONFIG_STRING(image_topic, "NavigationParameters.image_topic");
CONFIG_STRINGLIST(laser_topics, "NavigationParameters.laser_topics");
CONFIG_STRING(laser_frame, "NavigationParameters.laser_frame");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(localization_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(gps_topic, "OSMPlannerParameters.gps_topic");
CONFIG_STRING(gps_goals_topic, "OSMPlannerParameters.gps_goals_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");

// Convenience: convert rclcpp::Time to seconds.
inline double to_seconds(const rclcpp::Time &t) { return t.seconds(); }

namespace navigation {

vector<PathOption> ToOptions(vector<std::shared_ptr<PathRolloutBase>> paths) {
  vector<PathOption> options;
  for (size_t i = 0; i < paths.size(); ++i) {
    const ConstantCurvatureArc arc =
        *reinterpret_cast<ConstantCurvatureArc *>(paths[i].get());
    PathOption option;
    option.curvature = arc.curvature;
    option.free_path_length = arc.Length();
    option.clearance = arc.Clearance();
    options.push_back(option);
  }
  return options;
}

navigation::Odom OdomHandler(const Odometry::SharedPtr &msg) {
  navigation::Odom odom;
  odom.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  odom.position = {static_cast<float>(msg->pose.pose.position.x),
                   static_cast<float>(msg->pose.pose.position.y),
                   static_cast<float>(msg->pose.pose.position.z)};
  odom.orientation = {static_cast<float>(msg->pose.pose.orientation.w),
                      static_cast<float>(msg->pose.pose.orientation.x),
                      static_cast<float>(msg->pose.pose.orientation.y),
                      static_cast<float>(msg->pose.pose.orientation.z)};
  return odom;
}

AckermannCurvatureDriveMsg TwistToAckermann(const TwistStamped &twist) {
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

//============================================================================
// Ros2AdapterImpl: Implements the RosAdapter interface for ROS2.
//============================================================================
class Ros2AdapterImpl : public RosAdapter {
 public:
  explicit Ros2AdapterImpl(rclcpp::Node::SharedPtr node,
                           const navigation::NavigationParameters &params)
      : node_(node), params_(params) {}

  // Initialize ROS2: create the node, publishers, subscribers, services.
  void Initialize(std::shared_ptr<Navigation> navigation) override {
    navigation_ = navigation;

    setupPublishers();
    setupSubscriptions();
    setupServices();
  }

  // Spin loop: run the ROS event loop and periodically update navigation.
  void spinLoop() override {
    rclcpp::WallRate rate(1.0 / params_.dt);

    while (rclcpp::ok()) {
      visualization::ClearVisualizationMsg(local_viz_msg_);
      visualization::ClearVisualizationMsg(global_viz_msg_);

      // Process pending callbacks
      rclcpp::spin_some(node_);

      auto now = GET_TIME();

      Vector2f cmd_vel(0, 0);
      float cmd_angle_vel = 0;

      bool nav_succeeded = false;
      if (auto nav = navigation_.lock()) {
        nav_succeeded = nav->Run(now.seconds(), cmd_vel, cmd_angle_vel);
      }

      if (auto nav = navigation_.lock()) {
        if (!FLAGS_simulate) {
          PublishTF();
        }
        PublishNavStatus();
        PublishMissionStatus();
        PublishLocalization();
        PublishGlobalPlan();

        if (nav_succeeded) {
          PublishForwardPredictedPCL(nav->GetPredictedCloud());
          DrawRobot();
          if (nav->GetNavStatusUint8() !=
              static_cast<uint8_t>(NavigationState::kStopped)) {
            DrawTarget();
            DrawPathOptions();
          }
          PublishVisualizationMarkers();
          PublishPath();
          PublishNextGPSGoal();

          // Update message headers with current time.
          now = GET_TIME();
          local_viz_msg_.header.stamp = now;
          global_viz_msg_.header.stamp = now;
          viz_pub_->publish(local_viz_msg_);
          viz_pub_->publish(global_viz_msg_);

          // Optionally, if using a cost map evaluator, get and publish
          if (params_.evaluator_type == "cost_map" ||
              params_.evaluator_type == "cost_map_service" ||
              params_.evaluator_type == "terrain2") {
            cv_bridge::CvImage viz_img, bev_viz_img;
            bool result =
                nav->GetVisualizationImage(viz_img.image, bev_viz_img.image);
            if (result) {
              if (!viz_img.image.empty()) {
                viz_img.header.stamp = now;
                viz_img.encoding =
                    (params_.evaluator_type == "cost_map_service")
                        ? sensor_msgs::image_encodings::BGRA8
                        : sensor_msgs::image_encodings::BGR8;
                viz_img_pub_->publish(*viz_img.toImageMsg());
              }
              if (!bev_viz_img.image.empty()) {
                bev_viz_img.header.stamp = now;
                bev_viz_img.encoding =
                    (params_.evaluator_type == "cost_map_service")
                        ? sensor_msgs::image_encodings::BGRA8
                        : sensor_msgs::image_encodings::BGR8;
                viz_bev_img_pub_->publish(*bev_viz_img.toImageMsg());
              }
            }
          }
          // Finally, send the computed command.
          SendCommand(cmd_vel, cmd_angle_vel);
        }
      }
      rate.sleep();
    }
  }

 private:
  navigation::Odom odom_;
  vector<Vector2f> point_cloud_;
  cv::Mat last_image_;
  GPSPoint gps_goal_;
  bool gps_goal_updated_ = false;

  void PublishTF() {
    static tf2_ros::TransformBroadcaster tf_broadcaster_(node_);

    if (auto nav = navigation_.lock()) {
      navigation::Odom odom;
      GPSPoint gps_loc;
      if (!nav->GetInitialOdom(odom) || !nav->GetInitialGPS(gps_loc)) return;
      auto T_odom_map = nav->OdometryToUTMTransform(odom, gps_loc);
      Vector2f translation_2d = T_odom_map.translation().head<2>();
      float theta = atan2(T_odom_map.linear()(1, 0), T_odom_map.linear()(0, 0));
      TransformStamped transform_msg;
      transform_msg.header.stamp = node_->now();
      transform_msg.header.frame_id = "map";
      transform_msg.child_frame_id = "odom";
      transform_msg.transform.translation.x = translation_2d.x();
      transform_msg.transform.translation.y = translation_2d.y();
      transform_msg.transform.translation.z = 0.0;
      // Use tf2 to create a quaternion.
      tf2::Quaternion q;
      q.setRPY(0, 0, theta);
      transform_msg.transform.rotation.x = q.x();
      transform_msg.transform.rotation.y = q.y();
      transform_msg.transform.rotation.z = q.z();
      transform_msg.transform.rotation.w = q.w();
      // Broadcast the transform.
      tf_broadcaster_.sendTransform(transform_msg);
    }
  }

  void PublishNavStatus() {
    if (auto nav = navigation_.lock()) {
      NavStatusMsg status;
      status.header.stamp = node_->now();
      status.status = nav->GetNavStatusUint8();
      status_pub_->publish(status);
    }
  }

  void PublishMissionStatus() {
    if (auto nav = navigation_.lock()) {
      MissionStatusMsg status_msg;
      auto missionStatus = nav->GetMissionStatus();
      status_msg.header.stamp = rclcpp::Time(missionStatus.time);
      status_msg.status = missionStatus.status;
      status_msg.mission_id = missionStatus.mission_id;
      status_msg.next_goal_id = missionStatus.next_goal_id;
      for (size_t i = 0; i < missionStatus.goals.size(); ++i) {
        GPSMsg goal_msg;
        goal_msg.header.stamp = rclcpp::Time(missionStatus.goals[i].time);
        goal_msg.latitude = missionStatus.goals[i].lat;
        goal_msg.longitude = missionStatus.goals[i].lon;
        status_msg.goals.data.push_back(goal_msg);
        if (i < missionStatus.goals_reached.size()) {
          GPSMsg goal_reached_msg;
          goal_reached_msg.header.stamp =
              rclcpp::Time(missionStatus.goals_reached[i].time);
          goal_reached_msg.latitude = missionStatus.goals_reached[i].lat;
          goal_reached_msg.longitude = missionStatus.goals_reached[i].lon;
          status_msg.goals_reached.data.push_back(goal_reached_msg);
        }
      }
      mission_status_pub_->publish(status_msg);
    }
  }

  void PublishLocalization() {
    if (auto nav = navigation_.lock()) {
      Eigen::Vector3f robot_pose;
      if (!nav->GetRobotPose(robot_pose)) return;
      Localization2DMsg loc_msg;
      loc_msg.header.stamp = node_->now();
      loc_msg.pose.x = robot_pose.x();
      loc_msg.pose.y = robot_pose.y();
      loc_msg.pose.theta = robot_pose.z();
      localization_pub_->publish(loc_msg);
    }
  }

  void PublishGlobalPlan() {
    if (auto nav = navigation_.lock()) {
      vector<GPSPoint> plan;
      if (nav->GetGlobalPlan(plan)) {
        // Publish global plan on vectormap
        global_viz_msg_.lines.clear();
        auto map_route = nav->GPSRouteToMap(plan);
        for (const auto &p : map_route) {
          visualization::DrawPoint(p.cast<float>(), 0xFF0000, global_viz_msg_);
        }
        viz_pub_->publish(global_viz_msg_);

        // Publish global plan on foxglove
        ros_visualization::GPSRouteToGeoJSON(geojson_pub_, plan);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Global plan is not valid.");
      }
    }
  }

  void PublishForwardPredictedPCL(const vector<Vector2f> &pcl) {
    PointCloud fp_pcl_msg;
    fp_pcl_msg.points.resize(pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i) {
      fp_pcl_msg.points[i].x = pcl[i].x();
      fp_pcl_msg.points[i].y = pcl[i].y();
      fp_pcl_msg.points[i].z = 0.324f;
    }
    fp_pcl_msg.header.stamp = node_->now();
    fp_pcl_pub_->publish(fp_pcl_msg);
  }

  void DrawRobot() {
    if (auto nav = navigation_.lock()) {
      float kRobotLength = nav->GetRobotLength();
      float kRobotWidth = nav->GetRobotWidth();
      float kRearAxleOffset = 0.0f;
      float kObstacleMargin = nav->GetObstacleMargin();
      float l1 = -0.5f * kRobotLength - kRearAxleOffset - kObstacleMargin;
      float l2 = 0.5f * kRobotLength - kRearAxleOffset + kObstacleMargin;
      float w = 0.5f * kRobotWidth + kObstacleMargin;
      visualization::DrawLine(Vector2f(l1, w), Vector2f(l1, -w), 0xC0C0C0,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l2, w), Vector2f(l2, -w), 0xC0C0C0,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l1, w), Vector2f(l2, w), 0xC0C0C0,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l1, -w), Vector2f(l2, -w), 0xC0C0C0,
                              local_viz_msg_);

      l1 = -0.5f * kRobotLength - kRearAxleOffset;
      l2 = 0.5f * kRobotLength - kRearAxleOffset;
      w = 0.5f * kRobotWidth;
      visualization::DrawLine(Vector2f(l1, w), Vector2f(l1, -w), 0x000000,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l2, w), Vector2f(l2, -w), 0x000000,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l1, w), Vector2f(l2, w), 0x000000,
                              local_viz_msg_);
      visualization::DrawLine(Vector2f(l1, -w), Vector2f(l2, -w), 0x000000,
                              local_viz_msg_);
    }
  }

  void DrawTarget() {
    if (auto nav = navigation_.lock()) {
      float carrot_dist = nav->GetCarrotDist();
      Vector2f target = nav->GetTarget();
      visualization::DrawCross(nav->GetIntermediateGoal(), 0.2f, 0x0000FF,
                               global_viz_msg_);
      visualization::DrawArc(Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0,
                             local_viz_msg_);
      viz_pub_->publish(global_viz_msg_);
      visualization::DrawCross(target, 0.2f, 0xFF0080, local_viz_msg_);
    }
  }

  void DrawPathOptions() {
    if (auto nav = navigation_.lock()) {
      auto path_rollouts = nav->GetLastPathOptions();
      if (path_rollouts.empty()) return;
      auto path_options = navigation::ToOptions(path_rollouts);
      auto best_option = nav->GetOption();
      for (const auto &o : path_options) {
        visualization::DrawPathOption(o.curvature, o.free_path_length,
                                      o.clearance, 0x0000FF, false,
                                      local_viz_msg_);
      }
      if (best_option != nullptr) {
        auto best_option_as_option = navigation::ToOptions({best_option})[0];
        path_options.insert(path_options.begin(), best_option_as_option);
      }
      vector<vector<float>> colors(path_options.size(),
                                   {0.0f, 0.0f, 1.0f, 1.0f});
      colors[0] = {1.0f, 0.0f, 0.0f, 1.0f};
      ros_visualization::PathOptionToMarkerArray(fox_path_pub_, "base_link",
                                                 path_options, colors, false);
      if (best_option != nullptr) {
        ConstantCurvatureArc best_arc =
            *reinterpret_cast<ConstantCurvatureArc *>(best_option.get());
        visualization::DrawPathOption(best_arc.curvature, best_arc.length,
                                      best_arc.clearance, 0xFF0000, true,
                                      local_viz_msg_);
      }
    }
  }

  void PublishVisualizationMarkers() {
    static Vector2f current_loc = {0.0f, 0.0f};
    static float current_angle = 0.0f;
    map_lines_publisher_->publish(line_list_marker_);
    tf2::Quaternion robotQ;
    robotQ.setRPY(0, 0, current_angle);

    pose_marker_.header.stamp = node_->now();
    pose_marker_.header.frame_id = "base_link";
    pose_marker_.pose.position.x = current_loc.x() - cos(current_angle) * 0.0f;
    pose_marker_.pose.position.y = current_loc.y() - sin(current_angle) * 0.0f;
    pose_marker_.pose.position.z = 0.25f;
    pose_marker_.pose.orientation.x = robotQ.x();
    pose_marker_.pose.orientation.y = robotQ.y();
    pose_marker_.pose.orientation.z = robotQ.z();
    pose_marker_.pose.orientation.w = robotQ.w();
    pose_marker_publisher_->publish(pose_marker_);
  }

  // Helper to convert a carrot vector to a PoseStamped message.
  PoseStamped CarrotToPoseStamped(const Vector2f &carrot) {
    PoseStamped carrotPose;
    carrotPose.header.stamp = node_->now();
    carrotPose.header.frame_id = "base_link";
    carrotPose.pose.position.x = carrot.x();
    carrotPose.pose.position.y = carrot.y();
    carrotPose.pose.position.z = 0.0;
    carrotPose.pose.orientation.x = 0;
    carrotPose.pose.orientation.y = 0;
    carrotPose.pose.orientation.z = 0;
    carrotPose.pose.orientation.w = 1;
    return carrotPose;
  }

  void PublishPath() {
    if (auto nav = navigation_.lock()) {
      auto path = nav->GetPlanPath();
      if (path.size() >= 2) {
        Path path_msg;
        path_msg.header.stamp = node_->now();
        path_msg.header.frame_id = "map";
        for (size_t i = 0; i < path.size(); i++) {
          PoseStamped pose_plan;
          pose_plan.pose.position.x = path[i].loc.x();
          pose_plan.pose.position.y = path[i].loc.y();
          pose_plan.pose.orientation.x = 0;
          pose_plan.pose.orientation.y = 0;
          pose_plan.pose.orientation.z = 0;
          pose_plan.pose.orientation.w = 1;
          pose_plan.header.stamp = node_->now();
          pose_plan.header.frame_id = "map";
          path_msg.poses.push_back(pose_plan);
          path_pub_->publish(path_msg);
        }
        for (size_t i = 1; i < path.size(); i++) {
          visualization::DrawLine(path[i - 1].loc, path[i].loc, 0x007F00,
                                  global_viz_msg_);
        }
        auto global_path = nav->GetGlobalPath();
        for (size_t i = 1; i < global_path.size(); i++) {
          visualization::DrawLine(global_path[i - 1].loc, global_path[i].loc,
                                  0xA86032, global_viz_msg_);
        }
        Vector2f carrot;
        bool foundCarrot = nav->GetLocalCarrotHeading(carrot, false);
        if (foundCarrot) {
          carrot_pub_->publish(CarrotToPoseStamped(carrot));
        }
        CarrotPlan carrot_plan;
        bool foundCarrotPlan = nav->GetCarrotPlan(carrot_plan);
        if (foundCarrotPlan) {
          ros_visualization::CarrotPlanToMarkerArray(carrot_plan_pub_,
                                                     "base_link", carrot_plan);
        }
        bool foundGlobalCarrot = nav->GetGlobalCarrot(carrot);
        if (foundGlobalCarrot) {
          visualization::DrawCross(carrot, 0.2f, 0x10E000, global_viz_msg_);
        }
      }
    }
  }

  void PublishNextGPSGoal() {
    if (auto nav = navigation_.lock()) {
      GPSMsg goal_msg;
      gps_util::GPSPoint goal;
      bool valid = nav->GetNextGPSGoal(goal);

      // Convert GPSPoint to GPSMsg
      goal_msg.header.stamp = node_->now();
      goal_msg.latitude = goal.lat;
      goal_msg.longitude = goal.lon;
      goal_msg.altitude = 0.0;
      goal_msg.heading = goal.heading;
      if (valid) {
        next_gps_goal_pub_->publish(goal_msg);
      }
    }
  }

  void SendCommand(const Vector2f &vel, float ang_vel) {
    TwistStamped drive_msg;
    InitRosHeader("base_link", &drive_msg.header);
    drive_msg.header.stamp = node_->now();
    bool enabled = false;
    if (auto nav = navigation_.lock()) {
      enabled = nav->Enabled();
    }

    if (!FLAGS_no_joystick && !enabled) {
      // In original code, if joystick is not used and not enabled, zero out
      // velocity
      drive_msg.twist.linear.x = 0.0;
      drive_msg.twist.angular.z = 0.0;
    } else {
      drive_msg.twist.angular.x = 0;
      drive_msg.twist.angular.y = 0;
      drive_msg.twist.angular.z = ang_vel;
      drive_msg.twist.linear.x = vel.x();
      drive_msg.twist.linear.y = vel.y();
      drive_msg.twist.linear.z = 0;
    }

    auto ackermann_msg = TwistToAckermann(drive_msg);
    ackermann_drive_pub_->publish(ackermann_msg);
    twist_drive_pub_->publish(drive_msg);

    // This command is going to take effect system latency period after. Hence
    // modify the timestamp to reflect the time when it will take effect.
    if (auto nav = navigation_.lock()) {
      nav->UpdateCommandHistory(ToTwist(drive_msg));
    }
  }

  navigation::Twist ToTwist(const TwistStamped &twist_msg) {
    navigation::Twist twist;
    twist.time = to_seconds(twist_msg.header.stamp);
    twist.linear = {static_cast<float>(twist_msg.twist.linear.x),
                    static_cast<float>(twist_msg.twist.linear.y),
                    static_cast<float>(twist_msg.twist.linear.z)};
    twist.angular = {static_cast<float>(twist_msg.twist.angular.x),
                     static_cast<float>(twist_msg.twist.angular.y),
                     static_cast<float>(twist_msg.twist.angular.z)};
    return twist;
  }

  // Member variables
  rclcpp::Node::SharedPtr node_;
  navigation::NavigationParameters params_;
  Marker line_list_marker_;
  Marker pose_marker_;
  Marker target_marker_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;

  // ---------------- Publishers (ROS2) ----------------
  rclcpp::Publisher<AckermannCurvatureDriveMsg>::SharedPtr ackermann_drive_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_drive_pub_;
  rclcpp::Publisher<MissionStatusMsg>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<NavStatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<VisualizationMsg>::SharedPtr viz_pub_;
  rclcpp::Publisher<Marker>::SharedPtr map_lines_publisher_;
  rclcpp::Publisher<Marker>::SharedPtr pose_marker_publisher_;
  rclcpp::Publisher<PointCloud>::SharedPtr fp_pcl_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<GPSMsg>::SharedPtr next_gps_goal_pub_;
  rclcpp::Publisher<Localization2DMsg>::SharedPtr localization_pub_;
  rclcpp::Publisher<GeoJSON>::SharedPtr geojson_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr fox_path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr carrot_plan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viz_bev_img_pub_;

  // ---------------- Subscribers (ROS2) ----------------
  vector<rclcpp::Subscription<LaserScan>::SharedPtr> laser_subs_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goto_sub_;
  rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr
      goto_amrl_sub_;
  rclcpp::Subscription<Empty>::SharedPtr reset_nav_goals_sub_;
  rclcpp::Subscription<Bool>::SharedPtr enabler_sub_;
  rclcpp::Subscription<Bool>::SharedPtr halt_sub_;
  rclcpp::Subscription<Pose2Df>::SharedPtr override_sub_;
  rclcpp::Subscription<GPSMsg>::SharedPtr gps_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr local_costmap_sub_;

  // ---------------- Services (ROS2) ----------------
  rclcpp::Service<GraphNavGPSSrv>::SharedPtr gps_nav_srv_;

  void setupPublishers() {
    // The topics match those in navigation_main.cc (ROS1)
    ackermann_drive_pub_ = node_->create_publisher<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 10);

    twist_drive_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        FLAGS_twist_drive_topic, 10);

    mission_status_pub_ = node_->create_publisher<MissionStatusMsg>(
        "/navigation/mission_status", 10);

    status_pub_ =
        node_->create_publisher<NavStatusMsg>("navigation_goal_status", 10);

    viz_pub_ = node_->create_publisher<VisualizationMsg>("visualization", 10);

    map_lines_publisher_ = node_->create_publisher<Marker>("map_lines", 10);

    pose_marker_publisher_ =
        node_->create_publisher<Marker>("robot_position", 10);

    fp_pcl_pub_ =
        node_->create_publisher<PointCloud>("forward_predicted_pcl", 10);

    path_pub_ = node_->create_publisher<Path>("trajectory", 10);

    carrot_pub_ = node_->create_publisher<PoseStamped>("carrot", 10);

    next_gps_goal_pub_ = node_->create_publisher<GPSMsg>("next_gps_goal", 10);

    localization_pub_ =
        node_->create_publisher<Localization2DMsg>("localization", 10);

    geojson_pub_ =
        node_->create_publisher<GeoJSON>("/navigation/geojson_waypoints", 10);

    // Path rollouts
    fox_path_pub_ =
        node_->create_publisher<MarkerArray>("/navigation/path_rollouts", 10);

    carrot_plan_pub_ = node_->create_publisher<MarkerArray>(
        "/navigation/carrot_path_rollout", 10);

    viz_img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/navigation/costmap_rollouts_image", 10);
    viz_bev_img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/navigation/bev_costmap_rollouts_image", 10);

    local_viz_msg_ =
        visualization::NewVisualizationMessage("base_link", "navigation_local");
    global_viz_msg_ =
        visualization::NewVisualizationMessage("map", "navigation_global");
    InitSimulatorVizMarkers();

    LOG_INFO("ROS2 publishers set up.");
  }

  // Example helper: set up subscriptions (implement similar callbacks as
  // needed)
  void setupSubscriptions() {
    // 0) Enabler
    enabler_sub_ = node_->create_subscription<Bool>(
        CONFIG_enable_topic, 1,
        std::bind(&Ros2AdapterImpl::EnablerCallback, this,
                  std::placeholders::_1));

    // 1) Odom
    odometry_sub_ = node_->create_subscription<Odometry>(
        CONFIG_odom_topic, 10,
        std::bind(&Ros2AdapterImpl::OdometryCallback, this,
                  std::placeholders::_1));

    // 2) Laser topics
    for (const auto &topic : CONFIG_laser_topics) {
      auto sub = node_->create_subscription<LaserScan>(
          topic, 10, [this, topic](const LaserScan::SharedPtr msg) {
            this->LaserCallback(msg, topic);
          });
      laser_subs_.push_back(sub);
    }

    // 3) Image
    image_sub_ = node_->create_subscription<CompressedImage>(
        CONFIG_image_topic, 10,
        std::bind(&Ros2AdapterImpl::CompressedImageCallback, this,
                  std::placeholders::_1));

    // 4) GoTo subscriber: /move_base_simple/goal
    goto_sub_ = node_->create_subscription<PoseStamped>(
        "/move_base_simple/goal", 1,
        std::bind(&Ros2AdapterImpl::GoToCallback, this, std::placeholders::_1));

    // 6) Reset nav goals: /reset_nav_goals
    reset_nav_goals_sub_ = node_->create_subscription<Empty>(
        "/reset_nav_goals", 1,
        std::bind(&Ros2AdapterImpl::ResetNavGoalsCallback, this,
                  std::placeholders::_1));

    // 8) Halt
    halt_sub_ = node_->create_subscription<Bool>(
        "halt_robot", 1,
        std::bind(&Ros2AdapterImpl::HaltCallback, this, std::placeholders::_1));

    // 9) Override: "nav_override"
    override_sub_ = node_->create_subscription<Pose2Df>(
        "nav_override", 1,
        std::bind(&Ros2AdapterImpl::OverrideCallback, this,
                  std::placeholders::_1));

    // 10) GPS
    gps_sub_ = node_->create_subscription<GPSMsg>(
        CONFIG_gps_topic, 1,
        std::bind(&Ros2AdapterImpl::GPSCallback, this, std::placeholders::_1));

    LOG_INFO("ROS2 subscriptions set up.");
  }

  void setupServices() {
    gps_nav_srv_ = node_->create_service<GraphNavGPSSrv>(
        "graphNavGPSSrv",
        std::bind(&Ros2AdapterImpl::GPSPlanServiceCb, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  /** BEGIN SERVICE FUNCTION IMPLEMENTATIONS **/
  void GPSPlanServiceCb(const GraphNavGPSSrv::Request::SharedPtr request,
                        const GraphNavGPSSrv::Response::SharedPtr response) {
    LOG_INFO("Received GPS service request.");

    // Lock the weak pointer to get a shared pointer
    auto nav = navigation_.lock();
    if (!nav) {
      LOG_ERROR("Navigation pointer expired.");
      return;
    }

    // Build the start GPSPoint from the request.
    const GPSPoint start(request->start.latitude, request->start.longitude);
    LOG_INFO("Start: (%f, %f)", start.lat, start.lon);
    LOG_INFO("Goals: %d", int(request->goals.data.size()));
    // Build the goals vector from the request.
    std::vector<GPSPoint> goals;
    for (const auto &goal : request->goals.data) {
      goals.emplace_back(to_seconds(goal.header.stamp), goal.latitude,
                         goal.longitude, goal.heading);
    }

    // Compute the route using the Navigation object.
    auto route = nav->GlobalPlan(start, goals);
    auto map_route = nav->GPSRouteToMap(route);

    // Clear and update the global visualization message.
    global_viz_msg_.lines.clear();
    for (const auto &p : map_route) {
      visualization::DrawPoint(p.cast<float>(), 0xFF0000, global_viz_msg_);
    }
    viz_pub_->publish(global_viz_msg_);

    printf("Goals in osrm plan: %d\n", int(route.size()));
    nav->SetGPSNavGoals(route);

    // Build a GPSArrayMsg from the route.
    GPSArrayMsg gps_goals_msg;
    // Use the node's clock for ROS2 time.
    gps_goals_msg.header.stamp = node_->now();
    for (const auto &route_node : route) {
      GPSMsg goal_msg;
      goal_msg.header.stamp = gps_goals_msg.header.stamp;
      goal_msg.latitude = route_node.lat;
      goal_msg.longitude = route_node.lon;
      goal_msg.heading = route_node.heading;
      gps_goals_msg.data.emplace_back(goal_msg);
    }
    response->plan = gps_goals_msg;
    printf("Goals in gps_goals_msgs: %d\n", int(gps_goals_msg.data.size()));
  }

  /** END SERVICE FUNCTION IMPLEMENTATIONS **/

  /** BEGIN CALLBACK FUNCTION IMPLEMENTATIONS **/
  void EnablerCallback(const Bool::SharedPtr msg) {
    if (auto nav = navigation_.lock()) {
      nav->Enable(msg->data);
    }
  }

  void OdometryCallback(const Odometry::SharedPtr msg) {
    if (FLAGS_v > 2) {
      printf("Odometry t=%f\n",
             msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    }
    odom_ = OdomHandler(msg);

    if (auto nav = navigation_.lock()) {
      nav->UpdateOdometry(odom_);
    }
  }

  void LaserCallback(const LaserScan::SharedPtr msg, const string &topic) {
    vector<Vector2f> point_cloud;
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (r < msg->range_min || r > msg->range_max) r = msg->range_max;
      point_cloud.push_back(Vector2f(r * cos(angle), r * sin(angle)));
      angle += msg->angle_increment;
    }
    point_cloud_ = point_cloud;

    if (auto nav = navigation_.lock()) {
      nav->ObservePointCloud(point_cloud, to_seconds(msg->header.stamp));
    }
  }

  void CompressedImageCallback(const CompressedImage::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      last_image_ = cv_ptr->image;

      if (auto nav = navigation_.lock()) {
        nav->ObserveImage(last_image_, to_seconds(msg->header.stamp));
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void GoToCallback(const PoseStamped::SharedPtr msg) {
    Vector2f loc(msg->pose.position.x, msg->pose.position.y);
    float angle =
        2.0f * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
    printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);

    if (auto nav = navigation_.lock()) {
      nav->SetNavGoal(loc, angle);
      nav->Resume();
    }
  }

  void ResetNavGoalsCallback(const Empty::SharedPtr msg) {
    printf("Resetting all nav goals.\n");

    if (auto nav = navigation_.lock()) {
      nav->ResetNavGoals();
    }
  }

  void HaltCallback(const Bool::SharedPtr msg) {
    printf("Halting navigation.\n");

    if (auto nav = navigation_.lock()) {
      nav->Pause();
    }
  }

  void OverrideCallback(const Pose2Df::SharedPtr msg) {
    // Convert to generic override goal.
    Vector2f loc(msg->x, msg->y);
    float angle = msg->theta;

    printf("Overriding navigation with new goal: (%f,%f) %f\u00b0\n", loc.x(),
           loc.y(), angle);

    if (auto nav = navigation_.lock()) {
      nav->SetOverride(loc, angle);
    }
  }

  void GPSCallback(const GPSMsg::SharedPtr msg) {
    double stamp = to_seconds(msg->header.stamp);
    GPSPoint loc(stamp, msg->latitude, msg->longitude, msg->heading);

    if (FLAGS_v > 2) {
      printf("GPS Pose: %lf %lf\n", msg->latitude, msg->longitude);
    }

    if (auto nav = navigation_.lock()) {
      nav->UpdateGPS(loc);
    }
  }
  /** END CALLBACK FUNCTION IMPLEMENTATIONS **/

  void InitVizMarker(Marker &vizMarker, const std::string &ns, int id,
                     const std::string &type, const PoseStamped &p,
                     const Point32 &scale, double duration,
                     const std::vector<float> &color) {
    vizMarker.header.frame_id = p.header.frame_id;
    vizMarker.header.stamp = node_->now();

    vizMarker.ns = ns;
    vizMarker.id = id;

    // Set the marker type.
    if (type == "arrow") {
      vizMarker.type = Marker::ARROW;
    } else if (type == "cube") {
      vizMarker.type = Marker::CUBE;
    } else if (type == "sphere") {
      vizMarker.type = Marker::SPHERE;
    } else if (type == "cylinder") {
      vizMarker.type = Marker::CYLINDER;
    } else if (type == "linelist") {
      vizMarker.type = Marker::LINE_LIST;
    } else if (type == "linestrip") {
      vizMarker.type = Marker::LINE_STRIP;
    } else if (type == "points") {
      vizMarker.type = Marker::POINTS;
    } else {
      vizMarker.type = Marker::ARROW;
    }

    // Set the pose.
    vizMarker.pose = p.pose;
    vizMarker.points.clear();

    // Set the scale.
    vizMarker.scale.x = scale.x;
    vizMarker.scale.y = scale.y;
    vizMarker.scale.z = scale.z;

    // Set lifetime using our DURATION() macro.
    vizMarker.lifetime = DURATION(duration);

    // Set the marker color (RGBA)
    vizMarker.color.r = color.at(0);
    vizMarker.color.g = color.at(1);
    vizMarker.color.b = color.at(2);
    vizMarker.color.a = color.at(3);

    // Finally, set the marker action to ADD.
    vizMarker.action = Marker::ADD;
  }

  void InitSimulatorVizMarkers() {
    PoseStamped p;
    Point32 scale;
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

    InitVizMarker(pose_marker_, "robot_position", 1, "cube", p, scale, 0.0,
                  color);

    scale.x = 0.05;
    scale.y = 0.05;
    scale.z = 0.05;

    InitVizMarker(target_marker_, "targets", 1, "points", p, scale, 0.0, color);

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
};

// Factory function: implement RosAdapter::create outside the class definition.
std::shared_ptr<RosAdapter> RosAdapter::create(
    rclcpp::Node::SharedPtr node,
    const navigation::NavigationParameters &params) {
  return std::make_shared<Ros2AdapterImpl>(node, params);
}
};  // namespace navigation

#endif  // ROS2
