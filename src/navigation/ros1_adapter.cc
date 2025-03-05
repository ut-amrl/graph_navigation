#ifdef ROS1  // Compile this file when building for ROS1 (e.g. add_definitions(-DROS1))

#include "ros_adapter.h"

// ---------- ROS1 and other includes ----------
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>
#include <vector>

// ---------- Shared libraries, AMRL msgs, etc. ----------
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/GPSArrayMsg.h"
#include "amrl_msgs/GPSMsg.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/MissionStatusMsg.h"
#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "amrl_msgs/graphNavGPSSrv.h"
#include "foxglove_msgs/GeoJSON.h"
#include "graph_navigation/graphNavSrv.h"
#include "visualization_msgs/MarkerArray.h"

#include "config_reader/config_reader.h"
#include "constant_curvature_arcs.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "glog/logging.h"
#include "motion_primitives.h"
#include "nav_msgs/Path.h"
#include "navigation.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization/ros_visualization.h"
#include "visualization/visualization.h"

// ---------- GFlags Declarations (flags are defined in main) ----------
#include <gflags/gflags.h>
DECLARE_bool(no_joystick);
DECLARE_bool(no_intermed);
DECLARE_bool(debug_images);
DECLARE_bool(simulate);

// ---------- Using/Namespaces ----------
using namespace ros_helpers;
using namespace geometry;
using namespace math_util;
using namespace std_msgs;
using namespace std;

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::GPSArrayMsg;
using amrl_msgs::GPSMsg;
using amrl_msgs::Localization2DMsg;
using amrl_msgs::MissionStatusMsg;
using amrl_msgs::NavStatusMsg;
using amrl_msgs::Pose2Df;
using amrl_msgs::VisualizationMsg;
using amrl_msgs::graphNavGPSSrv;
using Eigen::Affine3f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using foxglove_msgs::GeoJSON;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::TwistStamped;
using graph_navigation::graphNavSrv;
using navigation::MissionStatus;
using navigation::MotionLimits;
using navigation::Navigation;
using navigation::PathOption;
using ros::Time;
using ros_helpers::InitRosHeader;
using ros_helpers::RateLoop;
using sensor_msgs::PointCloud;
using visualization_msgs::MarkerArray;

// Config-defined strings from your config .lua
CONFIG_STRING(image_topic, "NavigationParameters.image_topic");
CONFIG_STRINGLIST(laser_topics, "NavigationParameters.laser_topics");
CONFIG_STRING(laser_frame, "NavigationParameters.laser_frame");
CONFIG_STRING(odom_topic, "NavigationParameters.odom_topic");
CONFIG_STRING(localization_topic, "NavigationParameters.localization_topic");
CONFIG_STRING(gps_topic, "OSMPlannerParameters.gps_topic");
CONFIG_STRING(gps_goals_topic, "OSMPlannerParameters.gps_goals_topic");
CONFIG_STRING(init_topic, "NavigationParameters.init_topic");
CONFIG_STRING(enable_topic, "NavigationParameters.enable_topic");
CONFIG_STRING(map_param, "NavigationParameters.map");

DEFINE_string(twist_drive_topic, "navigation/cmd_vel", "Drive Command Topic");

// ---------- Ros1AdapterImpl Definition ----------
class Ros1AdapterImpl : public RosAdapter {
 public:
  explicit Ros1AdapterImpl(navigation::Navigation* nav)
      : nav_(nav),
        it_(nullptr),
        enabled_(false),
        received_odom_(false),
        received_laser_(false),
        received_gps_(false),
        current_angle_(0),
        goal_angle_(0) {
    CHECK(nav_) << "navigation::Navigation pointer is null!";
  }

  void Initialize(int argc, char** argv) override {
    // Note: The command line flags (GFlags) are parsed in navigation_main.cc
    // We do NOT parse them here, but we can still use them (DECLARED above).

    // Initialize ROS1
    ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle());

    // Create an ImageTransport
    it_.reset(new image_transport::ImageTransport(*nh_));

    // Setup Publishers, Subscribers, Services
    setupPublishers();
    setupSubscribers();
    setupServices();

    // Initialize local visualization messages or other structures
    initVisualization();

    // Only set if running offline (no real robot)
    simulate_ = FLAGS_simulate;
  }

  void spinLoop() override {
    // Spin at a rate determined by the navigation dt (or your preference)
    double dt = nav_->GetParams().dt;
    RateLoop loop(1.0 / dt);

    while (ros::ok()) {
      // 1. Clear old visualization data
      visualization::ClearVisualizationMsg(local_viz_msg_);
      visualization::ClearVisualizationMsg(global_viz_msg_);

      // 2. Mark that we haven't received a new laser in this loop iteration
      received_laser_ = false;

      // 3. Let ROS process new messages (calls callbacks)
      ros::spinOnce();

      // 4. Run the navigation logic
      Vector2f cmd_vel(0, 0);
      float cmd_angle_vel = 0.0f;
      bool nav_succeeded =
          nav_->Run(ros::Time::now().toSec(), cmd_vel, cmd_angle_vel);

      // 5. Publish transforms, statuses, mission updates
      if (!simulate_) {
        PublishTF();
      }
      PublishNavStatus();
      PublishMissionStatus();
      PublishLocalization();
      PublishGlobalPlan();

      // 6. If navigation succeeded, handle all the visualization & commands
      if (nav_succeeded) {
        if (!FLAGS_no_intermed) {
          // Additional visualizations if you like
        }
        // Forward predicted PCL, draw robot & target, path options, etc.
        PublishForwardPredictedPCL(nav_->GetPredictedCloud());
        DrawRobot();
        if (nav_->GetNavStatusUint8() !=
            static_cast<uint8_t>(navigation::NavigationState::kStopped)) {
          DrawTarget();
          DrawPathOptions();
        }
        PublishVisualizationMarkers();
        PublishPath();
        PublishNextGPSGoal();

        // Stamp and publish local/global visualization messages
        local_viz_msg_.header.stamp = ros::Time::now();
        global_viz_msg_.header.stamp = ros::Time::now();
        viz_pub_.publish(local_viz_msg_);
        viz_pub_.publish(global_viz_msg_);

        // Possibly publish debug images if you're using cost_map, etc.
        if (nav_->GetParams().evaluator_type == "cost_map" ||
            nav_->GetParams().evaluator_type == "cost_map_service" ||
            nav_->GetParams().evaluator_type == "terrain2") {
          cv_bridge::CvImage viz_img;
          cv_bridge::CvImage bev_viz_img;
          bool result =
              nav_->GetVisualizationImage(viz_img.image, bev_viz_img.image);
          printf("Does result exist? %d\n", result ? 1 : 0);

          auto img_encoding =
              (nav_->GetParams().evaluator_type == "cost_map_service")
                  ? sensor_msgs::image_encodings::BGRA8
                  : sensor_msgs::image_encodings::BGR8;

          if (result) {
            if (!viz_img.image.empty()) {
              viz_img.header.stamp = ros::Time::now();
              viz_img.encoding = img_encoding;
              viz_img_pub_.publish(viz_img.toImageMsg());
            }
            if (!bev_viz_img.image.empty()) {
              printf("Publishing bev_viz_img\n");
              bev_viz_img.header.stamp = viz_img.header.stamp;
              bev_viz_img.encoding = img_encoding;
              viz_bev_img_pub_.publish(bev_viz_img.toImageMsg());
            }
          }
        }

        // Finally, send motion commands to the robot
        SendCommand(cmd_vel, cmd_angle_vel);
      }

      // 7. Sleep until next iteration
      loop.Sleep();
    }  // while ros::ok()
  }

 private:
  // ============= Members ===============
  navigation::Navigation* nav_;

  // NodeHandle & others
  std::shared_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<image_transport::ImageTransport> it_;

  bool simulate_;       // Set from FLAGS_simulate in init()
  bool enabled_;
  bool received_odom_;
  bool received_laser_;
  bool received_gps_;

  Vector2f current_loc_;
  Vector2f current_vel_;
  float current_angle_;
  float goal_angle_;
  navigation::Odom odom_;
  vector<Vector2f> point_cloud_;
  sensor_msgs::LaserScan last_laser_msg_;
  cv::Mat last_image_;

  // LaserCache struct from original code
  struct LaserCache {
    double time = 0.0;
    float dtheta = 0.0f;
    float angle_min = 0.0f;
    vector<Vector3f> rays;
    Affine3f frame_tf = Affine3f::Identity();
  };
  unordered_map<string, LaserCache> laser_caches_;

  // Publishers
  ros::Publisher ackermann_drive_pub_;
  ros::Publisher twist_drive_pub_;
  ros::Publisher mission_status_pub_;
  ros::Publisher status_pub_;
  ros::Publisher viz_pub_;
  image_transport::Publisher viz_img_pub_;
  image_transport::Publisher viz_bev_img_pub_;
  ros::Publisher fp_pcl_pub_;
  ros::Publisher path_pub_;
  ros::Publisher carrot_pub_;
  ros::Publisher next_gps_goal_pub_;
  ros::Publisher localization_pub_;
  ros::Publisher geojson_pub_;
  ros::Publisher fox_path_pub_;
  ros::Publisher carrot_plan_pub_;

  // Services
  ros::ServiceServer nav_srv_;
  ros::ServiceServer gps_nav_srv_;

  // Visualization
  visualization_msgs::Marker line_list_marker_;
  visualization_msgs::Marker pose_marker_;
  visualization_msgs::Marker target_marker_;
  amrl_msgs::VisualizationMsg local_viz_msg_;
  amrl_msgs::VisualizationMsg global_viz_msg_;

  // ============= Setup Methods ==============
  void setupPublishers() {
    ackermann_drive_pub_ = nh_->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    twist_drive_pub_ =
        nh_->advertise<geometry_msgs::Twist>(FLAGS_twist_drive_topic, 1);
    mission_status_pub_ = nh_->advertise<MissionStatusMsg>(
        "/navigation/mission_status", 1, true);
    status_pub_ = nh_->advertise<NavStatusMsg>("navigation_goal_status", 1);
    viz_pub_ = nh_->advertise<VisualizationMsg>("visualization", 1);

    viz_img_pub_ =
        it_->advertise("/navigation/costmap_rollouts_image", 1);
    viz_bev_img_pub_ =
        it_->advertise("/navigation/bev_costmap_rollouts_image", 1);

    fp_pcl_pub_ = nh_->advertise<PointCloud>("forward_predicted_pcl", 1);
    path_pub_ = nh_->advertise<nav_msgs::Path>("trajectory", 1);
    carrot_pub_ = nh_->advertise<PoseStamped>("carrot", 1, true);
    next_gps_goal_pub_ = nh_->advertise<GPSMsg>("next_gps_goal", 1, true);
    localization_pub_ = nh_->advertise<Localization2DMsg>("localization", 1);
    geojson_pub_ = nh_->advertise<GeoJSON>("navigation/geojson_waypoints", 10);

    fox_path_pub_ =
        nh_->advertise<MarkerArray>("/navigation/path_rollouts", 1);
    carrot_plan_pub_ =
        nh_->advertise<MarkerArray>("/navigation/carrot_path_rollout", 1);
  }

  void setupSubscribers() {
    // Subscribe to Laser topics
    for (const auto &topic : CONFIG_laser_topics) {
      nh_->subscribe<sensor_msgs::LaserScan>(
          topic, 1,
          [this, topic](const sensor_msgs::LaserScan::ConstPtr &msg_ptr) {
            LaserCallback(*msg_ptr, topic);
          });
    }

    // Odom
    nh_->subscribe<nav_msgs::Odometry>(
        CONFIG_odom_topic, 1,
        [this](const nav_msgs::Odometry::ConstPtr &msg_ptr) {
          OdometryCallback(*msg_ptr);
        });

    // GPS
    nh_->subscribe<amrl_msgs::GPSMsg>(
        CONFIG_gps_topic, 1,
        [this](const amrl_msgs::GPSMsg::ConstPtr &msg) {
          GPSCallback(*msg);
        });

    // Image
    nh_->subscribe<sensor_msgs::CompressedImage>(
        CONFIG_image_topic, 1,
        [this](const sensor_msgs::CompressedImage::ConstPtr &msg) {
          ImageCallback(msg);
        });

    // /move_base_simple/goal
    nh_->subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1,
        [this](const geometry_msgs::PoseStamped::ConstPtr &msg) {
          GoToCallback(*msg);
        });

    // /move_base_simple/goal_amrl
    nh_->subscribe<Localization2DMsg>(
        "/move_base_simple/goal_amrl", 1,
        [this](const Localization2DMsg::ConstPtr &msg) {
          GoToCallbackAMRL(*msg);
        });

    // /reset_nav_goals
    nh_->subscribe<std_msgs::Empty>(
        "/reset_nav_goals", 1,
        [this](const std_msgs::Empty::ConstPtr &msg) {
          ResetNavGoalsCallback(*msg);
        });

    // Enabler
    nh_->subscribe<Bool>(
        CONFIG_enable_topic, 1,
        [this](const Bool::ConstPtr &msg) {
          EnablerCallback(*msg);
        });

    // halt_robot
    nh_->subscribe<Bool>(
        "halt_robot", 1,
        [this](const Bool::ConstPtr &msg) {
          HaltCallback(*msg);
        });

    // nav_override
    nh_->subscribe<Pose2Df>(
        "nav_override", 1,
        [this](const Pose2Df::ConstPtr &msg) {
          OverrideCallback(*msg);
        });

    // local_costmap
    nh_->subscribe<nav_msgs::OccupancyGrid>(
        "local_costmap", 1,
        [this](const nav_msgs::OccupancyGrid::ConstPtr &msg) {
          LocalCostmapCallback(*msg);
        });
  }

  void setupServices() {
    nav_srv_ = nh_->advertiseService(
        "graphNavSrv", &Ros1AdapterImpl::PlanServiceCb, this);

    gps_nav_srv_ = nh_->advertiseService(
        "graphNavGPSSrv", &Ros1AdapterImpl::GPSPlanServiceCb, this);
  }

  void initVisualization() {
    // Initialize your visualization messages
    local_viz_msg_ =
        visualization::NewVisualizationMessage("base_link", "navigation_local");
    global_viz_msg_ =
        visualization::NewVisualizationMessage("map", "navigation_global");
    InitSimulatorVizMarkers();
  }

  // =========== ROS Callback Methods (ported from navigation_main.cc) ==========
  void EnablerCallback(const std_msgs::Bool &msg) {
    enabled_ = msg.data;
  }

  void OdometryCallback(const nav_msgs::Odometry &msg) {
    received_odom_ = true;
    odom_ = OdomHandler(msg);
    nav_->UpdateOdometry(odom_);
  }

  navigation::Odom OdomHandler(const nav_msgs::Odometry &msg) {
    navigation::Odom odom;
    odom.time = msg.header.stamp.toSec();
    odom.orientation = {
        static_cast<float>(msg.pose.pose.orientation.w),
        static_cast<float>(msg.pose.pose.orientation.x),
        static_cast<float>(msg.pose.pose.orientation.y),
        static_cast<float>(msg.pose.pose.orientation.z)};
    odom.position = {
        static_cast<float>(msg.pose.pose.position.x),
        static_cast<float>(msg.pose.pose.position.y),
        static_cast<float>(msg.pose.pose.position.z)};
    return odom;
  }

  void GPSCallback(const amrl_msgs::GPSMsg &msg) {
    GPSPoint loc(msg.header.stamp.toSec(), msg.latitude, msg.longitude,
                 msg.heading);
    received_gps_ = true;
    nav_->UpdateGPS(loc);
  }

  void LocalCostmapCallback(const nav_msgs::OccupancyGrid &msg) {
    // Replicated from original code
    unsigned int width = msg.info.width;
    unsigned int height = msg.info.height;
    double resolution = msg.info.resolution;
    double origin_x = msg.info.origin.position.x;
    double origin_y = msg.info.origin.position.y;

    costmap_2d::Costmap2D costmap(width, height, resolution, origin_x, origin_y);
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
        unsigned int idx = x + y * width;
        int8_t cell_value = msg.data[idx];
        float norm_val;
        if (cell_value == -1) {
          norm_val = costmap_2d::NO_INFORMATION;
        } else {
          norm_val = static_cast<float>(cell_value) / 100.0f;
        }
        costmap.setCost(x, y, static_cast<unsigned char>(norm_val * 255.0f));
      }
    }
    nav_->UpdateLocalCostmap(costmap);
  }

  void RetrieveTransform(const std_msgs::Header &hdr, Affine3f &frame_tf) {
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform(CONFIG_laser_frame, hdr.frame_id, hdr.stamp,
                                   ros::Duration(0.01));
      tf_listener.lookupTransform(CONFIG_laser_frame, hdr.frame_id, hdr.stamp,
                                  transform);
      frame_tf = Eigen::Translation3f(
                     transform.getOrigin().getX(),
                     transform.getOrigin().getY(),
                     transform.getOrigin().getZ()) *
                 Eigen::Quaternionf(
                     transform.getRotation().getW(),
                     transform.getRotation().getX(),
                     transform.getRotation().getY(),
                     transform.getRotation().getZ());
    } catch (tf::TransformException &ex) {
      ROS_WARN("Failed transform from '%s' to '%s': %s",
               hdr.frame_id.c_str(), CONFIG_laser_frame.c_str(), ex.what());
      // fallback to identity
    }
  }

  void LaserHandler(const sensor_msgs::LaserScan &msg, const string &topic) {
    static bool first_laser = true;
    if (first_laser) {
      first_laser = false;
      point_cloud_.clear();
      received_laser_ = true;
    }

    auto &cache = laser_caches_[topic];
    if (cache.dtheta != msg.angle_increment ||
        cache.angle_min != msg.angle_min ||
        cache.rays.size() != msg.ranges.size()) {
      cache.dtheta = msg.angle_increment;
      cache.angle_min = msg.angle_min;
      cache.rays.resize(msg.ranges.size());
      for (size_t i = 0; i < cache.rays.size(); ++i) {
        float a = cache.angle_min + (float)i * cache.dtheta;
        cache.rays[i] = Vector3f(cos(a), sin(a), 0.0f);
      }
    }

    RetrieveTransform(msg.header, cache.frame_tf);

    size_t start_idx = point_cloud_.size();
    point_cloud_.resize(start_idx + cache.rays.size());
    for (size_t i = 0; i < cache.rays.size(); ++i) {
      float r = ((msg.ranges[i] > msg.range_min && msg.ranges[i] < msg.range_max)
                     ? msg.ranges[i]
                     : msg.range_max);
      point_cloud_[start_idx + i] =
          (cache.frame_tf * (r * cache.rays[i])).head<2>();
    }
  }

  void LaserCallback(const sensor_msgs::LaserScan &msg, const string &topic) {
    LaserHandler(msg, topic);
    nav_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  }

  void GoToCallback(const geometry_msgs::PoseStamped &msg) {
    Vector2f loc(msg.pose.position.x, msg.pose.position.y);
    float angle = 2.0f * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
    nav_->SetNavGoal(loc, angle);
    nav_->Resume();
  }

  void GoToCallbackAMRL(const Localization2DMsg &msg) {
    Vector2f loc(msg.pose.x, msg.pose.y);
    nav_->SetNavGoal(loc, msg.pose.theta);
    nav_->Resume();
  }

  void ResetNavGoalsCallback(const std_msgs::Empty &msg) {
    nav_->ResetNavGoals();
  }

  bool PlanServiceCb(graphNavSrv::Request &req, graphNavSrv::Response &res) {
    Vector2f start(req.start.x, req.start.y);
    Vector2f end(req.end.x, req.end.y);
    auto plan = nav_->GlobalPlan(start, end);
    res.plan = plan;
    return true;
  }

  bool GPSPlanServiceCb(graphNavGPSSrv::Request &req,
                        graphNavGPSSrv::Response &res) {
    GPSPoint start(req.start.latitude, req.start.longitude);
    vector<GPSPoint> goals;
    for (auto &g : req.goals.data) {
      goals.emplace_back(g.header.stamp.toSec(), g.latitude, g.longitude,
                         g.heading);
    }
    auto route = nav_->GlobalPlan(start, goals);
    auto map_route = nav_->GPSRouteToMap(route);
    global_viz_msg_.lines.clear();
    for (auto &p : map_route) {
      visualization::DrawPoint(p.cast<float>(), 0xFF0000, global_viz_msg_);
    }
    viz_pub_.publish(global_viz_msg_);

    nav_->SetGPSNavGoals(route);

    GPSArrayMsg gps_goals_msg;
    gps_goals_msg.header.stamp = ros::Time::now();
    for (auto &node : route) {
      GPSMsg goal_msg;
      goal_msg.header.stamp = gps_goals_msg.header.stamp;
      goal_msg.latitude = node.lat;
      goal_msg.longitude = node.lon;
      goal_msg.heading = node.heading;
      gps_goals_msg.data.push_back(goal_msg);
    }
    res.plan = gps_goals_msg;
    return true;
  }

  void OverrideCallback(const Pose2Df &msg) {
    Vector2f loc(msg.x, msg.y);
    nav_->SetOverride(loc, msg.theta);
  }

  void HaltCallback(const Bool &msg) {
    nav_->Pause();
  }

  AckermannCurvatureDriveMsg TwistToAckermann(const TwistStamped &twist) {
    AckermannCurvatureDriveMsg ackermann_msg;
    ackermann_msg.header = twist.header;
    ackermann_msg.velocity = twist.twist.linear.x;
    if (fabs(ackermann_msg.velocity) < kEpsilon) {
      ackermann_msg.curvature = 0.0f;
    } else {
      ackermann_msg.curvature = twist.twist.angular.z / ackermann_msg.velocity;
    }
    return ackermann_msg;
  }

  geometry_msgs::TwistStamped AckermannToTwist(const AckermannCurvatureDriveMsg &msg) {
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header = msg.header;
    twist_msg.twist.linear.x = msg.velocity;
    twist_msg.twist.angular.z = msg.velocity * msg.curvature;
    return twist_msg;
  }

  navigation::Twist ToTwist(const TwistStamped &twist_msg) {
    navigation::Twist twist;
    twist.time = twist_msg.header.stamp.toSec();
    twist.linear = {static_cast<float>(twist_msg.twist.linear.x),
                    static_cast<float>(twist_msg.twist.linear.y),
                    static_cast<float>(twist_msg.twist.linear.z)};
    twist.angular = {static_cast<float>(twist_msg.twist.angular.x),
                     static_cast<float>(twist_msg.twist.angular.y),
                     static_cast<float>(twist_msg.twist.angular.z)};
    return twist;
  }

  void PublishMissionStatus() {
    MissionStatus status = nav_->GetMissionStatus();
    MissionStatusMsg status_msg;
    status_msg.stamp = ros::Time(status.time);
    status_msg.status = status.status;
    status_msg.mission_id = status.mission_id;
    status_msg.next_goal_id = status.next_goal_id;

    for (size_t i = 0; i < status.goals.size(); ++i) {
      GPSMsg goal_msg;
      goal_msg.header.stamp = ros::Time(status.goals[i].time);
      goal_msg.latitude = status.goals[i].lat;
      goal_msg.longitude = status.goals[i].lon;
      status_msg.goals.data.push_back(goal_msg);

      if (i < status.goals_reached.size()) {
        GPSMsg reached_msg;
        reached_msg.header.stamp = ros::Time(status.goals_reached[i].time);
        reached_msg.latitude = status.goals_reached[i].lat;
        reached_msg.longitude = status.goals_reached[i].lon;
        status_msg.goals_reached.data.push_back(reached_msg);
      }
    }
    mission_status_pub_.publish(status_msg);
  }

  void PublishNavStatus() {
    NavStatusMsg status;
    status.stamp = ros::Time::now();
    status.status = nav_->GetNavStatusUint8();
    status_pub_.publish(status);
  }

  void SendCommand(const Vector2f &vel, float ang_vel) {
    geometry_msgs::TwistStamped drive_msg;
    InitRosHeader("base_link", &drive_msg.header);
    drive_msg.header.stamp = ros::Time::now();

    // If joystick is not used & not enabled, zero out velocity
    if (!FLAGS_no_joystick && !enabled_) {
      drive_msg.twist.linear.x = 0.0;
      drive_msg.twist.angular.z = 0.0;
    } else {
      drive_msg.twist.angular.z = ang_vel;
      drive_msg.twist.linear.x = vel.x();
      drive_msg.twist.linear.y = vel.y();
      drive_msg.twist.linear.z = 0;
      drive_msg.twist.angular.x = 0;
      drive_msg.twist.angular.y = 0;
    }

    AckermannCurvatureDriveMsg ackermann_msg = TwistToAckermann(drive_msg);
    ackermann_drive_pub_.publish(ackermann_msg);
    twist_drive_pub_.publish(drive_msg.twist);

    nav_->UpdateCommandHistory(ToTwist(drive_msg));
  }

  void PublishForwardPredictedPCL(const vector<Vector2f> &pcl) {
    PointCloud fp_pcl_msg;
    fp_pcl_msg.points.resize(pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i) {
      fp_pcl_msg.points[i].x = pcl[i].x();
      fp_pcl_msg.points[i].y = pcl[i].y();
      fp_pcl_msg.points[i].z = 0.324f;
    }
    fp_pcl_msg.header.stamp = ros::Time::now();
    fp_pcl_pub_.publish(fp_pcl_msg);
  }

  nav_msgs::Path CarrotToNavMsgsPath(const Vector2f &carrot) {
    nav_msgs::Path carrotNav;
    carrotNav.header.stamp = ros::Time::now();
    carrotNav.header.frame_id = "map";
    geometry_msgs::PoseStamped carrotPose;
    carrotPose.pose.position.x = carrot.x();
    carrotPose.pose.position.y = carrot.y();
    carrotPose.pose.orientation.w = 1.0;
    carrotPose.header = carrotNav.header;
    carrotNav.poses.push_back(carrotPose);
    return carrotNav;
  }

  PoseStamped CarrotToPoseStamped(const Vector2f &carrot) {
    PoseStamped carrotPose;
    carrotPose.header.stamp = ros::Time::now();
    carrotPose.header.frame_id = "base_link";
    carrotPose.pose.position.x = carrot.x();
    carrotPose.pose.position.y = carrot.y();
    carrotPose.pose.orientation.w = 1.0f;
    return carrotPose;
  }

  void PublishLocalization() {
    Eigen::Vector3f robot_pose;
    if (!nav_->GetRobotPose(robot_pose)) return;
    Localization2DMsg loc_msg;
    loc_msg.header.stamp = ros::Time::now();
    loc_msg.pose.x = robot_pose.x();
    loc_msg.pose.y = robot_pose.y();
    loc_msg.pose.theta = robot_pose.z();
    localization_pub_.publish(loc_msg);
  }

  void PublishPath() {
    auto path = nav_->GetPlanPath();
    if (path.size() < 2) return;

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    for (auto &step : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = step.loc.x();
      ps.pose.position.y = step.loc.y();
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    // Publish the path
    path_pub_.publish(path_msg);

    // Draw lines
    for (size_t i = 1; i < path.size(); i++) {
      visualization::DrawLine(path[i - 1].loc, path[i].loc, 0x007F00,
                              global_viz_msg_);
    }
    auto global_path = nav_->GetGlobalPath();
    for (size_t i = 1; i < global_path.size(); i++) {
      visualization::DrawLine(global_path[i - 1].loc, global_path[i].loc,
                              0xA86032, global_viz_msg_);
    }

    // Carrot
    Vector2f carrot;
    bool foundCarrot = nav_->GetLocalCarrotHeading(carrot, false);
    if (foundCarrot) {
      carrot_pub_.publish(CarrotToPoseStamped(carrot));
    }

    // CarrotPlan
    CarrotPlan carrot_plan;
    bool foundCarrotPlan = nav_->GetCarrotPlan(carrot_plan);
    if (foundCarrotPlan) {
      ros_visualization::CarrotPlanToMarkerArray(
          carrot_plan_pub_, "base_link", carrot_plan);
    }

    bool foundGlobalCarrot = nav_->GetGlobalCarrot(carrot);
    if (foundGlobalCarrot) {
      visualization::DrawCross(carrot, 0.2f, 0x10E000, global_viz_msg_);
    }
  }

  void PublishNextGPSGoal() {
    GPSMsg goal_msg;
    bool valid = nav_->GetNextGPSGoal(goal_msg);
    if (valid) {
      next_gps_goal_pub_.publish(goal_msg);
    }
  }

  void DrawTarget() {
    float carrot_dist = nav_->GetCarrotDist();
    Vector2f target = nav_->GetTarget();
    auto msg_copy = global_viz_msg_;
    visualization::DrawCross(nav_->GetIntermediateGoal(), 0.2f, 0x0000FF,
                             global_viz_msg_);
    visualization::DrawArc(Vector2f(0, 0), carrot_dist, -M_PI, M_PI, 0xE0E0E0,
                           local_viz_msg_);
    viz_pub_.publish(msg_copy);
    visualization::DrawCross(target, 0.2f, 0xFF0080, local_viz_msg_);
  }

  void DrawRobot() {
    float kRobotLength = nav_->GetRobotLength();
    float kRobotWidth = nav_->GetRobotWidth();
    float kObstacleMargin = nav_->GetObstacleMargin();
    float kRearAxleOffset = 0.0;

    {
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
    }
    {
      float l1 = -0.5f * kRobotLength - kRearAxleOffset;
      float l2 = 0.5f * kRobotLength - kRearAxleOffset;
      float w = 0.5f * kRobotWidth;
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

  vector<PathOption> ToOptions(const vector<shared_ptr<motion_primitives::PathRolloutBase>> &paths) {
    vector<PathOption> options;
    for (auto &p : paths) {
      const auto &arc =
          *reinterpret_cast<const motion_primitives::ConstantCurvatureArc *>(p.get());
      PathOption o;
      o.curvature = arc.curvature;
      o.free_path_length = arc.Length();
      o.clearance = arc.Clearance();
      options.push_back(o);
    }
    return options;
  }

  void DrawPathOptions() {
    auto path_rollouts = nav_->GetLastPathOptions();
    if (path_rollouts.empty()) return;

    auto path_options = ToOptions(path_rollouts);
    auto best_option = nav_->GetOption();

    // Draw all
    for (auto &opt : path_options) {
      visualization::DrawPathOption(opt.curvature, opt.free_path_length, opt.clearance,
                                    0x0000FF, false, local_viz_msg_);
    }

    // Move best option to front
    if (best_option != nullptr) {
      auto bestVec = ToOptions({best_option});
      path_options.insert(path_options.begin(), bestVec[0]);
    }

    // First is red, rest are blue
    vector<vector<float>> colors(path_options.size(), {0.f, 0.f, 1.f, 1.f});
    colors[0] = {1.f, 0.f, 0.f, 1.f};

    ros_visualization::PathOptionToMarkerArray(
        fox_path_pub_, "base_link", path_options, colors, false);

    if (best_option != nullptr) {
      auto best_arc =
          *reinterpret_cast<const motion_primitives::ConstantCurvatureArc *>(best_option.get());
      visualization::DrawPathOption(best_arc.curvature, best_arc.length,
                                    best_arc.clearance, 0xFF0000, true,
                                    local_viz_msg_);
    }
  }

  void PublishGlobalPlan() {
    vector<GPSPoint> plan;
    bool is_valid = nav_->GetGlobalPlan(plan);
    if (is_valid) {
      ros_visualization::GPSRouteToGeoJSON(geojson_pub_, plan);
    } else {
      ROS_WARN("Global plan is not valid. path length = %zu", plan.size());
    }
  }

  void InitVizMarker(visualization_msgs::Marker &vizMarker, string ns, int id,
                     string type, geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale, double duration,
                     vector<float> color) {
    vizMarker.header.frame_id = p.header.frame_id;
    vizMarker.header.stamp = ros::Time::now();
    vizMarker.ns = ns;
    vizMarker.id = id;

    if (type == "arrow") vizMarker.type = visualization_msgs::Marker::ARROW;
    else if (type == "cube") vizMarker.type = visualization_msgs::Marker::CUBE;
    else if (type == "sphere") vizMarker.type = visualization_msgs::Marker::SPHERE;
    else if (type == "cylinder") vizMarker.type = visualization_msgs::Marker::CYLINDER;
    else if (type == "linelist") vizMarker.type = visualization_msgs::Marker::LINE_LIST;
    else if (type == "linestrip") vizMarker.type = visualization_msgs::Marker::LINE_STRIP;
    else if (type == "points") vizMarker.type = visualization_msgs::Marker::POINTS;
    else vizMarker.type = visualization_msgs::Marker::ARROW;

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

  void InitSimulatorVizMarkers() {
    geometry_msgs::PoseStamped p;
    geometry_msgs::Point32 scale;
    vector<float> color(4, 0.0f);

    p.header.frame_id = "map";
    p.pose.orientation.w = 1.0f;
    scale.x = 0.02f; scale.y = 0.f; scale.z = 0.f;
    color[0] = 66.f / 255.f; color[1] = 134.f / 255.f; color[2] = 244.f / 255.f; color[3] = 1.f;
    InitVizMarker(line_list_marker_, "map_lines", 0, "linelist", p, scale, 0.0, color);

    p.pose.position.z = 0.0;
    scale.x = 0.5f; scale.y = 0.44f; scale.z = 0.5f;
    color[0] = 94.f / 255.f; color[1] = 156.f / 255.f; color[2] = 255.f / 255.f; color[3] = 0.8f;
    InitVizMarker(pose_marker_, "robot_position", 1, "cube", p, scale, 0.0, color);

    scale.x = 0.05f; scale.y = 0.05f; scale.z = 0.05f;
    InitVizMarker(target_marker_, "targets", 1, "points", p, scale, 0.0, color);
  }

  void PublishVisualizationMarkers() {
    // If you previously had separate publishers for e.g. "map_lines_publisher_"
    // to publish "line_list_marker_", replicate that here. 
    // For demonstration, we show how you might update pose_marker_:
    tf::Quaternion robotQ = tf::createQuaternionFromYaw(current_angle_);
    float rear_axle_offset = 0.0f;
    pose_marker_.pose.position.x =
        current_loc_.x() - cos(current_angle_) * rear_axle_offset;
    pose_marker_.pose.position.y =
        current_loc_.y() - sin(current_angle_) * rear_axle_offset;
    pose_marker_.pose.position.z = 0.25f; 
    pose_marker_.pose.orientation.x = robotQ.x();
    pose_marker_.pose.orientation.y = robotQ.y();
    pose_marker_.pose.orientation.z = robotQ.z();
    pose_marker_.pose.orientation.w = robotQ.w();
    // Then publish if you have a dedicated marker publisher or incorporate it 
    // into local_viz_msg_ or global_viz_msg_ as needed.
  }
};

// The factory method picks ROS1 when compiled with -DROS1.
std::unique_ptr<RosAdapter> RosAdapter::create(navigation::Navigation* nav) {
  return std::make_unique<Ros1AdapterImpl>(nav);
}

#endif  // ROS1
