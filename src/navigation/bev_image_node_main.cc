#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>

#include <opencv2/opencv.hpp>

#include "config_reader/config_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_double(image_scale, 0.5, "Scale factor for the image");

// A simple signal handler so we can catch SIGINT
void SignalHandler(int signum) {
  ROS_INFO("Caught signal %d. Shutting down ROS.", signum);
  ros::shutdown();
}

DEFINE_string(robot_config, "config/navigation_pacer.lua", "Robot config file");

struct ImageToBEVParameters {
  std::string calibration_file;
  std::string image_topic;
  std::string bev_topic;
  cv::Mat H;
  int bev_image_width;
  int bev_image_height;
};

class ImageToBEVNode {
 public:
  ImageToBEVNode(const ros::NodeHandle& nh, const ImageToBEVParameters& params)
      : nh_(nh), params_(params) {
    // 1) Subscribe to the compressed image topic
    //    (Must use a ros::Subscriber, not image_transport::Subscriber, for
    //    /compressed topics)
    sub_ = nh_.subscribe<sensor_msgs::CompressedImage>(
        params_.image_topic, 1, &ImageToBEVNode::imageCallback, this);

    // 2) Publish to the compressed image topic
    pub_ = nh_.advertise<sensor_msgs::CompressedImage>(params_.bev_topic, 1);

    ROS_INFO_STREAM("Warping images from ["
                    << params_.image_topic
                    << "] "
                       "to BEV ["
                    << params_.bev_topic
                    << "] "
                       "with output size "
                    << params_.bev_image_width / FLAGS_image_scale << "x"
                    << params_.bev_image_height / FLAGS_image_scale);
  }

 private:
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // Decode compressed image data into a cv::Mat
    try {
      // Construct a temporary cv::Mat header for the compressed bytes in
      // msg->data
      cv::Mat compressed(1, msg->data.size(), CV_8UC1,
                         const_cast<unsigned char*>(msg->data.data()));
      // Decode
      cv::Mat decoded = cv::imdecode(compressed, cv::IMREAD_COLOR);
      if (decoded.empty()) {
        ROS_ERROR("Decoded image is empty. Cannot warp.");
        return;
      }

      // 3) Apply warp with homography
      cv::Mat warped;
      cv::warpPerspective(
          decoded, warped, params_.H,
          cv::Size(params_.bev_image_width, params_.bev_image_height),
          cv::INTER_LINEAR, cv::BORDER_CONSTANT,
          cv::Scalar(0, 0, 0)  // fill border with black
      );

      // Downsample image 2 times for efficiency
      cv::resize(warped, warped, cv::Size(), FLAGS_image_scale,
                 FLAGS_image_scale);

      // 4) Re-encode to compressed image for publishing
      std::vector<uchar> encoded;
      cv::imencode(".jpg", warped, encoded);  // or ".png"

      // Build the CompressedImage message
      sensor_msgs::CompressedImage bev_msg;
      bev_msg.header =
          msg->header;          // Use same timestamp and frame_id if desired
      bev_msg.format = "jpeg";  // or "png"
      bev_msg.data = encoded;

      // Publish
      pub_.publish(bev_msg);
    } catch (const cv::Exception& e) {
      ROS_ERROR("OpenCV exception during decode/warp: %s", e.what());
    }
  }

  ros::NodeHandle nh_;
  ImageToBEVParameters params_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

void LoadConfig(ImageToBEVParameters& params) {
  // Macros that let us read from the config
  CONFIG_STRING(image_topic, "ImageToBEVParameters.image_topic");
  CONFIG_STRING(bev_topic, "ImageToBEVParameters.bev_topic");
  CONFIG_STRING(calibration_file, "ImageToBEVParameters.calibration_file");
  CONFIG_INT(bev_image_width, "ImageToBEVParameters.bev_image_width");
  CONFIG_INT(bev_image_height, "ImageToBEVParameters.bev_image_height");
  config_reader::ConfigReader reader({FLAGS_robot_config});

  // Populate struct
  params.image_topic = CONFIG_image_topic;
  params.bev_topic = CONFIG_bev_topic;
  params.bev_image_width = CONFIG_bev_image_width;
  params.bev_image_height = CONFIG_bev_image_height;
  params.calibration_file = CONFIG_calibration_file;

  ROS_INFO("Opening calibration file: %s", params.calibration_file.c_str());
  // Load homography from the calibration file
  cv::FileStorage fs(params.calibration_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ROS_ERROR_STREAM(
        "Failed to open calibration file: " << params.calibration_file);
    ros::shutdown();
    return;
  }

  // Read the homography from the file node "H"
  cv::FileNode node = fs["H"];
  if (!node.empty()) {
    node >> params.H;  // reads the matrix
    ROS_INFO("Loaded homography from file: %s",
             params.calibration_file.c_str());
  } else {
    ROS_ERROR("Homography matrix 'H' not found in calibration file!");
  }

  fs.release();

  // Print final loaded parameters
  ROS_INFO("Loaded parameters:");
  ROS_INFO("  image_topic: %s", params.image_topic.c_str());
  ROS_INFO("  bev_topic: %s", params.bev_topic.c_str());
  ROS_INFO("  bev_image_width: %d", params.bev_image_width);
  ROS_INFO("  bev_image_height: %d", params.bev_image_height);
  ROS_INFO("  calibration_file: %s", params.calibration_file.c_str());
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  ros::init(argc, argv, "image_to_bev_node");
  ros::NodeHandle nh;

  ImageToBEVParameters params;
  LoadConfig(params);

  ImageToBEVNode node(nh, params);

  ros::spin();
  return 0;
}
