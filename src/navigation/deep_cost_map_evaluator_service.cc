#include "deep_cost_map_evaluator_service.h"


#include <config_reader/config_reader.h>
#include <glog/logging.h>
#include <torch/script.h>

CONFIG_STRING(service_name, "DeepCostMapEvaluatorService.service_name");

// COSTMAP PARAMS
CONFIG_INT(patch_pixel_pixels, "DeepCostMapEvaluatorService.patch_size_pixels");
CONFIG_INT(bev_pixels_per_meter, "DeepCostMapEvaluatorService.bev_pixels_per_meter");
CONFIG_FLOAT(min_cost, "DeepCostMapEvaluatorService.min_cost");
CONFIG_FLOAT(max_cost, "DeepCostMapEvaluatorService.max_cost");
CONFIG_FLOAT(discount_factor, "DeepCostMapEvaluatorService.discount_factor");
CONFIG_FLOAT(rollout_density, "DeepCostMapEvaluatorService.rollout_density");

// COSTMAP WEIGHTS
CONFIG_FLOAT(dist_to_goal_weight, "DeepCostMapEvaluatorService.dist_to_goal_weight");
CONFIG_FLOAT(clearance_weight, "DeepCostMapEvaluatorService.clearance_weight");
CONFIG_FLOAT(clearance_weight_beta, "DeepCostMapEvaluatorService.clearance_weight_beta");
CONFIG_FLOAT(fpl_weight, "DeepCostMapEvaluatorService.fpl_weight");
CONFIG_FLOAT(learned_weight, "DeepCostMapEvaluatorService.learned_weight");
CONFIG_FLOAT(learned_weight_beta, "DeepCostMapEvaluatorService.learned_weight_beta");

// VISUALIZATION SETTINGs
CONFIG_INT(viz_radius, "DeepCostMapEvaluatorService.viz_radius");
CONFIG_INT(viz_thickness, "DeepCostMapEvaluatorService.viz_thickness");

using navigation::Odom;
using geometry::Line2f;
using std::shared_ptr;
using std::vector;
using namespace geometry;
using namespace math_util;

namespace motion_primitives {

DeepCostMapEvaluatorService::DeepCostMapEvaluatorService(const navigation::NavigationParameters& params)
  : params_(params), service_request_ongoing_(false), has_map_(false) {
  // Initialize ROS service client
  service_client_ = nh_.serviceClient<amrl_msgs::CostmapSrv>(CONFIG_service_name);
  latest_vis_image_ = cv::Mat3b(1, 1, cv::Vec3b(0, 0, 0));
}

std::shared_ptr<PathRolloutBase> DeepCostMapEvaluatorService::FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) {
  cv::Mat1f latest_costmap;
  bool has_map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_costmap = latest_costmap_;
    has_map = has_map_;
  }

  //0 Transform map to latest odom frame
  this->UpdateMap(odom_);

  //1 Compute learned costs
  vector<float> learned_path_costs(paths.size(), 0.0f);
  if (has_map) {
    learned_path_costs = GetLearnedCosts(paths, latest_costmap);
  }

  //2 Don't consider paths with endpoints blocked from end of local path to local target
  vector<float> clearance_to_goal(paths.size(), 0.0);
  vector<float> dist_to_goal(paths.size(), FLT_MAX);
  bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] =
        StraightLineClearance(Line2f(endpoint, local_target), point_cloud);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - local_target).norm();
      path_to_goal_exists = true;
    }
  }

  //3 First find the shortest path.
  shared_ptr<PathRolloutBase> best = nullptr;
  float best_path_length = FLT_MAX;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length =
        (path_to_goal_exists ? (paths[i]->Length() + dist_to_goal[i])
                             : dist_to_goal[i]);
    if (path_length < best_path_length) {
      best_path_length = path_length;
      best = paths[i];
    }
  }

  if (best == nullptr) {
    printf("No valid path found\n");
    // No valid paths!
    return nullptr;
  }

  //4 Next try to find better paths.
  auto best_path_it = std::find(paths.begin(), paths.end(), best);
  size_t best_index = std::distance(paths.begin(), best_path_it);
  float best_cost = CONFIG_dist_to_goal_weight * (best_path_length) + \
                    CONFIG_fpl_weight * best->Length() + \
                    ClearanceCost(best) + \
                    LearnedCost(learned_path_costs[best_index]);
  path_costs_.resize(paths.size(), 0.0f);
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length =
        (path_to_goal_exists ? (paths[i]->Length() + dist_to_goal[i])
                             : dist_to_goal[i]);
    const float cost =  ClearanceCost(paths[i]) + \
      CONFIG_dist_to_goal_weight * path_length + \
      CONFIG_fpl_weight * paths[i]->Length() + \
      LearnedCost(learned_path_costs[i]);

    path_costs_[i] = cost;
    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
    }
  }

  // Update latest_vis_image
  DrawPathCosts(paths, best);
  return best;
}

float DeepCostMapEvaluatorService::ClearanceCost(const shared_ptr<PathRolloutBase> &path) {
  return CONFIG_clearance_weight * exp(-CONFIG_clearance_weight_beta * path->Clearance());
}

float DeepCostMapEvaluatorService::LearnedCost(float cost) {
  return CONFIG_learned_weight * exp(-CONFIG_learned_weight_beta * cost);
}

std::vector<float> DeepCostMapEvaluatorService::GetLearnedCosts(
  const std::vector<std::shared_ptr<PathRolloutBase>>& paths, const cv::Mat1f& cost_map) {

  // Iterate through each candidate path
  std::vector<float> learned_path_costs;
  for (const auto& path : paths) {
    float total_cost = 0.0f;
    float path_length = path->Length();

    // Determine the number of sampling steps along the path based on rollout density
    int steps = std::max(1, static_cast<int>(path_length * CONFIG_rollout_density));

    for (int i = 0; i <= steps; ++i) {
      // Fraction along the path [0, 1]
      float f = static_cast<float>(i) / steps;

      // Retrieve the intermediate state at fraction f along the path
      pose_2d::Pose2Df pose = path->GetIntermediateState(f);

      // Map the pose to pixel coordinates in the cost map.
      // Assumption: world origin corresponds to the center of the image,
      // and y-axis inversion because image coordinates have y increasing downward.
      int px = static_cast<int>(-pose.translation(0) * CONFIG_bev_pixels_per_meter + cost_map.rows / 2);
      int py = static_cast<int>(-pose.translation(1) * CONFIG_bev_pixels_per_meter + cost_map.cols / 2);

      // Ensure pixel coordinates are within image bounds
      if (px >= 0 && px < cost_map.rows && py >= 0 && py < cost_map.cols) {
        float cost = cost_map(py, px);

        // Compute the discount based on distance along the path
        float discount = std::pow(CONFIG_discount_factor, f * path_length);

        // Accumulate the discounted cost
        total_cost += discount * cost;
      }
    }

    // Store the total cost for the current path
    learned_path_costs.push_back(total_cost);
  }

  learned_path_costs_ = learned_path_costs;
  return learned_path_costs;
}

std::vector<float> DeepCostMapEvaluatorService::GetLearnedPathCosts() const {
  return learned_path_costs_;
}

void DeepCostMapEvaluatorService::UpdateMap(const Odom& odom) {
  // Initiate a service request for new costmap if no request is ongoing
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!service_request_ongoing_) {
      service_request_ongoing_ = true;
      std::thread(&DeepCostMapEvaluatorService::RequestMapUpdate, this, odom).detach();
    }
  }

  // Wait until a valid previous cost map is available (optional, based on design).
  if (prev_costmap_.empty()) {
    printf("No previous cost map to transform\n");
    return;
  }
  // Transform the map from latest_odom_msg_ frame to current odom frame
  cv::Mat1f cost_map;
  Odom prev_odom;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cost_map = prev_costmap_;
    prev_odom = prev_odom_;
  }
  this->UpdateMapToLocalFrame(cost_map, prev_odom, odom);
}

void DeepCostMapEvaluatorService::UpdateMapToLocalFrame(const cv::Mat1f& costmap, const Odom& prev_odom, const Odom& odom) {
    // Compute the affine transform from previous odometry to current odometry.
    // This gives the transformation in the coordinate frame of the previous pose.
    Eigen::Affine2f T_curr_prev = prev_odom.toAffine2f().inverse() * odom.toAffine2f();

    // Convert Eigen::Affine2f to a 2x3 matrix for OpenCV
    Eigen::Matrix<float, 2, 3> eigen_affine = T_curr_prev.matrix().block<2,3>(0,0);

    // Create a cv::Mat from the Eigen matrix.
    // OpenCV uses CV_32F for single-precision float matrices.
    cv::Mat1f cv_affine(2, 3, CV_32F);
    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 3; ++c) {
            cv_affine.at<float>(r, c) = eigen_affine(r, c);
        }
    }

    // Apply the affine transformation to the costmap.
    cv::Mat transformed;
    cv::warpAffine(costmap, transformed, cv_affine, costmap.size(), 
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Update the original costmap with the transformed result.
    latest_costmap_ = transformed;
    has_map_ = true;
}

void DeepCostMapEvaluatorService::RequestMapUpdate(const Odom& odom) {
  try {
    amrl_msgs::CostmapSrv srv;

    // Prepare service request
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = "base_link";
    printf("Requesting deep cost map service");
    // Call the service
    if (service_client_.call(srv)) {
      if (srv.response.success.data) {
        printf("Deep cost map service returned a valid cost map");
        auto msg = srv.response.costmap;
        // Normalize costmap to 0 - 1
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat1f costmap = cv_ptr->image;
        cv::normalize(costmap, costmap, CONFIG_min_cost, CONFIG_max_cost, cv::NORM_MINMAX);
        // Update shared cost map
        {
          std::lock_guard<std::mutex> lock(mutex_);
          prev_costmap_ = costmap;
          prev_odom_ = odom;
        }
        printf("Completed deep cost map service");
        cv_.notify_all();
      } else {
        cv_.notify_all();
        ROS_ERROR("Deep cost map service failed to return a valid cost map");
      }
    } else {
      ROS_ERROR("Failed to call deep cost map service");
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in deep cost map service: %s", e.what());
  }

  // Mark service request as completed
  {
    std::lock_guard<std::mutex> lock(mutex_);
    service_request_ongoing_ = false;
  }
}

void DeepCostMapEvaluatorService::DrawPathCosts(
    const std::vector<std::shared_ptr<PathRolloutBase>>& paths,
    std::shared_ptr<PathRolloutBase> best_path) {
  if (latest_costmap_.empty()) {
    return;
  }
  
  // Clear the previous visualization image.
  latest_vis_image_ = cv::Mat3b(latest_costmap_.size(), cv::Vec3b(0, 0, 0));

  // Define a color map for visualizing path costs (e.g., from green to red).
  cv::Mat1b cost_map_scaled;
  cv::normalize(latest_costmap_, cost_map_scaled, 0, 255, cv::NORM_MINMAX);
  cv::applyColorMap(cost_map_scaled, latest_vis_image_, cv::COLORMAP_BONE);

  // TODO: debug this and why all costs are red
  // Normalize all costs linearly to be within [0, 1]
  std::vector<float> normalized_path_costs(path_costs_);
  const auto minmax_costs =
      std::minmax_element(normalized_path_costs.begin(), normalized_path_costs.end());
  const float min_cost = *minmax_costs.first;
  const float max_cost = *minmax_costs.second;
  for (float& cost : normalized_path_costs) {
    cost = (cost - min_cost) / (max_cost - min_cost);
  }


  // Iterate through all paths and draw circles at the intermediate points.
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto& path = paths[i];
    float path_cost = normalized_path_costs[i];

    // Determine number of points to sample along the path.
    int num_samples = std::max(1, static_cast<int>(path->Length() * CONFIG_rollout_density));

    // Use a gradient color based on the cost of the path.
    cv::Vec3b color = GetColorFromCost(path_cost);

    // Draw circles along the path.
    for (int j = 0; j <= num_samples; ++j) {
      float f = static_cast<float>(j) / num_samples;
      pose_2d::Pose2Df pose = path->GetIntermediateState(f);

      // Map pose to pixel coordinates in the cost map.
      int row = static_cast<int>(-pose.translation(0) * CONFIG_bev_pixels_per_meter + latest_costmap_.cols / 2);
      int col = static_cast<int>(-pose.translation(1) * CONFIG_bev_pixels_per_meter + latest_costmap_.rows / 2);

      // Ensure the pixel coordinates are within bounds before drawing.
      if (row >= 0 && row < latest_vis_image_.rows && col >= 0 && col < latest_vis_image_.cols) {
        cv::circle(latest_vis_image_, cv::Point(col, row), CONFIG_viz_radius, color, CONFIG_viz_thickness);
      }
    }

    // Highlight the best path with distinct circles (e.g., bright green).
    if (path == best_path) {
      cv::Vec3b best_color(0, 255, 0);  // Bright green for the best path
      int best_num_samples = std::max(1, static_cast<int>(best_path->Length() * CONFIG_rollout_density));
      for (int j = 0; j <= best_num_samples; ++j) {
        float f = static_cast<float>(j) / best_num_samples;
        pose_2d::Pose2Df pose = best_path->GetIntermediateState(f);

        int row = static_cast<int>(-pose.translation(0) * CONFIG_bev_pixels_per_meter + latest_costmap_.cols / 2);
        int col = static_cast<int>(-pose.translation(1) * CONFIG_bev_pixels_per_meter + latest_costmap_.rows / 2);

        if (row >= 0 && row < latest_vis_image_.rows && col >= 0 && col < latest_vis_image_.cols) {
          cv::circle(latest_vis_image_, cv::Point(col, row), CONFIG_viz_radius, best_color, CONFIG_viz_thickness);
        }
      }
    }
  }
}


// Helper function to convert cost to color
cv::Vec3b DeepCostMapEvaluatorService::GetColorFromCost(float cost) {
  // Map cost to a range [0, 255]
  float normalized_cost = std::min(std::max(cost, CONFIG_min_cost), CONFIG_max_cost);
  normalized_cost = (normalized_cost - CONFIG_min_cost) / (CONFIG_max_cost - CONFIG_min_cost) * 255;

  // Return a color based on the normalized cost.
  // This is a simple gradient from green (low cost) to red (high cost).
  return cv::Vec3b(0, 255 - static_cast<int>(normalized_cost), static_cast<int>(normalized_cost));
}


};