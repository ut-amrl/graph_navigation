#include "deep_cost_map_evaluator_service.h"

#include <opencv2/core/eigen.hpp>
#include <config_reader/config_reader.h>
#include <glog/logging.h>
#include <torch/script.h>

CONFIG_STRING(service_name, "DeepCostMapEvaluatorService.service_name");

// COSTMAP PARAMS
CONFIG_INT(center_crop_x, "DeepCostMapEvaluatorService.crop_params.center_x");
CONFIG_INT(center_crop_y, "DeepCostMapEvaluatorService.crop_params.center_y");
CONFIG_INT(center_crop_width, "DeepCostMapEvaluatorService.crop_params.width");
CONFIG_INT(center_crop_height, "DeepCostMapEvaluatorService.crop_params.height");
CONFIG_INT(output_width, "DeepCostMapEvaluatorService.crop_params.output_width");
CONFIG_INT(output_height, "DeepCostMapEvaluatorService.crop_params.output_height");
// DEFINE_int32(bev_input_width, 1280, "Width of input BEV image");
// DEFINE_int32(bev_input_height, 640, "Height of input BEV image");

CONFIG_INT(bev_pixels_per_meter, "DeepCostMapEvaluatorService.bev_pixels_per_meter");
CONFIG_FLOAT(min_cost, "DeepCostMapEvaluatorService.min_cost");
CONFIG_FLOAT(max_cost, "DeepCostMapEvaluatorService.max_cost");
CONFIG_FLOAT(discount_factor, "DeepCostMapEvaluatorService.discount_factor");
CONFIG_FLOAT(rollout_density, "DeepCostMapEvaluatorService.rollout_density");

CONFIG_FLOAT(robot_inflation_radius, "DeepCostMapEvaluatorService.robot_inflation_radius");
CONFIG_FLOAT(costmap_inflation_rate, "DeepCostMapEvaluatorService.costmap_inflation_rate");

// COSTMAP WEIGHTS
CONFIG_FLOAT(dist_to_goal_weight, "DeepCostMapEvaluatorService.dist_to_goal_weight");
CONFIG_FLOAT(clearance_weight, "DeepCostMapEvaluatorService.clearance_weight");
CONFIG_FLOAT(clearance_weight_beta, "DeepCostMapEvaluatorService.clearance_weight_beta");
CONFIG_FLOAT(fpl_weight, "DeepCostMapEvaluatorService.fpl_weight");
CONFIG_FLOAT(learned_weight, "DeepCostMapEvaluatorService.learned_weight");
CONFIG_FLOAT(learned_weight_beta, "DeepCostMapEvaluatorService.learned_weight_beta");
CONFIG_FLOAT(angle_weight, "DeepCostMapEvaluatorService.angle_weight");

// PHYSICAL PARAMS
CONFIG_FLOAT(base_link_offset_x, "DeepCostMapEvaluatorService.base_link_offset_x");
CONFIG_FLOAT(base_link_offset_y, "DeepCostMapEvaluatorService.base_link_offset_y");

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
  : params_(params), service_request_ongoing_(false) {
  // Initialize ROS service client
  service_client_ = nh_.serviceClient<amrl_msgs::CostmapSrv>(CONFIG_service_name);
  latest_vis_bevimage_ = cv::Mat3b(1, 1, cv::Vec3b(0, 0, 0));
  latest_vis_rgbimage_ = cv::Mat3b(1, 1, cv::Vec3b(0, 0, 0));

  // Check that matrices are not empty
  if (params_.K.empty() || params_.D.empty() || params_.H.empty() || params_.R.empty() || params_.P.empty(), params_.W.empty()) {
    LOG(FATAL) << "Camera calibration matrices are empty!";
  }
}

void DeepCostMapEvaluatorService::UpdateImage(const cv::Mat& image) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Undistort and rectify image using camera calibrations
  cv::Mat undistorted_rectified_image;
  if (map1_.empty() || map2_.empty()) {
    cv::initUndistortRectifyMap(
        params_.K,    // Camera intrinsic matrix
        params_.D,    // Distortion coefficients
        params_.R,    // Rectification transformation 
        params_.P,    // New camera matrix (projection matrix)
        image.size(),   // Size of the input image
        CV_32FC1,       // Type of the first output map
        map1_, map2_      // Output maps for remap
    );
  }
  // Apply the undistortion and rectification transformation.
  cv::remap(
      image, 
      undistorted_rectified_image, 
      map1_, 
      map2_, 
      cv::INTER_LINEAR
  );
  latest_image_ = undistorted_rectified_image;
}

std::shared_ptr<PathRolloutBase> DeepCostMapEvaluatorService::FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) {
  cv::Mat1f latest_costmap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_costmap = latest_costmap_;
  }

  //0 Transform map to latest odom frame
  this->UpdateMap(odom_);

  //1 Compute learned costs
  vector<float> learned_path_costs(paths.size(), 0.0f);
  if (!latest_costmap_.empty()) {
    learned_path_costs = GetLearnedCosts(paths, latest_costmap);
  }

  //2 Don't consider paths with endpoints blocked from end of local path to local target
  vector<float> clearance_to_goal(paths.size(), 0.0);
  vector<float> dist_to_goal(paths.size(), FLT_MAX);
  vector<float> angle_to_goal(paths.size(), 0.0);
  bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] =
        StraightLineClearance(Line2f(endpoint, local_target), point_cloud);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - local_target).norm();
      path_to_goal_exists = true;
    }

    // Compute angle to goal
    angle_to_goal[i] = VectorAngle(local_target, endpoint);
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

  // Normalize cost terms
  std::vector<float> raw_lengths(paths.size(), 0.0f);
  std::vector<float> raw_dists(paths.size(), 0.0f);
  std::vector<float> raw_clearances(paths.size(), 0.0f);
  std::vector<float> raw_angles(paths.size(), 0.0f);
  for (size_t i = 0; i < paths.size(); ++i) {
    raw_lengths[i] = paths[i]->Length();
    raw_dists[i] = dist_to_goal[i];
    raw_clearances[i] = ClearanceCost(paths[i]);
    raw_angles[i] = angle_to_goal[i];
  }
  auto [min_length_it, max_length_it] = std::minmax_element(raw_lengths.begin(), raw_lengths.end());
  auto [min_dist_it, max_dist_it] = std::minmax_element(raw_dists.begin(), raw_dists.end());
  auto [min_clearance_it, max_clearance_it] = std::minmax_element(raw_clearances.begin(), raw_clearances.end());
  auto [min_angle_it, max_angle_it] = std::minmax_element(raw_angles.begin(), raw_angles.end());
  float max_length = *max_length_it;
  float min_length = *min_length_it;
  float max_dist = *max_dist_it;
  float min_dist = *min_dist_it;
  float max_clearance = *max_clearance_it;
  float min_clearance = *min_clearance_it;
  float max_angle = *max_angle_it;
  float min_angle = *min_angle_it;

  // Compute normalized path lengths, distances, and clearances
  std::vector<float> norm_lengths(paths.size(), 0.0f);
  std::vector<float> norm_dists(paths.size(), 0.0f);
  std::vector<float> norm_clearances(paths.size(), 0.0f);
  std::vector<float> norm_angles(paths.size(), 0.0f);
  for (size_t i = 0; i < paths.size(); ++i) {
    norm_lengths[i] = (raw_lengths[i] - min_length) / (max_length - min_length + 1e-6);
    norm_dists[i] = (raw_dists[i] - min_dist) / (max_dist - min_dist + 1e-6);
    norm_clearances[i] = (raw_clearances[i] - min_clearance) / (max_clearance - min_clearance + 1e-6);
    norm_angles[i] = (raw_angles[i] - min_angle) / (max_angle - min_angle + 1e-6);
  }

  //4 Next try to find better paths.
  // auto best_path_it = std::find(paths.begin(), paths.end(), best);
  // size_t best_index = std::distance(paths.begin(), best_path_it);
  /** 
  float best_cost = \
    CONFIG_dist_to_goal_weight * norm_dists[best_index] + \
    CONFIG_fpl_weight * norm_lengths[best_index] + \
    CONFIG_clearance_weight * norm_clearances[best_index] + \
    CONFIG_learned_weight * learned_path_costs[best_index] + \
    CONFIG_angle_weight * norm_angles[best_index];
  */
  float max_cost = FLT_MIN;
  vector<float> path_costs;
  path_costs.resize(paths.size(), FLT_MAX);
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    // const float path_length =
    //     (path_to_goal_exists ? (paths[i]->Length() + dist_to_goal[i])
    //                          : dist_to_goal[i]);
    const float cost = \
      CONFIG_dist_to_goal_weight * norm_dists[i] + \
      CONFIG_fpl_weight * norm_lengths[i] + \
      CONFIG_clearance_weight * norm_clearances[i] + \
      CONFIG_learned_weight * learned_path_costs[i] + \
      CONFIG_angle_weight * norm_angles[i];

    max_cost = std::max(max_cost, cost);
    path_costs[i] = cost;
  }

  // Set paths with length 0 to max cost
  auto min_cost_it = std::min_element(path_costs.begin(), path_costs.end());
  float min_cost = *min_cost_it;

  // printf("Idx Cost\n");
  for (size_t i = 0; i < paths.size(); ++i) {
    if (path_costs[i] == FLT_MAX) {
      path_costs[i] = max_cost;
    }
    // printf("%ld %f\n", i, path_costs[i]);
  }

  // Normalize path costs
  for (size_t i = 0; i < paths.size(); ++i) {
    path_costs[i] = (path_costs[i] - min_cost) / (max_cost - min_cost + 1e-6);
  }

  // Select best path
  int best_index = std::distance(path_costs.begin(), std::min_element(path_costs.begin(), path_costs.end()));
  best = paths[best_index];

  // Update latest_vis_image
  DrawPathCosts(paths, best_index);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path_costs_ = path_costs;
  }
  return best;
}

float DeepCostMapEvaluatorService::ClearanceCost(const shared_ptr<PathRolloutBase> &path) {
  return CONFIG_clearance_weight * exp(-CONFIG_clearance_weight_beta * path->Clearance());
}

float DeepCostMapEvaluatorService::ComputeLearnedCost(float cost) {
  // printf("Learned cost: %f\n", cost);
  // printf("Resulting cost: %f\n", CONFIG_learned_weight * exp(-CONFIG_learned_weight_beta * cost));
  return cost;
}

Eigen::Vector2f DeepCostMapEvaluatorService::StateToPixel(const Eigen::Vector2f& state) {
  if (latest_costmap_.empty()) {
    return Eigen::Vector2f(0.0f, 0.0f);
  }
  const Eigen::Vector2f P_image_robot(
    (latest_costmap_.cols / 2) + CONFIG_base_link_offset_y * CONFIG_bev_pixels_per_meter, 
    (latest_costmap_.rows / 1) + CONFIG_base_link_offset_x * CONFIG_bev_pixels_per_meter);
  const Eigen::Vector2f& P_image_rel = 
    Eigen::Vector2f(-state(1), -state(0)) * CONFIG_bev_pixels_per_meter;
  const Eigen::Vector2f P_image = P_image_robot + P_image_rel;
  return P_image;
}

bool DeepCostMapEvaluatorService::ImageBoundCheck(const Eigen::Vector2i& pixel, const cv::Mat& costmap) {
  return pixel.x() >= 0 && pixel.x() < costmap.cols && pixel.y() >= 0 && pixel.y() < costmap.rows;
}

std::vector<Eigen::Vector2f> DeepCostMapEvaluatorService::GetWheelLocations(const pose_2d::Pose2Df& pose, float robot_width, float robot_length) {
  std::vector<Eigen::Vector2f> image_locs;
  // center
  Eigen::Vector2f center_loc = StateToPixel(pose.translation);
  image_locs.push_back(center_loc);

  float robot_length_half = robot_length * CONFIG_robot_inflation_radius / 2;
  float robot_width_half = robot_width * CONFIG_robot_inflation_radius / 2;
  // front left wheel
  Eigen::Vector2f fl_vec(robot_length_half, robot_width_half);
  Eigen::Vector2f fl_loc = StateToPixel(pose.translation + Eigen::Rotation2Df(pose.angle) * fl_vec);
  image_locs.push_back(fl_loc);
  // front right wheel
  Eigen::Vector2f fr_vec(robot_length_half, -robot_width_half);
  Eigen::Vector2f fr_loc = StateToPixel(pose.translation + Eigen::Rotation2Df(pose.angle) * fr_vec);
  image_locs.push_back(fr_loc);
  // back left wheel
  Eigen::Vector2f bl_vec(-robot_length_half, robot_width_half);
  Eigen::Vector2f bl_loc = StateToPixel(pose.translation + Eigen::Rotation2Df(pose.angle) * bl_vec);
  image_locs.push_back(bl_loc);
  //back right wheel
  Eigen::Vector2f br_vec(-robot_length_half, -robot_width_half);
  Eigen::Vector2f br_loc = StateToPixel(pose.translation + Eigen::Rotation2Df(pose.angle) * br_vec);
  image_locs.push_back(br_loc);

  return image_locs;
}

std::vector<float> DeepCostMapEvaluatorService::GetLearnedCosts(
  const std::vector<std::shared_ptr<PathRolloutBase>>& paths, const cv::Mat1f& costmap) 
{
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
      // const auto& pixel = StateToPixel(pose, costmap).cast<int>();
      const auto &wheels = GetWheelLocations(pose, params_.robot_width, params_.robot_length);

      for (const auto& wheelf : wheels) {
        const auto& wheeli = wheelf.cast<int>();
        // Ensure pixel coordinates are within image bounds
        if (ImageBoundCheck(wheeli, costmap)) {
          float cost = costmap.at<float>(wheeli.y(), wheeli.x());

          // Compute the discount based on distance along the path
          float discount = std::pow(CONFIG_discount_factor, f * path_length);
          // float discount = std::pow(CONFIG_discount_factor, pose.translation.norm());

          // Accumulate the discounted cost
          total_cost += discount * cost;
        }
      }
    }
    // Store the total cost for the current path
    learned_path_costs.push_back(total_cost);
  }

  // Ampilify difference between costs and then renormalize
  float min_cost = *std::min_element(learned_path_costs.begin(), learned_path_costs.end());
  float max_cost = *std::max_element(learned_path_costs.begin(), learned_path_costs.end());
  // printf("Old path costs: \n");
  for (auto& cost : learned_path_costs) {
    cost = (cost - min_cost) / (max_cost - min_cost);
    // printf("%f\n", cost);
    cost = exp(CONFIG_learned_weight_beta * cost);
  } 
  // Renoemalize costs to 0 - 1
  min_cost = *std::min_element(learned_path_costs.begin(), learned_path_costs.end());
  max_cost = *std::max_element(learned_path_costs.begin(), learned_path_costs.end());
  // printf("New path costs: \n");
  for (auto& cost : learned_path_costs) {
    cost = (cost - min_cost) / (max_cost - min_cost);
    // printf("%f\n", cost);
  }

  learned_path_costs_ = learned_path_costs;
  return learned_path_costs;
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
    // cost_map = prev_costmap_;
    // prev_odom = prev_odom_;
    cost_map = persistent_costmap_;
    prev_odom = persistent_odom_;
  }

  // Update latest costmap and transform to local frame
  latest_costmap_ = this->UpdateMapToLocalFrame(cost_map, prev_odom, odom);
  // latest_costmap_ = this->UpdateMapToLocalFrame(cost_map, prev_odom, odom);
}

// void DeepCostMapEvaluatorService::AccumulateCostmap(const cv::Mat1f& newCostmap, const Odom& newOdom) {
//   // If no persistent costmap exists, initialize it with the current costmap.
//   if (persistent_costmap_.empty()) {
//     persistent_costmap_ = newCostmap.clone();
//     persistent_odom_ = newOdom;
//     return;
//   }

//   // --- Step 1: Transform the persistent costmap into the current frame ---
//   // Use your helper function to transform persistent_costmap_ from persistent_odom_ to newOdom.
//   cv::Mat1f warpedPersistent = UpdateMapToLocalFrame(persistent_costmap_, persistent_odom_, newOdom);

//   // // Merge the transformed persistent costmap with the new costmap.
//   cv::Mat1f mergedCostmap = newCostmap.clone();
//   cv::Mat1b maskNew = newCostmap == 255.0f;

//   for (int y = 0; y < mergedCostmap.rows; ++y) {
//     for (int x = 0; x < mergedCostmap.cols; ++x) {
//       if (maskNew.at<uchar>(y, x)) {
//         mergedCostmap.at<float>(y, x) = warpedPersistent.at<float>(y, x);
//       }
//       // mergedCostmap.at<float>(y, x) = newCostmap.at<float>(y, x) * CONFIG_map_merge_discount_factor +
//       //                                 warpedPersistent.at<float>(y, x) * (1.0f - CONFIG_map_merge_discount_factor);
//       // if (maskNew.at<uchar>(y, x) == 255) {
//         // mergedCostmap.at<float>(y, x) = warpedPersistent.at<float>(y, x);
//       // }
//     }
//   }

//   // --- Step 4: Update persistent costmap and its associated odometry ---
//   persistent_costmap_ = mergedCostmap.clone();
//   persistent_odom_ = newOdom;
// }

void DeepCostMapEvaluatorService::AccumulateCostmap(const cv::Mat1f& newCostmap, const Odom& newOdom) {
  // If no persistent costmap exists, initialize it with the current costmap.
  if (persistent_costmap_.empty()) {
    persistent_costmap_ = newCostmap.clone();
    persistent_odom_ = newOdom;
    return;
  }

  // 2) Transform the old persistent map into the new frame.
  cv::Mat1f warpedPersistent =
      UpdateMapToLocalFrame(persistent_costmap_, persistent_odom_, newOdom);

   // 3) Merge:
  //    - If newCost == 255 => keep old cost
  //    - Else => inflate old cost slightly, then take min(old, new).
  //      That way old cost gradually approaches 255 if it stays outside the FOV.
  //      Over ~N steps, oldVal can become large but never exceed newVal if newVal is smaller.
  cv::Mat1f mergedCostmap = newCostmap.clone();

  // Suppose we want the old cost to reach 255 over 30 steps. Then an increment might be:
  float inflationPerStep = 255.0f / CONFIG_costmap_inflation_rate;
  // Adjust as needed (could be a constant like 5.0f, etc.)

  for (int y = 0; y < mergedCostmap.rows; ++y) {
    for (int x = 0; x < mergedCostmap.cols; ++x) {
      float newVal = newCostmap.at<float>(y, x);
      float oldVal = warpedPersistent.at<float>(y, x);

      if (newVal == 255.0f) {
        // The new cost is unknown => keep the old cost entirely.
        mergedCostmap.at<float>(y, x) = oldVal;

      } else {
        // The new cost is known => inflate the old cost a bit, 
        // then take the min with newVal.
        float inflatedOld = std::min(oldVal + inflationPerStep, 255.0f);
        mergedCostmap.at<float>(y, x) = std::min(inflatedOld, newVal);
      }
    }
  }

  // --- Step 5: Update persistent costmap and odometry ---
  persistent_costmap_ = mergedCostmap.clone();
  persistent_odom_ = newOdom;
}

cv::Mat1f DeepCostMapEvaluatorService::UpdateMapToLocalFrame(const cv::Mat1f& costmap, const Odom& prev_odom, const Odom& odom) {
  auto prev_transform = prev_odom.toAffine2f();
  auto new_transform = odom.toAffine2f();
  Eigen::Affine2f delta_transform = prev_transform.inverse() * new_transform;

  cv::Mat new_costmap = costmap.clone();
  cv::Mat flipped_image;
  cv::flip(new_costmap, flipped_image, 0);

  cv::Mat translation_mat;
  Eigen::Matrix2f eigen_rot = Eigen::Rotation2Df(-M_PI_2) *
                              delta_transform.rotation().matrix().inverse() *
                              Eigen::Rotation2Df(M_PI_2);
  Eigen::Vector2f eigen_trans = -eigen_rot * Eigen::Rotation2Df(-M_PI_2) *
                                delta_transform.translation().matrix();
  eigen_trans = eigen_trans.cwiseProduct(Eigen::Vector2f{40, 40});
  cv::eigen2cv(eigen_trans, translation_mat);

  Eigen::Rotation2Df rot;
  rot.fromRotationMatrix(eigen_rot);

  auto transform_matrix = cv::getRotationMatrix2D(
      cv::Point2f{(float)CONFIG_output_width / 2, (float)CONFIG_output_height},
      -rot.angle() * (180 / M_PI), 1.0);

  transform_matrix(cv::Rect(2, 0, 1, 2)) -= translation_mat;

  auto prev_map = flipped_image.clone();
  flipped_image.setTo(0);
  cv::warpAffine(prev_map, flipped_image, transform_matrix,
                 flipped_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                 cv::Scalar(255.0f));
  new_costmap.setTo(0);
  cv::flip(flipped_image, new_costmap, 0);
  return new_costmap;
}

// cv::Mat1f DeepCostMapEvaluatorService::UpdateMapToLocalFrame(
//     const cv::Mat1f& costmap,
//     const Odom& prev_odom,
//     const Odom& odom) 
// {
//   // 1) Build old->new in your odometry frame. 
//   //    Or new->old if that is your convention; the key is consistency.
//   //    Suppose you want old->new:
//   Eigen::Affine2f T_old2new = odom.toAffine2f() * prev_odom.toAffine2f().inverse();

//   // 2) Insert a rotation for coordinate mismatch.
//   //    +x in odom might be "up" in the image =>  -90 deg rotation or +90 deg, etc.
//   //    Adjust angle to suit your system. For example, a -90 deg:
//   Eigen::Affine2f R_correction(Eigen::Rotation2Df(-M_PI_2));

//   // 3) Multiply them to incorporate that rotation into the final transform.
//   //    If you want to rotate first, then apply T_old2new, or T_old2new then rotate. 
//   //    Typically you do R * T if you want to "post-rotate" the entire transform.
//   Eigen::Affine2f T_corrected = R_correction * T_old2new;

//   // 4) Convert to 2×3 for warpAffine.
//   Eigen::Matrix<float, 2, 3> block2x3 = T_corrected.matrix().block<2,3>(0,0);

//   // Optionally multiply the last column by your scale factor if needed:
//   float scale = 40.0f; // e.g. 100 px/m
//   block2x3(0,2) *= scale;
//   block2x3(1,2) *= scale;

//   cv::Mat1f cv_aff(2, 3, CV_32F);
//   for (int r = 0; r < 2; ++r) {
//     for (int c = 0; c < 3; ++c) {
//       cv_aff(r, c) = block2x3(r, c);
//     }
//   }

//   // 5) Warp the old costmap.
//   //    Fill out-of-bounds with 255.
//   cv::Mat transformed;
//   cv::warpAffine(costmap, transformed,
//                  cv_aff,
//                  costmap.size(),
//                  cv::INTER_LINEAR,
//                  cv::BORDER_CONSTANT,
//                  255.0f);

//   return transformed;
// }

// void UpdateFrame(const cv::Mat& costmap, const Odom& prev_odom, const Odom& odom) {
//   auto current_transform = prev_odom.ToAffine2f();
//   auto new_transform = odom.ToAffine2f();
//   Eigen::Affine2f delta_transform = current_transform.inverse() * new_transform;

//   cv::Mat flipped_image;
//   cv::flip(bev_image_, flipped_image, 0);

//   cv::Mat translation_mat;
//   Eigen::Matrix2f eigen_rot = Eigen::Rotation2Df(-M_PI_2) *
//                               delta_transform.rotation().matrix().inverse() *
//                               Eigen::Rotation2Df(M_PI_2);
//   Eigen::Vector2f eigen_trans = -eigen_rot * Eigen::Rotation2Df(-M_PI_2) *
//                                 delta_transform.translation().matrix();
//   eigen_trans = eigen_trans.cwiseProduct(Eigen::Vector2f{100, 100});
//   cv::eigen2cv(eigen_trans, translation_mat);

//   Eigen::Rotation2Df rot;
//   rot.fromRotationMatrix(eigen_rot);

//   auto transform_matrix = cv::getRotationMatrix2D(
//       cv::Point2f{(float)CONFIG_image_width / 2, (float)CONFIG_image_height},
//       -rot.angle() * (180 / M_PI), 1.0);

//   transform_matrix(cv::Rect(2, 0, 1, 2)) -= translation_mat;

//   auto prev_map = flipped_image.clone();
//   flipped_image.setTo(0);
//   cv::warpAffine(prev_map, flipped_image, transform_matrix,
//                  flipped_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
//                  0);
//   bev_image_.setTo(0);
//   cv::flip(flipped_image, bev_image_, 0);
// }

void DeepCostMapEvaluatorService::RequestMapUpdate(const Odom& odom) {
  try {
    amrl_msgs::CostmapSrv srv;

    // Prepare service request
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = "base_link";
    printf("Requesting deep cost map service\n");
    // Call the service
    if (service_client_.call(srv)) {
      if (srv.response.success.data) {
        printf("Deep cost map service returned a valid cost map\n");
        auto msg = srv.response.costmap;
        // Normalize costmap to 0 - 1
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat1f costmap = cv_ptr->image;

        // // Save costmap to image for visualization
        // cv::imwrite("costmap.png", costmap);

        // Resize and crop costmap to be larger
        cv::Mat1f cropped_costmap = CropAndResizeImage(
          costmap, cv::Point(CONFIG_center_crop_x, CONFIG_center_crop_y), 
          CONFIG_center_crop_width, CONFIG_center_crop_height, 
          CONFIG_output_width, CONFIG_output_height);

        // cv::imwrite("cropped_costmap.png", cropped_costmap);
        // Update shared cost map
        {
          std::lock_guard<std::mutex> lock(mutex_);
          prev_costmap_ = cropped_costmap;
          prev_odom_ = odom;

          // Accumulate the previous costmap into the current costmap.
          this->AccumulateCostmap(prev_costmap_, prev_odom_);

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
    const std::vector<std::shared_ptr<PathRolloutBase>>& paths, int best_index) {
  // Guard against empty images.
  if (latest_costmap_.empty() || latest_image_.empty()) {
    return;
  }
  const auto& best_path = paths[best_index];

  // Colorize costmap for visualization.
  cv::Mat cost_map_scaled;
  cv::normalize(latest_costmap_, cost_map_scaled, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::Mat latest_annotated_bev = cost_map_scaled.clone();
  cv::applyColorMap(latest_annotated_bev, latest_annotated_bev, cv::COLORMAP_BONE);

  // Normalize path costs
  float min_cost = *std::min_element(path_costs_.begin(), path_costs_.end());
  float max_cost = *std::max_element(path_costs_.begin(), path_costs_.end());
  vector<float> normalized_costs(path_costs_.size(), 0.0f);
  // printf("Normalized costs: \n");
  for (size_t i = 0; i < path_costs_.size(); ++i) {
    normalized_costs[i] = (path_costs_[i] - min_cost) / (max_cost - min_cost + 1e-6);
    // printf("%f\n", normalized_costs[i]);
  }

  // Compute sorted indices of paths by increasing cost.
  std::vector<int> sorted_indices(path_costs_.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int i, int j) {
    return path_costs_[i] < path_costs_[j];
  });

  // Create a ranks array such that ranks[i] is the rank of path i.
  std::vector<int> ranks(path_costs_.size());
  for (size_t rank = 0; rank < sorted_indices.size(); ++rank) {
    int path_index = sorted_indices[rank];
    ranks[path_index] = static_cast<int>(rank);
  }
  // Draw paths on the overlay.
  cv::Mat latest_annotated_bgr = latest_image_.clone();
  Eigen::Matrix<float, 3, 4> W;
  cv::cv2eigen(params_.W, W);
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto& path = paths[i];
    const auto& rank = ranks[i];
    int thickness = (best_path == path) ? -1 : CONFIG_viz_thickness;
    // Or if you're just using path_costs_ directly:
    // float path_cost = normalized_costs[i];

    // cv::Vec3b color = GetColorFromCost(path_cost);
    cv::Vec3b color = GetColorFromRanking(rank, (int) paths.size());

    // Determine number of points to sample along the path.
    int num_samples = std::max(1,
        static_cast<int>(path->Length() * CONFIG_rollout_density));

    for (int j = 0; j <= num_samples; ++j) {
      float f = static_cast<float>(j) / num_samples;
      pose_2d::Pose2Df pose = path->GetIntermediateState(f);

      // Construct world point assuming ground plane height of -0.30.
      Eigen::Vector4f homogeneous_point(pose.translation.x(), pose.translation.y(), -0.3f, 1.0f);
      Eigen::Vector3f camera_point = W * homogeneous_point; // 3x1

      // Project onto image plane using camera intrinsics.
      float depth = camera_point.z();
      Eigen::Vector3f projected = camera_point / depth;
      cv::Point2f image_point(projected.x(), projected.y());

      // Draw a circle at the projected point on the RGB image.
      if (image_point.x >= 0 && image_point.x < latest_annotated_bgr.cols &&
          image_point.y >= 0 && image_point.y < latest_annotated_bgr.rows &&
          depth >= 0.05f
          ) {
        cv::circle(latest_annotated_bgr, image_point, CONFIG_viz_radius*2, color, thickness);
      }

      // Convert from path coordinates to pixel coordinates in overlay.
      Eigen::Vector2i pixel = StateToPixel(pose.translation).cast<int>();

      // Make sure it's in bounds. We check latest_vis_image_ now (the same size
      // as the BEV).
      if (ImageBoundCheck(pixel, latest_annotated_bev)) {
        cv::circle(latest_annotated_bev, cv::Point(pixel.x(), pixel.y()),
                   CONFIG_viz_radius, color, thickness);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Update the latest visualization images
    latest_vis_bevimage_ = latest_annotated_bev;
    latest_vis_rgbimage_ = latest_annotated_bgr;
  }
}

// Helper functrion to map ranks to color
cv::Vec3b DeepCostMapEvaluatorService::GetColorFromRanking(int rank, int num_paths) {
  // Map rank to a range [0, 255]
  float normalized_rank = static_cast<float>(rank) / num_paths * 255;

  // Return a color based on the normalized rank.
  // This is a simple gradient from green (low cost) to red (high cost).
  return cv::Vec3b(0, 255 - static_cast<int>(normalized_rank), static_cast<int>(normalized_rank));
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

cv::Mat DeepCostMapEvaluatorService::CropAndResizeImage(const cv::Mat& input_img,
                           const cv::Point& crop_center,
                           int crop_width,
                           int crop_height,
                           int out_width,
                           int out_height)
{
    if (input_img.empty()) {
        // Return an empty Mat if the input is invalid
        return cv::Mat();
    }

    // Center-based top-left
    int x = crop_center.x - crop_width / 2;
    int y = crop_center.y - crop_height / 2;

    // Clamp top-left to image boundaries
    x = std::max(0, x);
    y = std::max(0, y);

    if (x + crop_width > input_img.cols) {
        crop_width = input_img.cols - x;
    }
    if (y + crop_height > input_img.rows) {
        crop_height = input_img.rows - y;
    }

    // If the region is invalid (e.g., 0 or negative width/height), return empty
    if (crop_width <= 0 || crop_height <= 0) {
        return cv::Mat();
    }

    cv::Rect roi(x, y, crop_width, crop_height);
    cv::Mat cropped = input_img(roi).clone();  // clone() to get a copy

    cv::Mat output;
    cv::resize(cropped, output, cv::Size(out_width, out_height), 0, 0, cv::INTER_LINEAR);

    return output;
}


};