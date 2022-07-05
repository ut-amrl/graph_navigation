#include "rgb_cost_map_evaluator.h"

#include <opencv2/core/eigen.hpp>

#include "math/line2d.h"

namespace motion_primitives {

void RGBCostMapEvaluator::Update(
    const Eigen::Vector2f &new_loc, const float new_ang,
    const Eigen::Vector2f &new_vel, const float new_ang_vel,
    const Eigen::Vector2f &new_local_target,
    const std::vector<Eigen::Vector2f> &new_point_cloud,
    const cv::Mat &new_image) {
  static const cv::Point2f image_offset = {1280 / 2, 720};
  static const std::vector<cv::Point2f> src_pts = {
      {358, 512}, {472, 383}, {743, 378}, {835, 503}};
  static const std::vector<cv::Point2f> dst_pts = {
      cv::Point2f{-0.5, -1.5} * pixels_per_meter_ + image_offset,
      cv::Point2f{-0.5, -2.5} * pixels_per_meter_ + image_offset,
      cv::Point2f{0.5, -2.5} * pixels_per_meter_ + image_offset,
      cv::Point2f{0.5, -1.5} * pixels_per_meter_ + image_offset};
  static const cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
  static float vals[] = {1.0, 0.0, -240, 0.0, 1.0, -320};
  static const cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, vals);

  curr_loc = new_loc;
  curr_ang = new_ang;
  vel = new_vel;
  ang_vel = new_ang_vel;
  local_target = new_local_target;
  point_cloud = new_point_cloud;
  image = new_image;

  // TODO: move this to its own thread
  UpdateMapToLocalFrame();
  map_loc_ = new_loc;
  map_angle_ = new_ang;

  auto img_copy = image.clone();
  cv::cvtColor(img_copy, img_copy, cv::COLOR_BGR2BGRA);
  cv::Mat trans_img = cv::Mat::zeros(800, 800, CV_8UC4);
  cv::warpPerspective(img_copy, img_copy, M, img_copy.size());
  cv::warpAffine(img_copy, trans_img, translation_matrix, trans_img.size());
  for (int y = 0; y < rgb_map_.rows; y++) {
    for (int x = 0; x < rgb_map_.cols; x++) {
      auto img_pixel = trans_img.at<cv::Vec4b>(y, x);
      if (img_pixel[3] > 0) {
        rgb_map_.at<cv::Vec4b>(y, x) = img_pixel;
      }
    }
  }
  // rgb_map_ = trans_img;
  std::cout << rgb_map_.type() << "\t" << trans_img.type() << std::endl;
  cv::Mat vis;
  cv::circle(rgb_map_, {400, 400}, 5, 0x0000FF);
  // cv::hconcat(rgb_map_, trans_img, vis);
  cv::imshow("map", rgb_map_);
  cv::waitKey(1);

  // TODO: only evaluate on new portions and then add to cost map?
  (*cost_function)(rgb_map_, cost_map_);
}

void RGBCostMapEvaluator::UpdateMapToLocalFrame() {
  auto curr_trans =
      Eigen::Translation2f(map_loc_) * Eigen::Rotation2Df(map_angle_);
  auto new_trans =
      Eigen::Translation2f(curr_loc) * Eigen::Rotation2Df(curr_ang);
  Eigen::Affine2f delta_trans = curr_trans.inverse() * new_trans;

  cv::Mat flipped_map;
  cv::flip(rgb_map_, flipped_map, 0);

  cv::Mat translation_mat;
  Eigen::Matrix2f eigen_rot = Eigen::Rotation2Df(-M_PI_2) *
                              delta_trans.rotation().matrix().inverse() *
                              Eigen::Rotation2Df(M_PI_2);
  Eigen::Vector2f eigen_trans = -eigen_rot * Eigen::Rotation2Df(-M_PI_2) *
                                delta_trans.translation().matrix();
  eigen_trans = eigen_trans.cwiseProduct(
      Eigen::Vector2f{pixels_per_meter_, pixels_per_meter_});
  cv::eigen2cv(eigen_trans, translation_mat);

  Eigen::Rotation2Df rot;
  rot.fromRotationMatrix(eigen_rot);

  // TODO: make point a parameter
  auto transform_mat = cv::getRotationMatrix2D(
      cv::Point2f{640, 780}, -rot.angle() * (180 / M_PI), 1.0);

  transform_mat(cv::Rect(2, 0, 1, 2)) -= translation_mat;

  auto prev_map = flipped_map.clone();
  flipped_map.setTo(0);
  cv::warpAffine(prev_map, flipped_map, transform_mat, flipped_map.size(),
                 cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  rgb_map_.setTo(0);
  cv::flip(flipped_map, rgb_map_, 0);
}

std::shared_ptr<PathRolloutBase> RGBCostMapEvaluator::FindBest(
    const std::vector<std::shared_ptr<PathRolloutBase>> &paths) {
  // mostly lifted from DeepCostMapEvaluator except not using torch
  if (paths.size() == 0) return nullptr;
  std::shared_ptr<PathRolloutBase> best = nullptr;

  // copy here for future threaded implementation
  cv::Mat cost_map = cost_map_.clone();
  // auto cm_loc = map_loc_;
  // auto cm_ang = map_angle_;

  std::vector<double> path_costs(paths.size(), 0);
  for (size_t i = 0; i < paths.size(); i++) {
    for (size_t j = 0; j <= ImageBasedEvaluator::ROLLOUT_DENSITY; j++) {
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * j;
      auto state = paths[i]->GetIntermediateState(f);
      auto discount = (1 - state.translation.norm() * DISCOUNT_FACTOR);
      // TODO: make sure this gets the right location
      auto loc = GetImageLocation(state.translation);
      auto cost = cost_map.at<float>(cv::Point((int)loc.x(), (int)loc.y()));
      path_costs[i] += cost * discount / ImageBasedEvaluator::ROLLOUT_DENSITY;
    }
  }

  std::vector<double> blurred_path_costs;
  if (BLUR_FACTOR > 0) {
    blurred_path_costs = std::vector<double>(paths.size(), 0);
    for (size_t i = 0; i < paths.size(); i++) {
      float remaining = 1.0;
      if (i > 0) {
        blurred_path_costs[i] += BLUR_FACTOR * path_costs[i - 1];
        remaining -= BLUR_FACTOR;
      }
      if (i < paths.size() - 1) {
        blurred_path_costs[i] += BLUR_FACTOR * path_costs[i + 1];
        remaining -= BLUR_FACTOR;
      }

      blurred_path_costs[i] += remaining * path_costs[i];
    }
  } else {
    blurred_path_costs = path_costs;
  }

  // Lifted from linear evaluator
  // Check if there is any path with an obstacle-free path from the end to the
  // local target.
  float curr_goal_dist = local_target.norm();

  std::vector<float> clearance_to_goal(paths.size(), 0.0);
  std::vector<float> dist_to_goal(paths.size(), FLT_MAX);
  // bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] = StraightLineClearance(
        geometry::Line2f(endpoint, local_target), point_cloud);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - local_target).norm();
      // path_to_goal_exists = true;
    }
  }

  // First find the shortest path.
  int best_idx = -1;
  float best_path_progress = -FLT_MAX;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_progress = curr_goal_dist - dist_to_goal[i];
    if (path_progress > best_path_progress) {
      best_path_progress = path_progress;
      best_idx = i;
      best = paths[i];
    }
  }

  if (best_idx == -1) {
    std::cout << "No Valid Paths" << std::endl;
    // No valid paths!
    return nullptr;
  }

  float best_cost = DISTANCE_WEIGHT * best_path_progress +
                    FPL_WEIGHT * paths[best_idx]->FPL() +
                    CLEARANCE_WEIGHT * paths[best_idx]->Clearance() +
                    COST_WEIGHT * path_costs[best_idx];

  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_progress = curr_goal_dist - dist_to_goal[i];
    const float cost = DISTANCE_WEIGHT * path_progress +
                       FPL_WEIGHT * paths[i]->FPL() +
                       CLEARANCE_WEIGHT * paths[i]->Clearance() +
                       COST_WEIGHT * path_costs[best_idx];

    // latest_cost_components_.push_back(dynamic_cast<ConstantCurvatureArc*>(paths[i].get())->curvature);
    // latest_cost_components_.push_back(dynamic_cast<ConstantCurvatureArc*>(paths[i].get())->length);
    // latest_cost_components_.push_back(DISTANCE_WEIGHT * path_progress);
    // latest_cost_components_.push_back(FPL_WEIGHT * paths[i]->FPL());
    // latest_cost_components_.push_back(CLEARANCE_WEIGHT *
    // paths[i]->Clearance()); latest_cost_components_.push_back(COST_WEIGHT *
    // normalized_path_costs.at<float>(i, 0));
    // latest_cost_components_.push_back(cost);
    // std::cout << "COST" << latest_cost_components_ << std::endl;

    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
      best_idx = i;
    }
  }

  // TODO: vis images

  return best;
}

}  // namespace motion_primitives