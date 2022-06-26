#include "rgb_cost_map_evaluator.h"

namespace motion_primitives {

void RGBCostMapEvaluator::Update(
    const Eigen::Vector2f &new_loc, const float new_ang,
    const Eigen::Vector2f &new_vel, const float new_ang_vel,
    const Eigen::Vector2f &new_local_target,
    const std::vector<Eigen::Vector2f> &new_point_cloud,
    const cv::Mat &new_image) {
  curr_loc = new_loc;
  curr_ang = new_ang;
  vel = new_vel;
  ang_vel = new_ang_vel;
  local_target = new_local_target;
  point_cloud = new_point_cloud;
  image = new_image;

  // TODO: move this to its own thread
  UpdateMapToLocalFrame();
  rgb_map_loc_ = new_loc;
  rgb_map_angle_ = new_ang;
  (*cost_function)(local_rgb_map_, local_cost_map_);
}

void RGBCostMapEvaluator::UpdateMapToLocalFrame() {
  auto curr_trans =
      Eigen::Translation2f(rgb_map_loc_) * Eigen::Rotation2Df(rgb_map_angle_);
  auto new_trans =
      Eigen::Translation2f(curr_loc) * Eigen::Rotation2Df(curr_ang);
  Eigen::Affine2f delta_trans = curr_trans.inverse() * new_trans;

  cv::Mat flipped_map;
  cv::flip(local_rgb_map_, flipped_map, 0);

  cv::Mat translation_mat;
  Eigen::Matrix2f eigen_rot = Eigen::Rotation2Df(-M_PI_2) *
                              delta_trans.rotation().matrix().inverse() *
                              Eigen::Rotation2Df(M_PI_2);
  Eigen::Vector2f eigen_trans = -eigen_rot * Eigen::Rotation2Df(-M_PI_2) *
                                delta_trans.translation().matrix();
  eigen_trans = eigen_trans.cwiseProduct(
      Eigen::Vector2f{pixels_per_meter_, pixels_per_meter_});

  Eigen::Rotation2Df rot;
  rot.fromRotationMatrix(eigen_rot);

  auto transform_mat = cv::getRotationMatrix2D(
      cv::Point2f{640, 780}, -rot.angle() * (180 / M_PI), 1.0);

  transform_mat(cv::Rect(2, 0, 1, 2)) -= translation_mat;

  auto prev_map = flipped_map.clone();
  flipped_map.setTo(0);
  cv::warpAffine(prev_map, flipped_map, transform_mat, flipped_map.size(),
                 cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  local_rgb_map_.setTo(0);
  cv::flip(flipped_map, local_rgb_map_, 0);
}

} // namespace motion_primitives