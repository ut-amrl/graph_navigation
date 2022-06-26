#ifndef RGB_COST_MAP_EVALUATOR_H
#define RGB_COST_MAP_EVALUATOR_H

#include "image_based_evaluator.h"
#include "motion_primitives.h"

namespace motion_primitives {

struct CostFunction {
  virtual void operator()(cv::Mat &rgb_map, cv::Mat &cost_map) = 0;
};

struct IdentityCostFunction : CostFunction {
  void operator()(cv::Mat &rgb_map, cv::Mat &cost_map) override {
    cost_map.setTo(1);
  }
};

struct RGBCostMapEvaluator : ImageBasedEvaluator {
  RGBCostMapEvaluator(const navigation::NavigationParameters &params,
                      std::shared_ptr<CostFunction> cost_function)
      : ImageBasedEvaluator(params), cost_function(cost_function) {
    // TODO: make these params
    map_width_ = 8;
    map_height = 8;
    pixels_per_meter_ = 100;

    rgb_map_ = cv::Mat::zeros(map_width_ * pixels_per_meter_,
                              map_height * pixels_per_meter_, CV_8UC4);
    cost_map_ = cv::Mat::zeros(map_width_ * pixels_per_meter_,
                               map_height * pixels_per_meter_, CV_8UC1);
  }

  void Update(const Eigen::Vector2f &new_loc, const float new_ang,
              const Eigen::Vector2f &new_vel, const float new_ang_vel,
              const Eigen::Vector2f &new_local_target,
              const std::vector<Eigen::Vector2f> &new_point_cloud,
              const cv::Mat &new_image) override;

  void UpdateMapToLocalFrame();

  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>> &paths) override;

  // size in meters of the maps in the y axis of the robot frame
  size_t map_width_;
  // size in meters of the maps in the x axis of the robot frame
  size_t map_height;
  // number of pixels per meter in the maps
  size_t pixels_per_meter_;

  cv::Mat rgb_map_;
  cv::Mat cost_map_;
  Eigen::Vector2f map_loc_;
  float map_angle_;

  std::shared_ptr<CostFunction> cost_function;

  double DISCOUNT_FACTOR = 0.25;
  double BLUR_FACTOR = 0.05;
  double DISTANCE_WEIGHT = -3.5;
  double FPL_WEIGHT = -0.75;
  double CLEARANCE_WEIGHT = -0.25;
  double COST_WEIGHT = 4.0;
};

}  // namespace motion_primitives

#endif