#include "terrain_evaluator.h"

#include <config_reader/config_reader.h>
#include <glog/logging.h>
#include <torch/script.h>

#include <boost/filesystem.hpp>
#include <stdexcept>
#include <vector>

namespace motion_primitives {

CONFIG_INT(patch_size_pixels, "TerrainEvaluator.patch_size_pixels");
CONFIG_INT(bev_pixels_per_meter, "TerrainEvaluator.bev_pixels_per_meter");
CONFIG_FLOAT(min_cost, "TerrainEvaluator.min_cost");
CONFIG_FLOAT(max_cost, "TerrainEvaluator.max_cost");
CONFIG_FLOAT(discount_factor, "TerrainEvaluator.discount_factor");
CONFIG_UINT(rollout_density, "TerrainEvaluator.rollout_density");
CONFIG_STRING(model_path, "TerrainEvaluator.model_path");

TerrainEvaluator::TerrainEvaluator(const navigation::NavigationParameters& navigation_parameters)
    : cost_model_path_(CONFIG_model_path),
      torch_device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU),
      patch_size_(CONFIG_patch_size_pixels),
      min_cost_(CONFIG_min_cost),
      max_cost_(CONFIG_max_cost),
      discount_factor_(CONFIG_discount_factor),
      rollout_density_(CONFIG_rollout_density) {}

bool TerrainEvaluator::LoadModel() {
  // Following the pytorch tutorial:
  // https://pytorch.org/tutorials/advanced/cpp_export.html#step-3-loading-your-script-module-in-c

  if (!boost::filesystem::exists(cost_model_path_)) {
    throw std::system_error(std::make_error_code(std::errc::no_such_file_or_directory),
                            "Model file '" + cost_model_path_ + "'");
  }

  try {
    cost_model_ = torch::jit::load(cost_model_path_, torch_device_);
    cost_model_.eval();

    return true;
  } catch (const c10::Error& e) {
    std::cerr << "Error loading the model:\n" << e.msg();
    return false;
  }
}

std::shared_ptr<PathRolloutBase> TerrainEvaluator::FindBest(
    const std::vector<std::shared_ptr<PathRolloutBase>>& paths) {
  // This reassignment is simply to reduce ambiguity.
  const cv::Mat3b& latest_bev_image = image;

  if (latest_bev_image.rows == 0) {
    // cannot plan if the image is not initialized
    return paths.front();
  }

  const cv::Mat1f cost_image = GetScalarCostImage(latest_bev_image);

  // TODO(eyang): toggle computation with a flag?
  latest_vis_image_ = GetRGBCostImage(cost_image);

  // TODO(eyang): skipped a bunch of code dealing with other factors: distance to goal,
  // clearance, progress, etc.

  // mostly adapted from DeepCostMapEvaluator
  path_costs_ = std::vector<float>(paths.size(), 0.0f);
  for (size_t i = 0; i < paths.size(); i++) {
    // Simple averaging scheme: average the costs of the visible points, discard
    // any out-of-view points.
    int num_visible_intermediate_states = 0;

    for (int j = 0; j <= rollout_density_; j++) {
      const pose_2d::Pose2Df state =
          paths[i]->GetIntermediateState(static_cast<float>(j) / rollout_density_);
      // TODO(eyang): does this computation make sense?
      // const float discount = 1.0f - state.translation.norm() * discount_factor_;

      // This discount assumes we have perfect knowledge of the terrain
      // const float discount = 1.0f;

      const Eigen::Vector2i P_image_state =
          GetImageLocation(latest_bev_image, state.translation).cast<int>();

      const float cost = cost_image.at<float>(P_image_state.y(), P_image_state.x());
      if (cost <= max_cost_) {
        path_costs_[i] += cost;
        num_visible_intermediate_states++;
      }

      // path_costs_[i] += cost * discount / rollout_density_;
    }

    if (num_visible_intermediate_states != 0) {
      path_costs_[i] /= num_visible_intermediate_states;
    } else {
      path_costs_[i] = max_cost_;
    }
  }

  std::shared_ptr<PathRolloutBase> best_path = paths[0];
  float best_path_cost = path_costs_[0];
  for (size_t i = 1; i < paths.size(); i++) {
    if (path_costs_[i] < best_path_cost) {
      best_path = paths[i];
      best_path_cost = path_costs_[i];
    }
  }

  DrawPathCosts(paths, best_path);

  return best_path;
}

cv::Mat1f TerrainEvaluator::GetScalarCostImage(const cv::Mat3b& bev_image) {
  std::vector<torch::Tensor> bev_patch_tensors;
  std::vector<cv::Rect> bev_patch_rects;

  // Iterate through the patches in the birds-eye-view image in row major order.
  for (int row = 0; row + patch_size_ <= bev_image.rows; row += patch_size_) {
    for (int col = 0; col + patch_size_ <= bev_image.cols; col += patch_size_) {
      // x, y, width, height
      const cv::Rect bev_patch_roi(col, row, patch_size_, patch_size_);

      // Need to clone the patch into its own Mat because the underlying data
      // pointer of the submat is of the original image. The old code didn't
      // need to do this though, why?
      cv::Mat3b bev_patch = bev_image(bev_patch_roi).clone();

      // Skip areas that are not visible. These areas are filled with black
      // pixels.
      const float minimum_nonzero_ratio = 0.5;
      int nonzero_count = 0;
      for (const cv::Vec3b& pixel : bev_patch) {
        if (pixel != cv::Vec3b::zeros()) {
          ++nonzero_count;
        }
      }
      if (static_cast<float>(nonzero_count) / (patch_size_ * patch_size_) < minimum_nonzero_ratio) {
        continue;
      }

      torch::Tensor bev_patch_tensor = torch::from_blob(
          bev_patch.data, {bev_patch.rows, bev_patch.cols, bev_patch.channels()}, torch::kByte);

      // Clone the tensor to take ownership of the underlying cv::Mat3b patch
      // data that will go out of scope.
      bev_patch_tensor = bev_patch_tensor.detach().clone();

      // This conversion might be necessary if the model's forward does not
      // convert the tensor to float. The model's expected behavior should be
      // standardized.
      // bev_patch_tensor = bev_patch_tensor.to(torch::kFloat);

      // TODO(eyang): The sample model was probably trained on BGR. We should
      // standardize the expected channel order.
      // bev_patch_tensor = bev_patch_tensor.flip(2);  // BGR -> RGB

      bev_patch_tensor = bev_patch_tensor.permute({2, 0, 1});

      bev_patch_tensors.push_back(bev_patch_tensor);
      bev_patch_rects.push_back(bev_patch_roi);
    }
  }

  torch::Tensor all_input_tensors = torch::stack(bev_patch_tensors).to(torch_device_);

  torch::NoGradGuard no_grad;
  const size_t BATCH_SIZE = 32;
  const int N_BATCHES = (bev_patch_tensors.size() + BATCH_SIZE - 1) / BATCH_SIZE;
  std::vector<torch::Tensor> batch_outputs;

  for (int batch = 0; batch < N_BATCHES; ++batch) {
    int batch_start_idx = batch * BATCH_SIZE;
    int batch_end_idx = std::min(batch_start_idx + BATCH_SIZE, bev_patch_tensors.size());

    torch::Tensor batch_tensor =
        all_input_tensors.index({torch::indexing::Slice(batch_start_idx, batch_end_idx)});

    // Type conversion to a vector of IValues is necessary for Module::forward
    std::vector<torch::jit::IValue> model_inputs;
    model_inputs.push_back(batch_tensor);

    batch_outputs.push_back(cost_model_.forward(model_inputs).toTensor().to(torch::kCPU));
  }

  torch::Tensor model_output_tensor = torch::cat(batch_outputs, 0).squeeze();

  // Regions that are out-of-view are indicated by a cost larger than the
  // model's max cost.
  cv::Mat1f cost_image(bev_image.rows, bev_image.cols, max_cost_ + 1);
  for (size_t i = 0; i < bev_patch_tensors.size(); ++i) {
    float patch_cost = model_output_tensor[i].item<float>();

    cv::Mat1f cost_image_patch = cost_image(bev_patch_rects[i]);
    cost_image_patch = patch_cost;
  }

  return cost_image;
}

cv::Mat3b TerrainEvaluator::GetRGBCostImage(const cv::Mat1f& scalar_cost_image) {
  cv::Mat3b rgb_cost_image = cv::Mat3b::zeros(scalar_cost_image.rows, scalar_cost_image.cols);

  cv::Mat1f intensity = (scalar_cost_image - min_cost_) / (max_cost_ - min_cost_);
  // Clamp the intensity values
  cv::min(intensity, 1.f, intensity);
  cv::max(intensity, 0.f, intensity);
  // Convert to byte range
  intensity *= 255;

  // TODO(eyang): there might be a builtin function that can do this copy more effiently
  for (int row = 0; row < rgb_cost_image.rows; ++row) {
    for (int col = 0; col < rgb_cost_image.cols; ++col) {
      // set the red channel (BGR)
      rgb_cost_image(row, col)[2] = static_cast<uchar>(intensity(row, col));
    }
  }

  return rgb_cost_image;
}

Eigen::Vector2f TerrainEvaluator::GetImageLocation(const cv::Mat3b& img,
                                                   const Eigen::Vector2f& P_robot) {
  // TODO(eyang): de-hardcode
  // TODO(eyang): For a single image, the robot's location is assumed to at the
  // center-bottom of the image. Eventually a fused BEV image may be used, where
  // the robot's location will be the center of the image.
  // TODO(eyang): maybe some latched msg or configuration for the center would be appropriate?
  // Location of the Robot's (0, 0) in the image
  const Eigen::Vector2f P_image_robot(img.cols / 2, img.rows - 1);

  // TODO(eyang): de-hardcode/configureify
  const int pixels_per_meter_ = 150;

  // Relative image coordinates of the query point.
  const Eigen::Vector2f P_image_rel =
      Eigen::Vector2f(-P_robot.y(), -P_robot.x()) * pixels_per_meter_;

  const Eigen::Vector2f P_image = P_image_robot + P_image_rel;

  if (P_image.x() < 0 || P_image.x() >= img.cols || P_image.y() < 0 || P_image.y() >= img.rows) {
    LOG(WARNING) << "Coordinates out of bounds (x: " << P_image.x() << ", y: " << P_image.y()
                 << ") for image with dimensions " << img.size();
  }

  return P_image;
}

cv::Rect TerrainEvaluator::GetPatchRectAtLocation(const cv::Mat3b& img,
                                                  const Eigen::Vector2f& P_robot) {
  const Eigen::Vector2f P_image = GetImageLocation(img, P_robot);

  // Top-left coordinates of patch
  int patch_tl_x = static_cast<int>(P_image.x() / patch_size_) * patch_size_;
  int patch_tl_y = static_cast<int>(P_image.y() / patch_size_) * patch_size_;

  return {patch_tl_x, patch_tl_y, patch_size_, patch_size_};
}

void TerrainEvaluator::DrawPathCosts(const std::vector<std::shared_ptr<PathRolloutBase>>& paths,
                                     std::shared_ptr<PathRolloutBase> best_path) {
  // TODO(eyang): perhaps have some toggle between the cost map image and the
  // BEV image? might be useful to have both options. Maybe this toggle should
  // be in FindBest, because that's where the latest_vis_image_ is set.

  std::vector<float> normalized_path_costs(path_costs_);
  const float max_cost =
      *std::max_element(normalized_path_costs.begin(), normalized_path_costs.end());
  for (float& cost : normalized_path_costs) {
    cost /= max_cost;
  }

  for (size_t i = 0; i < paths.size(); i++) {
    for (int j = 0; j < rollout_density_; j++) {
      const pose_2d::Pose2Df state =
          paths[i]->GetIntermediateState(static_cast<float>(j) / rollout_density_);
      const Eigen::Vector2f P_image_state = GetImageLocation(latest_vis_image_, state.translation);

      // Scale the color from green to yellow to red based on the normalized cost
      cv::Scalar color;  // RGB
      if (normalized_path_costs[i] < 0.5) {
        // green set to 255, increasing red changes color from green to yellow
        color[1] = 255.0;
        color[0] = normalized_path_costs[i] * 2 * 255.0;
      } else {
        // red set to 255, decreasing green changes color from yellow to red
        color[0] = 255.0;
        color[1] = 255.0 * (1 - 2 * normalized_path_costs[i]);
      }

      int thickness = 2;
      if (paths[i] == best_path) {
        // a negative thickness value fills in the drawn circle
        thickness = -thickness;
      }
      cv::circle(latest_vis_image_, cv::Point(P_image_state.x(), P_image_state.y()), 8, color,
                 thickness, cv::LineTypes::LINE_AA);
    }
  }

  const Eigen::Vector2f P_image_goal = GetImageLocation(latest_vis_image_, local_target);
  cv::drawMarker(latest_vis_image_, cv::Point(P_image_goal.x(), P_image_goal.y()), {255, 255, 255},
                 cv::MARKER_TILTED_CROSS, 32, 4, cv::LineTypes::LINE_AA);
}

}  // namespace motion_primitives
