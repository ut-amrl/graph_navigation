#include "terrain_evaluator.h"

#include <config_reader/config_reader.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <stdexcept>
#include <vector>

#include "constant_curvature_arcs.h"

namespace motion_primitives {

// A config reader must be initialized either in each program's main function or
// elsewhere by the caller.
CONFIG_INT(patch_size_pixels, "TerrainEvaluator.patch_size_pixels");
CONFIG_INT(bev_pixels_per_meter, "TerrainEvaluator.bev_pixels_per_meter");
CONFIG_FLOAT(min_cost, "TerrainEvaluator.min_cost");
CONFIG_FLOAT(max_cost, "TerrainEvaluator.max_cost");
CONFIG_FLOAT(discount_factor, "TerrainEvaluator.discount_factor");
CONFIG_INT(rollout_density, "TerrainEvaluator.rollout_density");
CONFIG_STRING(model_path, "TerrainEvaluator.model_path");

CONFIG_FLOAT(dist_to_goal_weight, "TerrainEvaluator.dist_to_goal_weight");
CONFIG_FLOAT(clearance_weight, "TerrainEvaluator.clearance_weight");
CONFIG_FLOAT(fpl_weight, "TerrainEvaluator.fpl_weight");
CONFIG_FLOAT(terrain_weight, "TerrainEvaluator.terrain_weight");

TerrainEvaluator::TerrainEvaluator()
    : cost_model_path_(CONFIG_model_path),
      torch_device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU) {}

bool TerrainEvaluator::LoadModel() {
  // Following the pytorch tutorial:
  // https://pytorch.org/tutorials/advanced/cpp_export.html#step-3-loading-your-script-module-in-c
  printf("Loading TerrainEvaluator model from %s\n", cost_model_path_.c_str());
  if (cost_model_path_.length() == 0) {
    return true;
  } else if (!boost::filesystem::exists(cost_model_path_)) {
    LOG(WARNING) << "Model file '" << cost_model_path_ << "' does not exist.";
    return false;
  }

  try {
    std::cout << "Loading model" << std::endl;
    cost_model_ = torch::jit::load(cost_model_path_, torch_device_);
    cost_model_.eval();

    // print done loading 
    std::cout << "Done loading model" << std::endl;


    return true;
  } catch (const c10::Error& e) {
    LOG(ERROR) << "Unable to load model: \n" << e.msg();
    return false;
  }
}

std::shared_ptr<PathRolloutBase> TerrainEvaluator::FindBest(
    const std::vector<std::shared_ptr<PathRolloutBase>>& paths) {
  // This reassignment is simply to reduce ambiguity.
  const cv::Mat3b& latest_bev_image = image;
  // print the shape
  std::cout << "latest bev image " << latest_bev_image.rows << " " << latest_bev_image.cols << std::endl;

  if (latest_bev_image.rows == 0) {
    // cannot plan if the image is not initialized
    return paths.front();
  }

  cv::Mat1f cost_image;
  if (cost_model_path_.length() != 0) {
    cost_image = GetScalarCostImage(latest_bev_image);
  } else {
    std::vector<cv::Mat> channels;
    cv::split(latest_bev_image, channels);
    channels[0].convertTo(cost_image, CV_32F);
  }

  // TODO(eyang): toggle computation with a flag?
  latest_cost_image_ = GetRGBCostImage(cost_image);
  latest_cost_image_.copyTo(latest_vis_image_);

  // TODO(eyang): skipped a bunch of code dealing with other factors: distance to goal,
  // clearance, progress, etc.

  // Don't consider paths with endpoints that are blocked from the goal.
  std::vector<float> endpoint_clearance_to_goal(paths.size(), 0.0f);
  std::vector<float> endpoint_dist_to_goal(paths.size(), local_target.norm());
  for (size_t i = 0; i < paths.size(); ++i) {
    const Eigen::Vector2f path_endpoint = paths[i]->EndPoint().translation;
    endpoint_clearance_to_goal[i] =
        StraightLineClearance(geometry::Line2f(path_endpoint, local_target), point_cloud);
    // TODO(eyang): should this be a hyperparameter?
    if (endpoint_clearance_to_goal[i] > 0.05) {
      endpoint_dist_to_goal[i] = (path_endpoint - local_target).norm();
    }
  }

  // mostly adapted from DeepCostMapEvaluator
  std::vector<float> terrain_costs(paths.size(), 0.0f);
  for (size_t i = 0; i < paths.size(); i++) {
    float weight_sum = 0.0f;

    for (int j = 0; j <= CONFIG_rollout_density; j++) {
      const pose_2d::Pose2Df state =
          paths[i]->GetIntermediateState(static_cast<float>(j) / CONFIG_rollout_density);
      const Eigen::Vector2i P_image_state =
          GetImageLocation(latest_bev_image, state.translation).cast<int>();

      // TODO: eventually in this case, ascribe the "out-of-view point" cost
      if (!ImageBoundCheck(cost_image, P_image_state)) {
        LOG(ERROR) << "Cost image query point is outside the bounds of the image.";
      }

      const float center_cost = cost_image.at<float>(P_image_state.y(), P_image_state.x());
      if (center_cost > CONFIG_max_cost) {
        // indicates this point is not visible
        continue;
      }

      // Calculate the average terrain cost of the wheels/legs.
      float cost = 0;
      int num_valid_wheels = 0;

      // TODO(eyang): use the robot width and length from config
      const float robot_length = 0.6;
      const float robot_width = 0.6;
      const Eigen::Rotation2Df state_rotation(state.angle);

      for (int i = 0; i < 4; i++) {
        // Generate corners. These "side" values can either be 1 or -1.
        const int x_side = 1 - (i & 0b10);
        const int y_side = 1 - 2 * (i & 0b1);

        const Eigen::Vector2f P_robot_corner(x_side * robot_length / 2, y_side * robot_width / 2);
        const Eigen::Vector2i P_image_corner =
            GetImageLocation(latest_bev_image, state.translation + state_rotation * P_robot_corner)
                .cast<int>();

        if (ImageBoundCheck(cost_image, P_image_corner)) {
          cost += cost_image.at<float>(P_image_corner.y(), P_image_corner.x());
          ++num_valid_wheels;
        }
      }

      if (num_valid_wheels != 0) {
        cost /= num_valid_wheels;
      } else {
        cost = CONFIG_max_cost;
      }

      const float weight = std::pow(CONFIG_discount_factor, state.translation.norm());
      terrain_costs[i] += weight * cost;
      weight_sum += weight;
    }

    if (weight_sum != 0) {
      terrain_costs[i] /= weight_sum;
    } else {
      // indicates none of the points were visible
      terrain_costs[i] = CONFIG_max_cost;
    }
  }

  std::shared_ptr<PathRolloutBase> best_path = nullptr;
  float best_path_cost = std::numeric_limits<float>::infinity();
  path_costs_ = std::vector<float>(paths.size(), best_path_cost);
  for (size_t i = 0; i < paths.size(); ++i) {
    const float path_progress = local_target.norm() - endpoint_dist_to_goal[i];
    path_costs_[i] =
        CONFIG_dist_to_goal_weight * path_progress + CONFIG_clearance_weight * paths[i]->FPL() +
        CONFIG_clearance_weight * paths[i]->Clearance() + CONFIG_terrain_weight * terrain_costs[i];

    if (path_costs_[i] < best_path_cost) {
      best_path_cost = path_costs_[i];
      best_path = paths[i];
    }
  }

  DrawPathCosts(paths, best_path);
  // save the image to a file
  // latest vis from rgb to bgr
  // cv::cvtColor(latest_vis_image_, latest_vis_image_, cv::COLOR_RGB2BGR);
  // cv::imwrite("latest_vis.png", latest_vis_image_);
  // cv::cvtColor(latest_vis_image_, latest_vis_image_, cv::COLOR_BGR2RGB);
  // cv::imwrite("latest_cost.png", latest_cost_image_);
  // static int functionCallCount = 0;
  // functionCallCount++;
  // if (functionCallCount > 1) {
  //   std::cout << "Exiting here" << std::endl;
  //   exit(0);
  // }


  return best_path;
}

cv::Mat1f TerrainEvaluator::GetScalarCostImage(const cv::Mat3b& bev_image) {
  // print old get scalar cost image
  std::cout << "Old GetScalarCostImage" << std::endl;

  std::vector<torch::Tensor> bev_patch_tensors;
  std::vector<cv::Rect> bev_patch_rects;

  // Iterate through the patches in the birds-eye-view image in row major order.
  for (int row = 0; row + CONFIG_patch_size_pixels <= bev_image.rows;
       row += CONFIG_patch_size_pixels) {
    for (int col = 0; col + CONFIG_patch_size_pixels <= bev_image.cols;
         col += CONFIG_patch_size_pixels) {
      // x, y, width, height
      const cv::Rect bev_patch_roi(col, row, CONFIG_patch_size_pixels, CONFIG_patch_size_pixels);

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
      if (static_cast<float>(nonzero_count) /
              (CONFIG_patch_size_pixels * CONFIG_patch_size_pixels) <
          minimum_nonzero_ratio) {
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

      bev_patch_tensor = bev_patch_tensor.flip(2);  // BGR -> RGB

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
  cv::Mat1f cost_image(bev_image.rows, bev_image.cols, CONFIG_max_cost + 1);
  for (size_t i = 0; i < bev_patch_tensors.size(); ++i) {
    float patch_cost = model_output_tensor[i].item<float>();

    cv::Mat1f cost_image_patch = cost_image(bev_patch_rects[i]);
    cost_image_patch = patch_cost;
  }

  return cost_image;
}

cv::Mat3b TerrainEvaluator::GetRGBCostImage(const cv::Mat1f& scalar_cost_image) {
  cv::Mat3b rgb_cost_image = cv::Mat3b(scalar_cost_image.rows, scalar_cost_image.cols);

  cv::Mat1f intensity = (scalar_cost_image - CONFIG_min_cost) / (CONFIG_max_cost - CONFIG_min_cost);
  // Clamp the intensity values
  cv::min(intensity, 1.f, intensity);
  cv::max(intensity, 0.f, intensity);
  // Convert to byte range
  intensity *= 255;

  // TODO(eyang): there might be a builtin function that can do this copy more effiently
  for (int row = 0; row < rgb_cost_image.rows; ++row) {
    for (int col = 0; col < rgb_cost_image.cols; ++col) {
      if (scalar_cost_image(row, col) <= CONFIG_max_cost) {
        rgb_cost_image.at<cv::Vec3b>(row, col) = cv::Vec3b::all(intensity(row, col));
      } else {
        rgb_cost_image.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
      }
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

  // Relative image coordinates of the query point.
  const Eigen::Vector2f P_image_rel =
      Eigen::Vector2f(-P_robot.y(), -P_robot.x()) * CONFIG_bev_pixels_per_meter;

  const Eigen::Vector2f P_image = P_image_robot + P_image_rel;

  return P_image;
}

cv::Rect TerrainEvaluator::GetPatchRectAtLocation(const cv::Mat3b& img,
                                                  const Eigen::Vector2f& P_robot) {
  const Eigen::Vector2f P_image = GetImageLocation(img, P_robot);

  // Top-left coordinates of patch
  int patch_tl_x =
      static_cast<int>(P_image.x() / CONFIG_patch_size_pixels) * CONFIG_patch_size_pixels;
  int patch_tl_y =
      static_cast<int>(P_image.y() / CONFIG_patch_size_pixels) * CONFIG_patch_size_pixels;

  return {patch_tl_x, patch_tl_y, CONFIG_patch_size_pixels, CONFIG_patch_size_pixels};
}

// Simple alpha-blending helper; we can define it as a member or free function
void TerrainEvaluator::AlphaBlend(const cv::Mat& overlay, cv::Mat& base) {
  // overlay and base must both be BGRA
  for (int y = 0; y < base.rows; ++y) {
    const cv::Vec4b* oRow = overlay.ptr<cv::Vec4b>(y);
    cv::Vec4b* bRow       = base.ptr<cv::Vec4b>(y);
    for (int x = 0; x < base.cols; ++x) {
      const cv::Vec4b& oPix = oRow[x];
      if (oPix[3] > 0) {
        float alpha = oPix[3] / 255.0f;
        cv::Vec4b& bPix = bRow[x];
        for (int c = 0; c < 3; ++c) {
          bPix[c] = cv::saturate_cast<uchar>(
              oPix[c] * alpha + bPix[c] * (1.0f - alpha));
        }
        bPix[3] = 255;
      }
    }
  }
}

void TerrainEvaluator::DrawPathCosts(const std::vector<std::shared_ptr<PathRolloutBase>>& paths,
                                     std::shared_ptr<PathRolloutBase> best_path) {
  if (paths.empty() || latest_vis_image_.empty()) {
    return;
  }

  // 1) Prepare the "cost-based" image (latest_vis_image_) for annotation
  cv::Mat bev_base;
  cv::cvtColor(latest_vis_image_, bev_base, cv::COLOR_BGR2BGRA);
  cv::Mat line_overlay = cv::Mat::zeros(bev_base.size(), bev_base.type());

  cv::Mat rgb = image.clone();

  // 2) Prepare the "raw RGB" image (rgb_) for annotation
  cv::Mat rgb_base;
  if (!rgb.empty()) {
    cv::cvtColor(rgb, rgb_base, cv::COLOR_BGR2BGRA);
  } else {
    LOG(WARNING) << "rgb_ image is empty, skipping separate RGB annotation.";
  }
  // Create an overlay for the RGB image if valid
  cv::Mat line_overlay_rgb;
  if (!rgb_base.empty()) {
    line_overlay_rgb = cv::Mat::zeros(rgb_base.size(), rgb_base.type());
  }

  // 3) Identify best path index
  int best_index = -1;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i] == best_path) {
      best_index = static_cast<int>(i);
      break;
    }
  }

  // 4) Normalize path costs to [0,1] for coloring
  if (path_costs_.size() != paths.size()) {
    LOG(WARNING) << "path_costs_ size != paths.size(), skipping DrawPathCosts.";
    return;
  }
  float min_val = std::numeric_limits<float>::infinity();
  float max_val = -std::numeric_limits<float>::infinity();
  for (float c : path_costs_) {
    min_val = std::min(min_val, c);
    max_val = std::max(max_val, c);
  }
  float denom = (max_val - min_val) + 1e-6f;
  auto normalize_cost = [&](float c) {
    return (c - min_val) / denom; // [0..1]
  };

  // A small helper for mapping normalized cost to a BGR color (green→yellow→red)
  auto costToBGR = [&](float norm_cost) -> cv::Vec3b {
    if (norm_cost <= 0.5f) {
      float t = norm_cost / 0.5f;
      // B=0, G=255, R ~ [0..255]
      uchar red = static_cast<uchar>(255.0f * t);
      return cv::Vec3b(0, 255, red);
    } else {
      float t = (norm_cost - 0.5f) / 0.5f;
      uchar green = static_cast<uchar>(255.0f * (1.0f - t));
      return cv::Vec3b(0, green, 255);
    }
  };

  // 5) Draw polylines for each path on both images
  int thickness_best = 4;
  int thickness_regular = 2;
  int circle_radius = 6;

  for (size_t i = 0; i < paths.size(); ++i) {
    bool is_best = (static_cast<int>(i) == best_index);
    // Best path = Aqua BGRA(255,255,0,255) fully opaque
    // Other path = Yellow BGRA(0,255,255,128) half alpha
    cv::Scalar line_color_bgra = is_best
        ? cv::Scalar(255,255,0,255)
        : cv::Scalar(0,255,255,128);
    int thickness = is_best ? thickness_best : thickness_regular;

    // Compute normalized cost → circle color
    float norm_c = normalize_cost(path_costs_[i]);
    cv::Vec3b bgr_col = costToBGR(norm_c);
    cv::Vec4b end_circle_color(bgr_col[0], bgr_col[1], bgr_col[2], 255);

    // Gather discrete points along the path
    std::vector<cv::Point> poly_points;
    int num_samples = std::max(1, CONFIG_rollout_density);
    for (int j = 0; j <= num_samples; ++j) {
      float alpha = static_cast<float>(j) / num_samples;
      pose_2d::Pose2Df st = paths[i]->GetIntermediateState(alpha);
      // (A) For the cost-based image
      Eigen::Vector2f P_img = GetImageLocation(latest_vis_image_, st.translation);
      poly_points.emplace_back((int)P_img.x(), (int)P_img.y());
    }

    // Draw polylines on the cost-based overlay
    if (poly_points.size() >= 2) {
      std::vector<std::vector<cv::Point>> contour{ poly_points };
      cv::polylines(line_overlay, contour, false, line_color_bgra, thickness, cv::LINE_AA);
    }

    // Draw the end circle on the cost-based image
    if (!poly_points.empty()) {
      cv::circle(bev_base, poly_points.back(), circle_radius, end_circle_color, -1, cv::LINE_AA);
    }

    // --- (B) If we have a valid rgb_ image, do the same logic for that image ---
    if (!rgb_base.empty()) {
      // Gather points for the RGB image
      std::vector<cv::Point> rgb_points;
      for (int j = 0; j <= num_samples; ++j) {
        float alpha = static_cast<float>(j) / num_samples;
        pose_2d::Pose2Df st = paths[i]->GetIntermediateState(alpha);
        // Possibly you need a different transform for the RGB image?
        // For now, reuse the same function:
        Eigen::Vector2f P_img_rgb = GetImageLocation(rgb, st.translation);
        rgb_points.emplace_back((int)P_img_rgb.x(), (int)P_img_rgb.y());
      }

      // Draw polylines on the rgb overlay
      if (rgb_points.size() >= 2) {
        std::vector<std::vector<cv::Point>> contour{ rgb_points };
        cv::polylines(line_overlay_rgb, contour, false, line_color_bgra, thickness, cv::LINE_AA);
      }

      // Draw end circle fully opaque
      if (!rgb_points.empty()) {
        cv::circle(rgb_base, rgb_points.back(), circle_radius, end_circle_color, -1, cv::LINE_AA);
      }
    }
  }

  // 6) Alpha blend the polylines onto each base
  AlphaBlend(line_overlay, bev_base);
  if (!rgb_base.empty()) {
    AlphaBlend(line_overlay_rgb, rgb_base);
  }

  // 7) Optionally draw the local_target marker on each image
  {
    // Cost-based image target
    Eigen::Vector2f tgt_img = GetImageLocation(latest_vis_image_, local_target);
    cv::Point tgt_pt((int)tgt_img.x(), (int)tgt_img.y());
    cv::drawMarker(bev_base, tgt_pt, cv::Scalar(255,255,255,255),
                   cv::MARKER_TILTED_CROSS, 20, 2, cv::LINE_AA);

    // RGB-based image target
    if (!rgb_base.empty()) {
      Eigen::Vector2f tgt_img_rgb = GetImageLocation(rgb, local_target);
      cv::Point tgt_pt_rgb((int)tgt_img_rgb.x(), (int)tgt_img_rgb.y());
      cv::drawMarker(rgb_base, tgt_pt_rgb, cv::Scalar(255,255,255,255),
                     cv::MARKER_TILTED_CROSS, 20, 2, cv::LINE_AA);
    }
  }

  // 8) Convert both BGRA images back to BGR
  cv::cvtColor(bev_base, latest_vis_image_, cv::COLOR_BGRA2BGR);
  if (!rgb_base.empty()) {
    cv::Mat annotated_bgr;
    cv::cvtColor(rgb_base, annotated_bgr, cv::COLOR_BGRA2BGR);
    annotated_rgb_image_ = annotated_bgr.clone();
  }
}

}  // namespace motion_primitives
