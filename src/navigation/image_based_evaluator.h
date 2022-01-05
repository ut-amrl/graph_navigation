//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    image_based_evaluator.h
\brief   Based class for warped image-based evaluation (2d plane projection of the image).
\author  Kavan Sikand, (C) 2021
*/
//========================================================================

#include <string>
#include <vector>

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "opencv2/opencv.hpp"

#ifndef IMAGE_BASED_EVALUATOR_H
#define IMAGE_BASED_EVALUATOR_H
#define SIMULATION_MODE 0

namespace motion_primitives {

struct ImageBasedEvaluator :  PathEvaluatorBase {
  ImageBasedEvaluator(const navigation::NavigationParameters& params) : params_(params) {
    cameraMatrix = params.K;
    distortionMatrix = params.D;

    // Homography computation
    SCALING = Eigen::Vector2f(100, 100);
    if (params.use_kinect) {
      CENTER = Eigen::Vector2f(640, 780);
    } else  {
      CENTER = Eigen::Vector2f(640, 1024);
    }

    // For sim
    #if SIMULATION_MODE
    CENTER = Eigen::Vector2f(400, 400);
    #endif

    std::vector<cv::Point2f> input_points;
    std::vector<Eigen::Vector2f> output_points_vec;

    if (params.H.rows == 0) {
      std::cerr << "Homography information is missing!" << std::endl;
      exit(1);
    }

    for(int i = 0; i < params.H.rows; i++) {
      auto input_point = cv::Point2f(params.H.at<double>(i, 2), params.H.at<double>(i, 3));
      auto output_point = Eigen::Vector2f(params.H.at<double>(i, 0), params.H.at<double>(i, 1));
      input_points.push_back(input_point);
      output_points_vec.push_back(output_point);
    }

    std::vector<cv::Point2f> output_points;
    for(size_t i = 0; i < output_points_vec.size(); i++) {
      Eigen::Vector2f transformed = output_points_vec[i].cwiseProduct(SCALING) + CENTER;
      output_points.emplace_back(transformed.x(), transformed.y());
    }
    

    homography = cv::findHomography(input_points, output_points);
  }

  cv::Mat GetPatchAtLocation(const cv::Mat& img, const Eigen::Vector2f& location, float* validity, bool filter_empty);
  std::vector<cv::Mat> GetPatchesAtPose(const cv::Mat& img, const pose_2d::Pose2Df& pose, std::vector<Eigen::Vector2f>* image_locs, std::vector<float>* validity, bool filter_empty, float robot_width, float robot_length);

  cv::Mat GetPatchAtImageLocation(const cv::Mat& img, const Eigen::Vector2f& location, float* validity, bool filter_empty);
  Eigen::Vector2f GetImageLocation(const Eigen::Vector2f& rel_loc);

  std::vector<Eigen::Vector2f> GetWheelLocations(const pose_2d::Pose2Df& pose, float robot_width, float robot_length);

  std::vector<Eigen::Vector2f> GetTilingLocations(const cv::Mat& img, const int tile_size);

  cv::Mat GetWarpedImage();

  cv::Mat cameraMatrix;
  cv::Mat distortionMatrix;
  cv::Mat homography;
  cv::Mat latest_vis_image_ = cv::Mat::zeros(10, 10, CV_32F);

  const navigation::NavigationParameters& params_;

  Eigen::Vector2f SCALING;
  Eigen::Vector2f CENTER;
  static const int PATCH_SIZE = 40;
  static const int HALF_PATCH_SIZE = PATCH_SIZE / 2;
  static const int PATCH_PIXEL_COUNT = PATCH_SIZE * PATCH_SIZE;
  static constexpr float PATCH_EMPTY_THRESHOLD = 0.15f;
  static constexpr float ROBOT_SIZE_SAMPLE_SCALING = 1.0f;
  static const size_t ROLLOUT_DENSITY = 80;


  #if SIMULATION_MODE
    static constexpr float TILING_START_PCT = 0.0f;
    static constexpr float TILING_END_PCT = 0.4f;
  #else
    static constexpr float TILING_START_PCT = 0.6f;
    static constexpr float TILING_END_PCT = 1.0f;
  #endif

};

}  // namespace motion_primitives


#endif  // IMAGE_BASED_EVALUATOR_H