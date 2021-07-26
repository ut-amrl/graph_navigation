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
#include "opencv2/opencv.hpp"

#ifndef IMAGE_BASED_EVALUATOR_H
#define IMAGE_BASED_EVALUATOR_H

namespace motion_primitives {

struct ImageBasedEvaluator :  PathEvaluatorBase {
  ImageBasedEvaluator(const std::vector<double>& K, const std::vector<double>& D, const std::vector<std::vector<float>>& H, bool kinect) {
    cameraMatrix = cv::Mat(3, 3, CV_64F);
    memcpy(cameraMatrix.data, K.data(), K.size()*sizeof(double));

    distortionMatrix = cv::Mat(5, 1, CV_64F);
    memcpy(distortionMatrix.data, D.data(), D.size()*sizeof(double));

    // Homography computation
    SCALING = Eigen::Vector2f(100, 100);
    if (kinect) {
      CENTER = Eigen::Vector2f(640, 812);
    } else{
      CENTER = Eigen::Vector2f(640, 1024);
    }
    std::vector<cv::Point2f> input_points;
    std::vector<Eigen::Vector2f> output_points_vec;

    for(auto point_pair : H) {
      input_points.emplace_back(point_pair[2], point_pair[3]);
      output_points_vec.emplace_back(point_pair[0], point_pair[1]);
    }

    std::vector<cv::Point2f> output_points;
    for(size_t i = 0; i < output_points_vec.size(); i++) {
      Eigen::Vector2f transformed = output_points_vec[i].cwiseProduct(SCALING) + CENTER;
      output_points.emplace_back(transformed.x(), transformed.y());
    }
    

    homography = cv::findHomography(input_points, output_points);
  }

  cv::Mat GetPatchAtLocation(const cv::Mat& img, const cv::Point& coord, float* validity, bool filter_empty);
  std::vector<cv::Mat> GetPatchesAtLocation(const cv::Mat& img, const Eigen::Vector2f& location, std::vector<float>* validities, bool blur, bool filter_empty);

  Eigen::Vector2f GetImageLocation(const Eigen::Vector2f& rel_loc);

  cv::Mat GetWarpedImage();

  cv::Mat cameraMatrix;
  cv::Mat distortionMatrix;
  cv::Mat homography;

  Eigen::Vector2f SCALING;
  Eigen::Vector2f CENTER;
  static const int PATCH_SIZE = 40;
  static const int HALF_PATCH_SIZE = PATCH_SIZE / 2;
  static const int PATCH_PIXEL_COUNT = PATCH_SIZE * PATCH_SIZE;
  static constexpr float PATCH_EMPTY_THRESHOLD = 0.35f;
  static constexpr float MIN_IMAGE_Y_PCT = 0.55f;
  static const size_t ROLLOUT_DENSITY = 20;
};

}  // namespace motion_primitives


#endif  // IMAGE_BASED_EVALUATOR_H