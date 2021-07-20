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
\file    deep_irl_evaluator.cc
\brief   Path rollout evaluator using a NN learned via deep IRL.
\author  Joydeep Biswas, Kavan Sikand, (C) 2021
*/
//========================================================================

#include <math.h>
#include <float.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "image_based_evaluator.h"

using std::min;
using std::max;
using std::string;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;

namespace motion_primitives {

  cv::Mat ImageBasedEvaluator::GetWarpedImage() {
    cv::Mat image_undistorted = image.clone();
    cv::undistort(image.clone(), image_undistorted, cameraMatrix, distortionMatrix);
    cv::Mat warped;
    cv::warpPerspective(image_undistorted, warped, homography, image_undistorted.size());
    return warped;
  }

  Eigen::Vector2f ImageBasedEvaluator::GetImageLocation(const Eigen::Vector2f& rel_loc) {
    return Eigen::Vector2f(-rel_loc.y(), -rel_loc.x()).cwiseProduct(SCALING) + CENTER;
  }

  cv::Mat ImageBasedEvaluator::GetPatchAtLocation(const cv::Mat& img, const cv::Point& coord, float* validity, bool filter_empty) {
    if ((coord.y - (ImageBasedEvaluator::HALF_PATCH_SIZE)) < img.rows * ImageBasedEvaluator::MIN_IMAGE_Y_PCT  ||
        (coord.x - (ImageBasedEvaluator::HALF_PATCH_SIZE)) < 0 ||
        (coord.y + (ImageBasedEvaluator::HALF_PATCH_SIZE)) >= img.rows ||
        (coord.x + (ImageBasedEvaluator::HALF_PATCH_SIZE)) >= img.cols) {
      return cv::Mat();
    }

    cv::Rect patchBounds(coord.x - ImageBasedEvaluator::HALF_PATCH_SIZE,
                         coord.y - ImageBasedEvaluator::HALF_PATCH_SIZE,
                         ImageBasedEvaluator::PATCH_SIZE,
                         ImageBasedEvaluator::PATCH_SIZE);


    cv::Mat patch = img(patchBounds);

    if (filter_empty) {
      cv::Mat patch_gray;
      cv::cvtColor(patch, patch_gray, cv::COLOR_BGR2GRAY);
      int pixelCount = ImageBasedEvaluator::PATCH_PIXEL_COUNT;
      int zeroPixels = pixelCount - cv::countNonZero(patch_gray);
      float val = float(zeroPixels) / pixelCount;
      if (val > ImageBasedEvaluator::PATCH_EMPTY_THRESHOLD) {
        return cv::Mat();
      }
      *validity = 1.0f - val;
    }

    return patch;
  }

  std::vector<cv::Mat> ImageBasedEvaluator::GetPatchesAtLocation(const cv::Mat& img, const Eigen::Vector2f& location, std::vector<float>* validities, bool blur, bool filter_empty) {
    Eigen::Vector2f image_loc = GetImageLocation(location);
    std::vector<cv::Mat> patches;
    std::vector<cv::Point> coords = {
      cv::Point(image_loc.x(), image_loc.y())
    };

    if (blur) {
      coords.emplace_back(image_loc.x() - ImageBasedEvaluator::HALF_PATCH_SIZE, image_loc.y() - ImageBasedEvaluator::HALF_PATCH_SIZE);
      coords.emplace_back(image_loc.x() - ImageBasedEvaluator::HALF_PATCH_SIZE, image_loc.y() + ImageBasedEvaluator::HALF_PATCH_SIZE);
      coords.emplace_back(image_loc.x() + ImageBasedEvaluator::HALF_PATCH_SIZE, image_loc.y() - ImageBasedEvaluator::HALF_PATCH_SIZE);
      coords.emplace_back(image_loc.x() + ImageBasedEvaluator::HALF_PATCH_SIZE, image_loc.y() + ImageBasedEvaluator::HALF_PATCH_SIZE);
    }
    
    for (size_t i = 0; i < coords.size(); i++) {
      float validity;
      cv::Mat patch = GetPatchAtLocation(img, coords[i], &validity, filter_empty);
      if (patch.rows > 0) {
        patches.push_back(patch);
        validities->push_back(validity);
      }
    }

    return patches;
  }



}  // namespace motion_primitives
