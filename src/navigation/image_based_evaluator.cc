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
\file    image_based_evaluator.cc
\brief   Path rollout evaluator using images.
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
    #if SIMULATION_MODE
      return local_img;
    #endif
    cv::Mat image_undistorted = image.clone();
    cv::undistort(image.clone(), image_undistorted, cameraMatrix, distortionMatrix);
    cv::Mat warped;
    cv::warpPerspective(image_undistorted, warped, homography, image_undistorted.size());
    return warped;
  }

  Eigen::Vector2f ImageBasedEvaluator::GetImageLocation(const Eigen::Vector2f& rel_loc) {
    return Eigen::Vector2f(-rel_loc.y(), -rel_loc.x()).cwiseProduct(SCALING) + CENTER;
  }

  cv::Mat ImageBasedEvaluator::GetPatchAtLocation(const cv::Mat& img, const Eigen::Vector2f& location, float* validity, bool filter_empty) {
    Eigen::Vector2f image_loc = GetImageLocation(location);
    return GetPatchAtImageLocation(img, image_loc, validity, filter_empty);
  }

  std::vector<Eigen::Vector2f> ImageBasedEvaluator::GetWheelLocations(const pose_2d::Pose2Df& pose, float robot_width, float robot_length) {
    std::vector<Eigen::Vector2f> image_locs;
    // center
    Eigen::Vector2f center_loc = GetImageLocation(pose.translation);
    image_locs.push_back(center_loc);
    // front left wheel
    Eigen::Vector2f fl_vec(robot_length / 2, robot_width / 2);
    Eigen::Vector2f fl_loc = GetImageLocation(pose.translation + Eigen::Rotation2Df(pose.angle) * fl_vec * ROBOT_SIZE_SAMPLE_SCALING);
    image_locs.push_back(fl_loc);
    // front right wheel
    Eigen::Vector2f fr_vec(robot_length / 2, -robot_width / 2);
    Eigen::Vector2f fr_loc = GetImageLocation(pose.translation + Eigen::Rotation2Df(pose.angle) * fr_vec * ROBOT_SIZE_SAMPLE_SCALING);
    image_locs.push_back(fr_loc);
    // back left wheel
    Eigen::Vector2f bl_vec(-robot_length / 2, robot_width / 2);
    Eigen::Vector2f bl_loc = GetImageLocation(pose.translation + Eigen::Rotation2Df(pose.angle) * bl_vec * ROBOT_SIZE_SAMPLE_SCALING);
    image_locs.push_back(bl_loc);
    //back right wheel
    Eigen::Vector2f br_vec(-robot_length / 2, -robot_width / 2);
    Eigen::Vector2f br_loc = GetImageLocation(pose.translation + Eigen::Rotation2Df(pose.angle) * br_vec * ROBOT_SIZE_SAMPLE_SCALING);
    image_locs.push_back(br_loc);

    return image_locs;
  }

  std::vector<cv::Mat> ImageBasedEvaluator::GetPatchesAtPose(const cv::Mat& img, const pose_2d::Pose2Df& pose, std::vector<Eigen::Vector2f>* image_locations, std::vector<float>* validities, bool filter_empty, float robot_width, float robot_length) {
    std::vector<Eigen::Vector2f> image_locs = GetWheelLocations(pose, robot_width, robot_length);
    std::vector<cv::Mat> patches;
    // printf("LOCATIONS c: (%0.2f %0.2f) fl: (%0.2f %0.2f) fr: (%0.2f %0.2f) bl (%0.2f, %0.2f) br (%0.2f %f)\n",
    //       center_loc[0], center_loc[1], fl_loc[0], fl_loc[1], fr_loc[0], fr_loc[1], bl_loc[0], bl_loc[1], br_loc[0], br_loc[1]);

    for(auto loc : image_locs) {
      float validity;
      auto patch = GetPatchAtImageLocation(img, loc, &validity, filter_empty);
      patches.push_back(patch.clone());
      validities->emplace_back(validity);
      image_locations->push_back(loc);
    }

    return patches;
  }

  cv::Mat ImageBasedEvaluator::GetPatchAtImageLocation(const cv::Mat& img, const Eigen::Vector2f& image_loc, float* validity, bool filter_empty) {
    cv::Point coord = cv::Point(image_loc.x(), image_loc.y());

    if ((coord.y - (ImageBasedEvaluator::PATCH_SIZE / 2)) < 0 ||
        (coord.x - (ImageBasedEvaluator::PATCH_SIZE / 2)) < 0 ||
        (coord.y + (ImageBasedEvaluator::PATCH_SIZE / 2)) >= img.rows ||
        (coord.x + (ImageBasedEvaluator::PATCH_SIZE / 2)) >= img.cols) {
      return cv::Mat();
    }

    cv::Rect patchBounds(coord.x - ImageBasedEvaluator::PATCH_SIZE / 2,
                         coord.y - ImageBasedEvaluator::PATCH_SIZE / 2,
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

  std::vector<Eigen::Vector2f> ImageBasedEvaluator::GetTilingLocations(const cv::Mat& img, const int tile_size) {
    std::vector<Eigen::Vector2f> locations;
    for(int i = 0; i < img.cols; i+= tile_size) {
      for(int j = (int)(img.rows * TILING_START_PCT); j < (int)(img.rows * TILING_END_PCT); j += tile_size) {
        locations.emplace_back(i, j);
      }
    }
    
    return locations;
  }

}  // namespace motion_primitives
