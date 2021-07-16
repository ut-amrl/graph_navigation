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
    cv::undistort(image, image_undistorted, cameraMatrix, distortionMatrix);
    cv::Mat warped;
    cv::warpPerspective(image_undistorted, warped, homography, image_undistorted.size());
    return warped;
  }

  cv::Mat ImageBasedEvaluator::GetPatchAtLocation(const cv::Mat& img, Eigen::Vector2f location) {
    Eigen::Vector2f transformed = Eigen::Vector2f(-location.y(), -location.x()).cwiseProduct(SCALING) + CENTER;

    cv::Point coord = cv::Point(transformed.x(), transformed.y());

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

    return img(patchBounds);
  }


}  // namespace motion_primitives
