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
\file    navigation_tests.cc
\brief   Unit testing of navigation-related components
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================


#include <stdio.h>
#include <gtest/gtest.h>

#include "eigen3/Eigen/Dense"
#include "math/geometry.h"

#include "motion_primitives.h"
#include "navigation_parameters.h"

TEST(Run1DTimeOptimalControl, SimpleCases) {
  {
    navigation::MotionLimits limits(1, 1, 1);
    const float v_cmd = motion_primitives::Run1DTimeOptimalControl(
        limits, 0, 0, 2, 0, 0.1);
    EXPECT_FLOAT_EQ(v_cmd, 0.1);
  }
}


