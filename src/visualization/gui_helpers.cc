// Copyright (c) 2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "gui_helpers.h"

#include "glog/logging.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace {
  int marker_id_ = 0;
}  // namespace
namespace gui_helpers {

const Color4f Color4f::kRed(1, 0, 0, 1);
const Color4f Color4f::kGreen(0, 1, 0, 1);
const Color4f Color4f::kBlue(0, 0, 1, 1);
const Color4f Color4f::kWhite(1, 1, 1, 1);
const Color4f Color4f::kBlack(0, 0, 0, 1);
const Color4f Color4f::kYellow(1, 1, 0, 1);
const Color4f Color4f::kCyan(0, 1, 1, 1);
const Color4f Color4f::kMagenta(1, 0, 1, 1);

void InitializeMarker(
    int marker_type,
    const Color4f& color,
    float scale_x,
    float scale_y,
    float scale_z,
    visualization_msgs::Marker* msg) {
  msg->id = marker_id_;
  ++marker_id_;
  msg->type = marker_type;
  msg->action = visualization_msgs::Marker::ADD;
  msg->pose.position.x = 0;
  msg->pose.position.y = 0;
  msg->pose.position.z = 0;
  msg->pose.orientation.x = 0.0;
  msg->pose.orientation.y = 0.0;
  msg->pose.orientation.z = 0.0;
  msg->pose.orientation.w = 1.0;
  msg->scale.x = scale_x;
  msg->scale.y = scale_y;
  msg->scale.z = scale_z;
  msg->header.frame_id = "map";
}

}  // namespace gui_helpers


