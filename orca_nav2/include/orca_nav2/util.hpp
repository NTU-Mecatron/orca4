// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ORCA_NAV2__UTIL_HPP_
#define ORCA_NAV2__UTIL_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "tf2_ros/buffer.h"

namespace orca
{

double dist_sq(double x, double y);

double dist(double x, double y, double z);

double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

bool transform_with_tolerance(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & tolerance);

}  // namespace orca

#endif  // ORCA_NAV2__UTIL_HPP_
