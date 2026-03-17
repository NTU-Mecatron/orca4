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

#include "orca_nav2/util.hpp"

#include <cmath>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca
{

double dist_sq(double x, double y)
{
  return x * x + y * y;
}

double dist(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return dist(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

bool transform_with_tolerance(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & tolerance)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Interpolate
    out_pose = tf->transform(in_pose, frame);
    return true;
  } catch (const tf2::ExtrapolationException & e) {
    // Use the most recent transform if possible
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      tolerance)
    {
      RCLCPP_ERROR(
        logger, "Transform too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(
        logger, "Data: %ds %uns, transform: %ds %uns",
        in_pose.header.stamp.sec, in_pose.header.stamp.nanosec,
        transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return false;
  }
}

}  // namespace orca
