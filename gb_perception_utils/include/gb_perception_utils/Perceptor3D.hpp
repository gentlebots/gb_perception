// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GB_PERCEPTION_UTILS__PERCEPTOR3D_HPP_
#define GB_PERCEPTION_UTILS__PERCEPTOR3D_HPP_

#include <list>
#include <string>
#include <memory>

#include "image_geometry/pinhole_camera_model.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"

namespace gb_perception_utils
{

class Perceptor3D
{
public:
  explicit Perceptor3D(rclcpp::Node::SharedPtr node);

  tf2::Vector3 get_3d_from_pixel(
    int u, int v, rclcpp::Time ts,
    const std::string & target_frame_id);

private:
  rclcpp::Node::SharedPtr node_;

  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  void info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  std::list<sensor_msgs::msg::Image::UniquePtr> image_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  std::shared_ptr<image_geometry::PinholeCameraModel> model_;

  tf2::BufferCore buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace gb_perception_utils

#endif  // GB_PERCEPTION_UTILS__PERCEPTOR3D_HPP_
