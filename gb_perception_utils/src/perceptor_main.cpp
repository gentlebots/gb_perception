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

#include <memory>

#include "gb_perception_utils/Perceptor3D.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto perceptor_3d_node = rclcpp::Node::make_shared("perceptor_3d_node");
  auto perceptor_3d = std::make_shared<gb_perception_utils::Perceptor3D>(
    perceptor_3d_node);

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    rclcpp::spin_some(perceptor_3d_node);

    auto point3d = perceptor_3d->get_3d_from_pixel(
      160, 120, perceptor_3d_node->now() - rclcpp::Duration(1s), "base_footprint");
    std::cerr << "Pose in base_footprint is (" << point3d.x() << ", " <<
      point3d.y() << ", " << point3d.z() << ")" << std::endl;

    rate.sleep();
  }

  rclcpp::spin(perceptor_3d_node);

  rclcpp::shutdown();

  return 0;
}
