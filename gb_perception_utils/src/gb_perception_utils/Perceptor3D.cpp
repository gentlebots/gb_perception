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

#include <cv_bridge/cv_bridge.h>

#include <list>
#include <limits>
#include <string>
#include <utility>
#include <memory>

#include "image_geometry/pinhole_camera_model.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "gb_perception_utils/Perceptor3D.hpp"

#include "rclcpp/rclcpp.hpp"


namespace gb_perception_utils
{

using namespace std::chrono_literals;

#define MAX_BUFFER_SIZE 60

Perceptor3D::Perceptor3D(rclcpp::Node::SharedPtr node)
: node_(node), buffer_(), listener_(buffer_)
{
  info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    std::string(node_->get_name()) + "/camera_info", 1,
    std::bind(&Perceptor3D::info_callback, this, std::placeholders::_1));
}

void
Perceptor3D::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  image_buffer_.push_back(std::move(msg));

  if (image_buffer_.size() > MAX_BUFFER_SIZE) {
    image_buffer_.pop_front();
  }
}

void
Perceptor3D::info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (model_ == nullptr) {
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      std::string(node_->get_name()) + "/depth_image", 100,
      std::bind(&Perceptor3D::image_callback, this, std::placeholders::_1));

    info_sub_ = nullptr;

    model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  }
  model_->fromCameraInfo(msg);
}

std::optional<tf2::Vector3>
Perceptor3D::get_3d_from_pixel(
  double u, double v, rclcpp::Time ts,
  const std::string & target_frame_id)
{
  if (model_ == nullptr || image_buffer_.empty()) {
    return {};
  }
  double last_diff = std::numeric_limits<double>::max();

  auto it = image_buffer_.rbegin();
  auto selected_it = it;

  while (it != image_buffer_.rend()) {
    double current_diff = fabs((ts - rclcpp::Time((*it)->header.stamp)).seconds());

    if (current_diff > last_diff) {
      break;
    } else {
      selected_it = it;
      last_diff = current_diff;
      ++it;
    }
  }

  sensor_msgs::msg::Image & image = **selected_it;

  if (image.encoding != "32FC1") {
    RCLCPP_ERROR(node_->get_logger(), "The image type has not depth info");
    return {};
  } else {
    cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(image, image.encoding);
    float depth = cv_depth_ptr->image.at<float>(cv::Point2d(u, v));

    if (std::isnan(depth)) {
      return {};
    }

    cv::Point3d ray = model_->projectPixelTo3dRay(
      model_->rectifyPoint(cv::Point2d(u, v)));

    ray = ray / ray.z;

    cv::Point3d point = ray * depth;

    geometry_msgs::msg::TransformStamped image2target_msg;
    tf2::Stamped<tf2::Transform> image2target;

    image2target_msg = buffer_.lookupTransform(
      target_frame_id, image.header.frame_id, tf2_ros::fromMsg(
        image.header.stamp));
    tf2::fromMsg(image2target_msg, image2target);

    tf2::Vector3 point_tf(point.x, point.y, point.z);
    tf2::Vector3 p_bf = image2target * point_tf;

    return p_bf;
  }
}

}  // namespace gb_perception_utils
