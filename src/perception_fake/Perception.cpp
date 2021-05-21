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

#include <string>

#include "gazebo_msgs/ModelStates.h"
#include "vision_msgs/Detection3DArray.h"
#include "vision_msgs/Detection3D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"
#include "geometry_msgs/Pose.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "perception_fake/Perception.hpp"

#include "ros/ros.h"


#define XTION_H_FOV (70.0 * M_PI / 180.0)
#define XTION_V_FOV (60.0 * M_PI / 180.0)

namespace perception_fake {

Perception::Perception()
: nh_(), buffer_(), listener_(buffer_)
{
  gazebo_model_sub_ = nh_.subscribe("/gazebo/model_states", 100, &Perception::gazebo_model_callback, this);
  detection_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("/dope/detected_objects", 100);
}

void
Perception::gazebo_model_callback(const gazebo_msgs::ModelStates::ConstPtr & msg)
{
  // if (detection_pub_.getNumSubscribers() == 0) {
  //   return;
  // }

  // Find Tiago
  tf2::Transform world2tiago;
  bool tiago_found = false;
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "tiago") {
      tiago_found = true;
      tf2::fromMsg(msg->pose[i], world2tiago);
    }
  }

  if (!tiago_found) return;

  vision_msgs::Detection3DArray out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "base_footprint";

  for (int i = 0; i < msg->name.size(); i++) {
    const std::string & name = msg->name[i];
    const geometry_msgs::Pose & pose = msg->pose[i];

    tf2::Transform world2item;
    tf2::fromMsg(pose, world2item);
 
    std::string counter_id;
    std::string class_id;

    if (name.find("task") != std::string::npos) {
      size_t pos_1 = name.rfind("_");
      size_t pos_ycb = name.find("ycb_");
      size_t pos_2 = name.find("_", pos_ycb + 4);

      counter_id = name.substr(pos_1 + 1);
      class_id = name.substr(pos_2 + 1, pos_1 - pos_2 - 2);


      if (counter_id != class_id) {
        vision_msgs::Detection3D detection;
        detection.header.stamp = ros::Time::now();
        detection.header.frame_id = "base_footprint";
        vision_msgs::ObjectHypothesisWithPose hypo;
        hypo.id = get_id(class_id);
        hypo.score = 1.0;
        
        tf2::Transform tiago2item = world2tiago.inverse() * world2item;
        tf2::toMsg(tiago2item, hypo.pose.pose);
        tf2::Stamped<tf2::Transform> tiago2item_stamped(tiago2item, ros::Time::now(), "base_footprint");
        if (hypo.id >= 0 && is_in_fov(tiago2item_stamped)) {
          detection.results.push_back(hypo);
          out_msg.detections.push_back(detection);
        }
      }
    }

    detection_pub_.publish(out_msg);
  }
}

int
Perception::get_id(const std::string & class_id)
{
  for (const auto & pair : class_ids) {
    if (class_id.find(pair.first) != std::string::npos) { 
      return pair.second;
    }
  }

  return -1;
}

bool
Perception::is_in_fov(const tf2::Stamped<tf2::Transform> & tiago2item)
{
  std::string error;
  if (buffer_.canTransform(tiago2item.frame_id_, "xtion_link", tiago2item.stamp_, ros::Duration(0.1), &error))
  {
    geometry_msgs::TransformStamped tiago2xtion_msg;
    tf2::Stamped<tf2::Transform> tiago2xtion;
    tf2::Transform xtion2item;

    tiago2xtion_msg = buffer_.lookupTransform(tiago2item.frame_id_, "xtion_link", tiago2item.stamp_);
    tf2::fromMsg(tiago2xtion_msg, tiago2xtion);
 
    xtion2item = tiago2xtion.inverse() * tiago2item;

    double yaw = atan2(xtion2item.getOrigin().y(), xtion2item.getOrigin().x());
    double pitch = atan2(xtion2item.getOrigin().z(), xtion2item.getOrigin().x());

    return fabs(yaw) < XTION_H_FOV / 2 && fabs(pitch) < XTION_V_FOV / 2;
  }

  return false;

}

}  // namespace perception_fake

