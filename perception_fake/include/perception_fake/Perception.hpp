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

#ifndef PERCEPTION_FAKE__PERCEPTION__HPP_
#define PERCEPTION_FAKE__PERCEPTION__HPP_

#include <map>
#include <string>

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"

#include "gazebo_msgs/ModelStates.h"

#include "ros/ros.h"

namespace perception_fake {

static const std::map<std::string, int> class_ids {
    {"cracker", 1},
    {"gelatin", 2},
    {"meat",    3},
    {"mustard", 4},
    {"soup",    5},
    {"sugar",   6},
    {"bleach",  7},
    {"AlphabetSoup"      , 9},
    {"Ketchup"           , 10},
    {"Pineapple"         , 11},
    {"BBQSauce"          , 12},
    {"MacaroniAndCheese" , 13},
    {"Popcorn"           , 14},
    {"Butter"            , 15},
    {"Mayo"              , 16},
    {"Raisins"           , 17},
    {"Cherries"          , 18},
    {"Milk"              , 19},
    {"SaladDressing"     , 20},
    {"ChocolatePudding"  , 21},
    {"Mushrooms"         , 22},
    {"Spaghetti"         , 23},
    {"Cookies"           , 24},
    {"Mustard"           , 25},
    {"TomatoSauce"       , 26},
    {"Corn"              , 27},
    {"OrangeJuice"       , 28},
    {"Tuna"              , 29},
    {"CreamCheese"       , 20},
    {"Parmesan"          , 31},
    {"Yogurt"            , 32},
    {"GranolaBars"       , 33},
    {"Peaches"           , 34},
    {"GreenBeans"        , 35},
    {"PeasAndCarrots"    , 36}};

class Perception 
{
public:
  Perception();

private:
  ros::NodeHandle nh_;
  ros::Subscriber gazebo_model_sub_;
  ros::Publisher detection_pub_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  void gazebo_model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
  bool is_in_fov(const tf2::Stamped<tf2::Transform> & tiago2item);
  int get_id(const std::string & class_id);
};

}  // namespace perception_fake

#endif  // PERCEPTION_FAKE__PERCEPTION__HPP_