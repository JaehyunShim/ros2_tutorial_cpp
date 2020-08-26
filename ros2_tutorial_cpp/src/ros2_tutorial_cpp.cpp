// Copyright 2020 ROBOTIS CO., LTD.
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

/* Authors: Ryan Shim */

#include "ros2_tutorial_cpp/ros2_tutorial_cpp.hpp"

#include <memory>

namespace ros2_tutorial_cpp
{
ROS2TutorialCPP::ROS2TutorialCPP()
: Node("ros2_tutorial_cpp")
{
  RCLCPP_INFO(this->get_logger(), "Initialized ROS2 Tutorial.");
}

ROS2TutorialCPP::~ROS2TutorialCPP()
{
  RCLCPP_INFO(this->get_logger(), "Terminated ROS2 Tutorial.");
}
}  // namespace ros2_tutorial_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_tutorial_cpp::ROS2TutorialCPP>());
  rclcpp::shutdown();

  return 0;
}
