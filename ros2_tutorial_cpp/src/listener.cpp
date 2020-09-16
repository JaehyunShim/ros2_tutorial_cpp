// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int8.hpp"

#include "ros2_tutorial_cpp/visibility_control.hpp"

namespace ros2_tutorial_cpp
{
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Subscriber
    auto count_callback =
      [this](const std_msgs::msg::Int8::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%d", msg->data);
      };
    count_sub_ = create_subscription<std_msgs::msg::Int8>("chatter", 10, count_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized Listener.");
  }

  ~Listener()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated Listener.");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr count_sub_;
};
}  // namespace ros2_tutorial_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_tutorial_cpp::Listener)
