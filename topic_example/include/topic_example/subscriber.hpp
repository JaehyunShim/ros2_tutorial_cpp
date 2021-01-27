// Copyright Open Source Robotics Foundation, Inc.
// Copyright 2021, Jaehyun Shim, ROBOTIS CO., LTD.
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

#ifndef TOPIC_EXAMPLE__SUBSCRIBER_HPP_
#define TOPIC_EXAMPLE__SUBSCRIBER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "topic_example/msg/count.hpp"

namespace topic_example
{
class Subscriber : public rclcpp::Node
{
public:
  explicit Subscriber(const rclcpp::NodeOptions & options)
  : Node("subscriber", options)
  {
    // ROS Subscriber
    auto count_callback =
      [this](const topic_example::msg::Count::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%d", msg->data);
        msg_received = true;
      };
    count_sub_ = create_subscription<topic_example::msg::Count>("chatter", 10, count_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized subscriber node");
  }

  ~Subscriber()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated subscriber node");
  }

  bool msg_received = false;

private:
  rclcpp::Subscription<topic_example::msg::Count>::SharedPtr count_sub_;
};
}  // namespace topic_example

#endif  // TOPIC_EXAMPLE__SUBSCRIBER_HPP_
