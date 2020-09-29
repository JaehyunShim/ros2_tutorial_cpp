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

/* Authors: Jaehyun Shim */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "topic_example/msg/count.hpp"
#include "topic_example/visibility_control.h"

namespace topic_example
{
class Subscriber : public rclcpp::Node
{
public:
  explicit Subscriber(const rclcpp::NodeOptions & options)
  : Node("subscriber", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Subscriber
    auto count_callback =
      [this](const topic_example::msg::Count::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%d", msg->data);
      };
    count_sub_ = create_subscription<topic_example::msg::Count>("chatter", 10, count_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized subscriber node");
  }

  ~Subscriber()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated subscriber node");
  }

private:
  rclcpp::Subscription<topic_example::msg::Count>::SharedPtr count_sub_;
};
}  // namespace topic_example

RCLCPP_COMPONENTS_REGISTER_NODE(topic_example::Subscriber)
