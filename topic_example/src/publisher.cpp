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

#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "topic_example/msg/count.hpp"
#include "topic_example/visibility_control.h"

using namespace std::chrono_literals;

namespace topic_example
{
class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const rclcpp::NodeOptions & options)  // options for what ???
  : Node("publisher", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    count_pub_ = this->create_publisher<topic_example::msg::Count>("chatter", qos);

    // ROS Timer
    auto timer_callback =
      [this]() -> void
      {
        msg_ = std::make_unique<topic_example::msg::Count>();
        msg_->data = count_++;
        RCLCPP_INFO(this->get_logger(), "%d", msg_->data);
        count_pub_->publish(std::move(msg_));
      };
    timer_ = this->create_wall_timer(1s, timer_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized publisher node");
  }
  ~Publisher()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated publisher node");
  }

private:
  uint16_t count_ = 0;
  std::unique_ptr<topic_example::msg::Count> msg_;
  rclcpp::Publisher<topic_example::msg::Count>::SharedPtr count_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace topic_example

RCLCPP_COMPONENTS_REGISTER_NODE(topic_example::Publisher)
