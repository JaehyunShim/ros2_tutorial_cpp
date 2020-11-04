// Copyright 2020 ROBOTIS CO., LTD.
// Copyright 2020, Jaehyun Shim, ROBOTIS CO., LTD.
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

#include "launch_example/launch.hpp"

#include <memory>

#include <utility>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace launch_example
{
Launch::Launch()
: Node("launch_example")
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // ROS Publisher
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  pub_ = this->create_publisher<std_msgs::msg::Int64>("chatter", qos);

  // ROS Timer
  timer_ = this->create_wall_timer(
    1ms,
    std::bind(&Launch::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Initialized launch example node");
}

Launch::~Launch()
{
  RCLCPP_INFO(this->get_logger(), "Terminated launch example node");
}

void Launch::timer_callback()
{
  // Publish topic
  static uint16_t count_ = 0;
  msg_ = std::make_unique<std_msgs::msg::Int64>();
  msg_->data = count_++;
  pub_->publish(std::move(msg_));

  // Jitter test
  rclcpp::Time time_now = rclcpp::Clock().now();
  RCLCPP_INFO(
    this->get_logger(),
    "[Jitter Test] current time: %ld",
    static_cast<uint64_t>(time_now.nanoseconds()));
}
}  // namespace launch_example

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<launch_example::Launch>());
  rclcpp::shutdown();

  return 0;
}
