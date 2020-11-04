// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef INTRA_PROCESS_EXAMPLE__PUBLISHER_HPP_
#define INTRA_PROCESS_EXAMPLE__PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace intra_process_example
{
class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const std::string & name)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<std_msgs::msg::Int64>("chatter", qos);

    // ROS Timer
    auto timer_callback =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::Int64>();
        msg_->data = count_++;
        RCLCPP_INFO(this->get_logger(), "%d", msg_->data);
        pub_->publish(std::move(msg_));
      };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized publisher node");
  }

  ~Publisher()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated publisher node");
  }

private:
  uint16_t count_ = 0;
  std::unique_ptr<std_msgs::msg::Int64> msg_;  // why unique pointer for this?
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace intra_process_example

#endif  // INTRA_PROCESS_EXAMPLE__PUBLISHER_HPP_
