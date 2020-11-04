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

#ifndef INTRA_PROCESS_EXAMPLE__SUBSCRIBER_HPP_
#define INTRA_PROCESS_EXAMPLE__SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace intra_process_example
{
class Subscriber : public rclcpp::Node
{
public:
  explicit Subscriber(const std::string & name)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Subscriber
    auto sub_callback =
      [this](const std_msgs::msg::Int64::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%d", msg->data);
      };
    sub_ = create_subscription<std_msgs::msg::Int64>("chatter", 10, sub_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized subscriber node");
  }

  ~Subscriber()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated subscriber node");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
};
}  // namespace intra_process_example

#endif  // INTRA_PROCESS_EXAMPLE__SUBSCRIBER_HPP_
