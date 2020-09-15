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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int8.hpp"

#include "ros2_tutorial_cpp/visibility_control.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorial_cpp
{
class Talker : public rclcpp::Node
{
public:
  Talker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    count_pub_ = this->create_publisher<std_msgs::msg::Int8>("chatter", qos);

    // ROS Timer
    auto timer_callback =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::Int8>();
        msg_->data = count_++;
        count_pub_->publish(std::move(msg_));
      };
    timer_ = this->create_wall_timer(10s, timer_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized Talker.");
  }
  ~Talker()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated Talker.");
  }

private:
  uint16_t count_ = 0;
  std::unique_ptr<std_msgs::msg::Int8> msg_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr count_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ros2_tutorial_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_tutorial_cpp::Talker)
