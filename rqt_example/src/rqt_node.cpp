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

#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rqt_example/rqt_node.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace rqt_example
{
RqtNode::RqtNode()
: Node("rqt_example")
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // ROS Publisher & Subscriber
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);
  chatter_sub_ =
    this->create_subscription<std_msgs::msg::String>(
    "chatter",
    qos,
    std::bind(&RqtNode::chatter_callback, this, std::placeholders::_1));

  // ROS Timer
  timer_ = this->create_wall_timer(
    1s,
    std::bind(&RqtNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Initialized rqt example node");
}

RqtNode::~RqtNode()
{
  RCLCPP_INFO(this->get_logger(), "Terminated rqt example node");
}

void RqtNode::timer_callback()
{
  static int count = 0;
  if (pub_onoff_ == true) {
    std::stringstream ss;
    ss << "hello world " << count;
    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    chatter_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publisher: %s", msg.data.c_str());
    count++;
  }
}

void RqtNode::chatter_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (sub_onoff_ == true) {
    RCLCPP_INFO(this->get_logger(), "Subscriber: %s", msg->data.c_str());
  }
}
}  // namespace rqt_example
