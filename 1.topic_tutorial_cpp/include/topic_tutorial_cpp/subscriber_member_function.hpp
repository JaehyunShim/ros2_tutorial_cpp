// Copyright 2016 Open Source Robotics Foundation, Inc.
// Copyright 2021 Jaehyun Shim
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

#ifndef TOPIC_TUTORIAL_CPP__SUBSCRIBER_MEMBER_FUNCTION_HPP_
#define TOPIC_TUTORIAL_CPP__SUBSCRIBER_MEMBER_FUNCTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;  // using namespace cannot be used in hpp files

namespace topic_tutorial_cpp
{
class SubscriberMemberFunction : public rclcpp::Node
{
public:
  SubscriberMemberFunction()
  : Node("subscriber_member_function"),  // initialize Node with name "subscriber_member_function"
    msg_received_(false)
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "topic_member_function", 10, std::bind(&SubscriberMemberFunction::topic_callback, this, _1));
  }

  bool msg_received_;
  std::string received_msg_;

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard '%s'", msg->data.c_str());
    if (msg_received_ == false) {
      msg_received_ = true;
      received_msg_ = msg->data;
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};
}  // namespace topic_tutorial_cpp

#endif  // TOPIC_TUTORIAL_CPP__SUBSCRIBER_MEMBER_FUNCTION_HPP_
