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

#ifndef TOPIC_TUTORIAL_CPP__PUBLISHER_MEMBER_FUNCTION_HPP_
#define TOPIC_TUTORIAL_CPP__PUBLISHER_MEMBER_FUNCTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace topic_tutorial_cpp
{
class PublisherMemberFunction : public rclcpp::Node
{
public:
  PublisherMemberFunction()
  : Node("publisher_member_function"),  // initialize Node with the name "publisher_member_function"
    msg_published_(false),
    count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_member_function", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PublisherMemberFunction::timer_callback, this));  // std::bind to use a non
                                                                   // static member function
  }

  bool msg_published_;
  std::string published_msg_;

private:
  void timer_callback()  // declare timer_callback as a member function
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());
    publisher_->publish(message);
    if (msg_published_ == false) {
      msg_published_ = true;
      published_msg_ = message.data;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
}  // namespace topic_tutorial_cpp

#endif  // TOPIC_TUTORIAL_CPP__PUBLISHER_MEMBER_FUNCTION_HPP_
