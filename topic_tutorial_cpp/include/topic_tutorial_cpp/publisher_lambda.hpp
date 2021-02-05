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

// Use below to make this header compiled only once
#ifndef TOPIC_TUTORIAL_CPP__PUBLISHER_LAMBDA_HPP_
#define TOPIC_TUTORIAL_CPP__PUBLISHER_LAMBDA_HPP_

#include <chrono>  // To use chrono in the cpp file
#include <memory>  // To use smart pointers
#include <string>  // To use string in the cpp file

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;  // using namespace cannot be used in hpp files

namespace topic_tutorial_cpp  // namespace
{
class PublisherLambda : public rclcpp::Node  // inherit from rclcpp::Node
{
public:
  PublisherLambda()  // constructor
  : Node("publisher_lambda"),  // initialize Node with the name "publisher_lambda"
    msg_published_(false),
    count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_lambda", 10);
    auto timer_callback =  // declare timer_callback as a lambda expression
      [this]() -> void {  // [catch](parameter) -> return { body }
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);  // int to string
        RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());  // str to char*
        this->publisher_->publish(message);
        if (msg_published_ == false) {
          msg_published_ = true;
          published_msg_ = message.data;
        }
      };
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),  // call timer_callback every 500ms
      timer_callback);
  }

  bool msg_published_;
  std::string published_msg_;

private:
  rclcpp::TimerBase::SharedPtr timer_;  // declare a timer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // declare a publisher
  size_t count_;  // unsigned max-data size
};
}  // namespace topic_tutorial_cpp

#endif  // TOPIC_TUTORIAL_CPP__PUBLISHER_LAMBDA_HPP_
