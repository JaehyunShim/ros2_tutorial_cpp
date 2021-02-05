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

#ifndef PARAMETER_TUTORIAL_CPP__PARAMETER_HPP_
#define PARAMETER_TUTORIAL_CPP__PARAMETER_HPP_

#include <chrono>  // to use chrono
#include <functional>  // to use std::bind
#include <string>  // to use std::string

#include "rclcpp/rclcpp.hpp"

namespace parameter_tutorial_cpp
{
class Parameter : public rclcpp::Node  // public inherit from rclcpp::Node
{
public:
  Parameter()
  : Node("parameter"),  // initialize 'Node' with the name 'parameter'
    parameter_updated_(false)
  {
    this->declare_parameter<std::string>("my_parameter", "world");  // declare a parameter
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Parameter::update, this));
  }

  void update()
  {
    // Get parameters
    this->get_parameter("my_parameter", parameter_string_);
    RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());

    // Set parameter
    std::string new_parameter_string = "Korea";
    this->set_parameter(rclcpp::Parameter("my_parameter", new_parameter_string));

    // For test codes
    if (parameter_updated_ == false) {
      parameter_updated_ = true;
      init_parameter_ = parameter_string_;
      new_parameter_ = new_parameter_string;
      this->get_parameter("my_parameter", updated_parameter_);
    }
  }

  bool parameter_updated_;
  std::string init_parameter_;
  std::string new_parameter_;
  std::string updated_parameter_;

private:
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace parameter_tutorial_cpp

#endif  // PARAMETER_TUTORIAL_CPP__PARAMETER_HPP_
