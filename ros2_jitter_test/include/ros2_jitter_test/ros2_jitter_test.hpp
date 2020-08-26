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

#ifndef ROS2_JITTER_TEST__ROS2_JITTER_TEST_HPP_
#define ROS2_JITTER_TEST__ROS2_JITTER_TEST_HPP_

#include <rclcpp/rclcpp.hpp>

namespace ros2_jitter_test
{

class ROS2JitterTest : public rclcpp::Node
{
public:
  ROS2JitterTest();
  virtual ~ROS2JitterTest();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};

}  // namespace ros2_jitter_test
#endif  // ROS2_JITTER_TEST__ROS2_JITTER_TEST_HPP_
