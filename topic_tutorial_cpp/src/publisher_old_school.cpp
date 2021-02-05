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

#include <chrono>  // To use chrono
#include <string>  // To use string

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;  // using namespace can be used in *.cpp files.

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // initialize rclcpp
  auto node = rclcpp::Node::make_shared("publisher_not_composable");  // initialize node
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "topic_old_school", 10);  // initailize publisher
  std_msgs::msg::String message;
  auto publish_count = 0;
  // rclcpp::Rate loop_rate(500ms);  // use std::chrono::system_clock
  rclcpp::WallRate loop_rate(500ms);  // use std::chrono::steady_clock

  while (rclcpp::ok()) {
    message.data = "Hello, world! " + std::to_string(publish_count++);  // append strings
    RCLCPP_INFO(node->get_logger(), "Publishing: %s", message.data.c_str());
    try {
      publisher->publish(message);
      rclcpp::spin_some(node);  // slightly different from ros::spinOnce()
                                // https://github.com/ros2/rclcpp/issues/471
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(node->get_logger(), "unexpecctedly failed with %s", e.what());
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
