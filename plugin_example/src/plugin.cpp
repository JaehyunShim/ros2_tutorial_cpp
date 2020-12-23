// Copyright (c) 2012, Willow Garage, Inc.
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

#include <chrono>
#include <memory>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"


#include "plugin_example/plugin_base.hpp"

using namespace std::chrono_literals;

namespace plugin_example
{
class Plugin : public PluginBase
{
public:
  Plugin()
  // : Node("plugin")
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    count_pub_ = this->create_publisher<std_msgs::msg::Int64>("chatter", qos);

    // ROS Timer
    auto timer_callback =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::Int64>();
        msg_->data = count_++;
        RCLCPP_INFO(this->get_logger(), "Publish data: %d", msg_->data);
        count_pub_->publish(std::move(msg_));
      };
    timer_ = this->create_wall_timer(1s, timer_callback);

    RCLCPP_INFO(this->get_logger(), "Initialized plugin node");
  }
  ~Plugin()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated plugin node");
  }

  void update()
  {
    RCLCPP_INFO(this->get_logger(), "Called update function");
  }

private:
  uint16_t count_ = 0;
  std::unique_ptr<std_msgs::msg::Int64> msg_;  // why unique pointer for this?
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace plugin_example

PLUGINLIB_EXPORT_CLASS(plugin_example::Plugin, plugin_example::PluginBase)
