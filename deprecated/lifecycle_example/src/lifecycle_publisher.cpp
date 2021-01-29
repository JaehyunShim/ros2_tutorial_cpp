// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "rcutils/logging_macros.h"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace lifecycle_example
{
class LifecyclePublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecyclePublisher(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("lifecycle_publisher", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(this->get_logger(), "Initialized lifecycle publisher node");
  }

  ~LifecyclePublisher()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated lifecycle publisher node");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // ROS Publisher
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    msg_pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", qos);

    // ROS Timer
    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecyclePublisher::timer_callback, this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Transition callback for state activating
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    // Activate the lifecycle publisher
    msg_pub_->on_activate();

    // Diference from RCLCPP_INFO()??
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    std::this_thread::sleep_for(2s);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Deactivate the lifecycle publisher
    msg_pub_->on_deactivate();

    // Diference from RCLCPP_INFO()??
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    // Release the shared pointers to the timer and publisher
    // These entities are no longer available
    timer_.reset();
    msg_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    // Release the shared pointers to the timer and publisher
    // These entities are no longer available
    timer_.reset();
    msg_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void timer_callback()
  {
    static uint64_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle message #" + std::to_string(++count);

    // Print the current state for demo purposes
    if (!msg_pub_->is_activated()) {
      RCLCPP_INFO(
        get_logger(),
        "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Lifecycle publisher is active. Publishing: [%s]",
        msg->data.c_str());
    }

    msg_pub_->publish(std::move(msg));
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr msg_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace lifecycle_example

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_example::LifecyclePublisher)
