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

#include "rcutils/logging_macros.h"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace lifecycle_example
{
class LifecycleSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleSubscriber(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("lifecycle_subscriber", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(this->get_logger(), "Initialized lifecycle subscriber node");
  }

  ~LifecycleSubscriber()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated lifecycle subscriber node");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    // ROS Subscriber
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    msg_sub_ = this->create_subscription<std_msgs::msg::String>(
      "lifecycle_chatter",
      qos,
      std::bind(&LifecycleSubscriber::msg_callback, this, std::placeholders::_1));
    notification_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lifecycle_publisher/transition_event",
      qos,
      std::bind(&LifecycleSubscriber::notification_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    // Activate the lifecycle publisher, if there are
    // rclcpp_lifecycle::LifecycleSubscription does not exist

    // Diference from RCLCPP_INFO()??
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    std::this_thread::sleep_for(2s);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Deactivate the lifecycle publisher, if there are
    // rclcpp_lifecycle::LifecycleSubscription does not exist

    // Diference from RCLCPP_INFO()??
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    // Release the shared pointers to subscriber
    // These entities are no longer available
    msg_sub_.reset();
    notification_sub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    // Release the shared pointers to subscriber
    // These entities are no longer available
    msg_sub_.reset();
    notification_sub_.reset();

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void msg_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "msg_callback: %s", msg->data.c_str());
  }

  void notification_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(),
      "notification_callback: Transition from state %s to %s",
      msg->start_state.label.c_str(),
      msg->goal_state.label.c_str());
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr notification_sub_;
};
}  // namespace lifecycle_example

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_example::LifecycleSubscriber)
