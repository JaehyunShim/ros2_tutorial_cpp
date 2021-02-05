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

#ifndef ACTION_TUTORIAL_CPP__ACTION_CLIENT_HPP_
#define ACTION_TUTORIAL_CPP__ACTION_CLIENT_HPP_

#include <functional>  // to use std::bind
#include <memory>  // to use smart pointers
#include <thread>  // to use std::thread

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using ClientGoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

namespace action_tutorial_cpp
{
class ActionClient : public rclcpp::Node  // public inherit from rclcpp::Node
{
public:
  ActionClient()
  : Node("action_client"),  // initialize the Node with the name "action_client"
    goal_sent_(false),
    feedback_received_(false),
    result_received_(false)
  {
    this->action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    this->timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ActionClient::send_goal, this));
  }

  bool goal_sent_;
  bool feedback_received_;
  bool result_received_;

  auto get_action_client()
  {
    return action_client_;
  }

private:
  void send_goal()
  {
    // Cancel timer and send an action goal only once
    this->timer_->cancel();

    // Wait for action server to start up
    if (!this->action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // Define an action goal
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    // Call async_send_goal() method
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionClient::result_callback, this, _1);
    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<ClientGoalHandleFibonacci::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");

      goal_sent_ = true;
    }
  }

  void feedback_callback(
    ClientGoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    feedback_received_ = true;
  }

  void result_callback(const ClientGoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    result_received_ = true;
  }
};
}  // namespace action_tutorial_cpp

#endif  // ACTION_TUTORIAL_CPP__ACTION_CLIENT_HPP_
