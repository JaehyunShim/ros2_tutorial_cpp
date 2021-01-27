// Copyright Open Source Robotics Foundation, Inc.
// Copyright 2021, Jaehyun Shim, ROBOTIS CO., LTD.
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

#ifndef ACTION_EXAMPLE__ACTION_CLIENT_HPP_
#define ACTION_EXAMPLE__ACTION_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_example/action/fetch.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Fetch = action_example::action::Fetch;
using GoalHandleFetch = rclcpp_action::ClientGoalHandle<Fetch>;

namespace action_example
{
class ActionClient : public rclcpp::Node
{
public:
  explicit ActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("action_client", node_options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Action Client
    this->fetch_action_cli_ = rclcpp_action::create_client<Fetch>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fetch");

    // ROS Timer
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ActionClient::send_goal, this));

    RCLCPP_INFO(this->get_logger(), "Initialized action client node");
  }

  ~ActionClient()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated action client node");
  }

  void send_goal()
  {
    // Cancel timer and send a goal only once
    timer_->cancel();

    // Block until a action service is available
    if (!fetch_action_cli_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    // Define an action goal  (do not know why not using shared pointer)
    auto goal_msg = Fetch::Goal();
    goal_msg.task_time = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fetch>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionClient::result_callback, this, _1);
    this->fetch_action_cli_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fetch>::SharedPtr fetch_action_cli_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleFetch::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFetch::SharedPtr,
    const std::shared_ptr<const Fetch::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received left time %d", feedback->time_left);
  }

  void result_callback(const GoalHandleFetch::WrappedResult & result)
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

    RCLCPP_INFO(this->get_logger(), "Received result %s", result.result->answer.c_str());

    rclcpp::shutdown();
  }
};  // class ActionClient
}  // namespace action_example

#endif  // ACTION_EXAMPLE__ACTION_CLIENT_HPP_
