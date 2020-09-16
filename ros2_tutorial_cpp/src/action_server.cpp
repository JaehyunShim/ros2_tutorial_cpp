// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ros2_tutorial_cpp/action/fetch.hpp"
#include "ros2_tutorial_cpp/visibility_control.hpp"

namespace ros2_tutorial_cpp
{
class ActionServer : public rclcpp::Node
{
public:
  using Fetch = ros2_tutorial_cpp::action::Fetch;
  using GoalHandleFetch = rclcpp_action::ServerGoalHandle<Fetch>;

  ROS2_TUTORIAL_CPP_PUBLIC
  explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_server", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initialized action server node");
    // using namespace std::placeholders;

    // this->action_server_ = rclcpp_action::create_server<Fetch>(
    //   this->get_node_base_interface(),
    //   this->get_node_clock_interface(),
    //   this->get_node_logging_interface(),
    //   this->get_node_waitables_interface(),
    //   "fetch",
    //   std::bind(&ActionServer::handle_goal, this, _1, _2),
    //   std::bind(&ActionServer::handle_cancel, this, _1),
    //   std::bind(&ActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fetch>::SharedPtr action_server_;

  // ROS2_TUTORIAL_CPP_LOCAL
  // rclcpp_action::GoalResponse handle_goal(
  //   const rclcpp_action::GoalUUID & uuid,
  //   std::shared_ptr<const Fetch::Goal> goal)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
  //   (void)uuid;
  //   // Let's reject sequences that are over 9000
  //   if (goal->order > 9000) {
  //     return rclcpp_action::GoalResponse::REJECT;
  //   }
  //   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  // }

  // ROS2_TUTORIAL_CPP_LOCAL
  // rclcpp_action::CancelResponse handle_cancel(
  //   const std::shared_ptr<GoalHandleFetch> goal_handle)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  //   (void)goal_handle;
  //   return rclcpp_action::CancelResponse::ACCEPT;
  // }

  // ROS2_TUTORIAL_CPP_LOCAL
  // void handle_accepted(const std::shared_ptr<GoalHandleFetch> goal_handle)
  // {
  //   using namespace std::placeholders;
  //   // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  //   std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
  // }

  // ROS2_TUTORIAL_CPP_LOCAL
  // void execute(const std::shared_ptr<GoalHandleFetch> goal_handle)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Executing goal");
  //   rclcpp::Rate loop_rate(1);
  //   const auto goal = goal_handle->get_goal();
  //   auto feedback = std::make_shared<Fetch::Feedback>();
  //   auto & sequence = feedback->partial_sequence;
  //   sequence.push_back(0);
  //   sequence.push_back(1);
  //   auto result = std::make_shared<Fetch::Result>();

  //   for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
  //     // Check if there is a cancel request
  //     if (goal_handle->is_canceling()) {
  //       result->sequence = sequence;
  //       goal_handle->canceled(result);
  //       RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //       return;
  //     }
  //     // Update sequence
  //     sequence.push_back(sequence[i] + sequence[i - 1]);
  //     // Publish feedback
  //     goal_handle->publish_feedback(feedback);
  //     RCLCPP_INFO(this->get_logger(), "Publish feedback");

  //     loop_rate.sleep();
  //   }

  //   // Check if goal is done
  //   if (rclcpp::ok()) {
  //     result->sequence = sequence;
  //     goal_handle->succeed(result);
  //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  //   }
  // }
};  // class ActionServer
}  // namespace ros2_tutorial_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_tutorial_cpp::ActionServer)
