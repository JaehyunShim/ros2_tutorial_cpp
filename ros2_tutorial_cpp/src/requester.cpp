// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ros2_tutorial_cpp/srv/inquiry.hpp"

#include "ros2_tutorial_cpp/visibility_control.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorial_cpp
{
class Requester : public rclcpp::Node
{
public:
  explicit Requester(const rclcpp::NodeOptions & options)
  : Node("Requester", options)
  {
    // Control stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Client
    inquiry_cli_ = create_client<ros2_tutorial_cpp::srv::Inquiry>("inquiry");

    RCLCPP_INFO(this->get_logger(), "Initialized requester node");
    queue_async_request();
  }

  ~Requester()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated requester node");
  }

  void queue_async_request()
  {
    // Block until a service is available
    while (!inquiry_cli_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Define a service request
    auto request = std::make_shared<ros2_tutorial_cpp::srv::Inquiry::Request>();
    request->question = "ryan smart?";
    RCLCPP_INFO(this->get_logger(), "Request: %s", request->question.c_str());

    // Call async_send_request() method
    using ServiceResponseFuture = rclcpp::Client<ros2_tutorial_cpp::srv::Inquiry>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Response: %s", result->answer.c_str());
        rclcpp::shutdown();
      };
    auto future_result = inquiry_cli_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<ros2_tutorial_cpp::srv::Inquiry>::SharedPtr inquiry_cli_;
};
}  // namespace ros2_tutorial_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_tutorial_cpp::Requester)
