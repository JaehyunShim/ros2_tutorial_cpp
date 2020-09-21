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

#include "service_example/srv/inquiry.hpp"
#include "service_example/visibility_control.h"

using namespace std::chrono_literals;

namespace service_example
{
class Client : public rclcpp::Node
{
public:
  explicit Client(const rclcpp::NodeOptions & options)
  : Node("Client", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Client
    inquiry_cli_ = create_client<service_example::srv::Inquiry>("inquiry");

    RCLCPP_INFO(this->get_logger(), "Initialized client node");
    while (1) {
      queue_async_request();
    }
  }

  ~Client()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated client node");
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
    auto request = std::make_shared<service_example::srv::Inquiry::Request>();
    request->question = "ryan smart?";

    // Call async_send_request() method
    using ServiceResponseFuture = rclcpp::Client<service_example::srv::Inquiry>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Received Response: %s", result->answer.c_str());
        rclcpp::shutdown();
      };
    auto future_result = inquiry_cli_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<service_example::srv::Inquiry>::SharedPtr inquiry_cli_;
};
}  // namespace service_example

RCLCPP_COMPONENTS_REGISTER_NODE(service_example::Client)
