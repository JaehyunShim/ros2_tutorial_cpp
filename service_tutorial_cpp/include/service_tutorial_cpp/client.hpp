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

#ifndef SERVICE_TUTORIAL_CPP__CLIENT_HPP_
#define SERVICE_TUTORIAL_CPP__CLIENT_HPP_

#include <chrono>  // to use std::chrono::seconds()
#include <cstdlib>  // to use atoll
#include <memory>  // to use smart pointers

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

namespace service_tutorial_cpp
{
class Client : public rclcpp::Node  // inherit from Node
{
public:
  Client(int a, int b)
  : Node("client"),  // name the node "client"
    srv_requested(false)
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    queue_async_request(a, b);
  }

  bool srv_requested;

private:
  void queue_async_request(int a, int b)
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    // wait for server to start up
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %ld", result->sum);
        srv_requested = true;
      };
    auto future_result = client_->async_send_request(request, response_received_callback);
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
}  // namespace service_tutorial_cpp

#endif  // SERVICE_TUTORIAL_CPP__CLIENT_HPP_
