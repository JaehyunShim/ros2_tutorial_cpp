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
    srv_requested_(false)
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this, a, b]() {
        send_request(a, b);
      });
  }

  bool srv_requested_;
  int requested_srv_a_;
  int requested_srv_b_;
  int responded_srv_;

private:
  void send_request(int a, int b)
  {
    // Cancel timer and send a request only once
    timer_->cancel();

    // Wait for server to start up
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the service.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Define a service request
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    requested_srv_a_ = request->a;
    requested_srv_b_ = request->b;

    // Call async_send_request() method
    using ServiceResponseFuture = rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum);
        srv_requested_ = true;
        responded_srv_ = response->sum;
      };
    auto future_response = client_->async_send_request(request, response_received_callback);
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace service_tutorial_cpp

#endif  // SERVICE_TUTORIAL_CPP__CLIENT_HPP_
