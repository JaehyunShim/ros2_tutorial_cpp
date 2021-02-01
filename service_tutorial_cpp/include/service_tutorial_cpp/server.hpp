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

#ifndef SERVICE_TUTORIAL_CPP__SERVER_HPP_
#define SERVICE_TUTORIAL_CPP__SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace service_tutorial_cpp
{
class Server : public rclcpp::Node  // inherit from Node
{
public:
  Server()
  : Node("server"),  // name the node "server"
    srv_responded_(false)
  {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&Server::service_callback, this, _1, _2));  // to use a non static member function
  }

  bool srv_responded_;
  int requested_srv_a_;
  int requested_srv_b_;
  int responded_srv_;

private:
  void service_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(
      this->get_logger(), "Incoming request \na: %ld b: %ld",
      request->a, request->b);
    RCLCPP_INFO(
      this->get_logger(), "Sending back response: [%ld]",
      response->sum);
    srv_responded_ = true;
    requested_srv_a_ = request->a;
    requested_srv_b_ = request->b;
    responded_srv_ = response->sum;
  }
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};
}  // namespace service_tutorial_cpp

#endif  // SERVICE_TUTORIAL_CPP__SERVER_HPP_
