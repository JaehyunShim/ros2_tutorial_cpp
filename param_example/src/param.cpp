// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "param_example/visibility_control.h"

using namespace std::chrono_literals;

namespace param_example
{
class Param : public rclcpp::Node
{
public:
  PARAM_EXAMPLE_PUBLIC  // Purpose of this? PARAM_EXAMPLE_PUBLIC
  explicit Param(const rclcpp::NodeOptions & options)
  : Node("param", options)
  {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ROS Parameter
    this->declare_parameter("name");
    this->declare_parameter("sim");
    this->declare_parameter("control_period");
    this->declare_parameter("control_mode");

    // Get parameters from yaml
    this->get_parameter_or<std::string>("name", name_, std::string("Robot"));
    this->get_parameter_or<bool>("sim", sim_, false);
    this->get_parameter_or<double>("control_period", control_period_, 0.050);
    this->get_parameter_or<uint8_t>("control_mode", control_mode_, 2);
    RCLCPP_INFO(this->get_logger(), "name: %s", name_.c_str());
    RCLCPP_INFO(this->get_logger(), "sim: %s", sim_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "control_period: %lf", control_period_);
    RCLCPP_INFO(this->get_logger(), "control_mode: %d", control_mode_);

    RCLCPP_INFO(this->get_logger(), "Initialized Param Node");

    set_parameter();
  }

  ~Param()
  {
    RCLCPP_INFO(this->get_logger(), "Terminated Param Node");
  }

  void set_parameter()
  {
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this);

    // Block until a service is available
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Set several different types of parameters
    auto results = parameters_client->set_parameters(
      {
        rclcpp::Parameter("name", "Robot"),
        rclcpp::Parameter("sim", true),
        rclcpp::Parameter("control_period", 0.030),
        rclcpp::Parameter("control_mode", 3),
      });

    // Wait for the results
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), results) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "set_parameters service call failed. Exiting tutorial.");
      rclcpp::shutdown();
    }

    // Check to see if they were set
    for (auto & result : results.get()) {
      if (!result.successful) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
      }
    }

    // Get a few of the parameters just set
    auto parameters = parameters_client->get_parameters(
      {
        "name",
        "sim",
        "control_period",
        "control_mode"
      });

    // Wait for the results
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), parameters) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed. Exiting tutorial.");
      rclcpp::shutdown();
    }

    // See values just set
    std::stringstream ss;
    for (auto & parameter : parameters.get()) {
      ss << "\nParameter name: " << parameter.get_name();
      ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
        parameter.value_to_string();
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    rclcpp::shutdown();
  }

private:
  std::string name_;
  bool sim_;
  double control_period_;
  uint8_t control_mode_;
};
}  // namespace param_example

RCLCPP_COMPONENTS_REGISTER_NODE(param_example::Param)
