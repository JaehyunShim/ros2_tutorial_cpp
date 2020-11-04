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

#ifndef LAUNCH_EXAMPLE__LAUNCH_HPP_
#define LAUNCH_EXAMPLE__LAUNCH_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace launch_example
{
/**
 * @class Launch
 * @brief Publishes "chatter"
 */
class Launch : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  Launch();

  /**
   * @brief Virtual destructor
   */
  virtual ~Launch();

private:
  std::unique_ptr<std_msgs::msg::Int64> msg_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
};
}  // namespace launch_example

#endif  // LAUNCH_EXAMPLE__LAUNCH_HPP_
