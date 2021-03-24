// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <signal.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "intra_process_tutorial_cpp/publisher.hpp"
#include "intra_process_tutorial_cpp/subscriber.hpp"

int main(int argc, char ** argv)
{
  // init
  rclcpp::init(argc, argv);

  // ros nodes
  auto pub_node = std::make_shared<intra_process_tutorial_cpp::Publisher>("publisher");
  auto sub_node = std::make_shared<intra_process_tutorial_cpp::Subscriber>("subscriber");

  // ros executors
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
