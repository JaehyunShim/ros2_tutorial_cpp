// Copyright 2020 ROBOTIS CO., LTD.
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

#include <gtest/gtest.h>
#include <memory>

#include "action_example/action_example.hpp"

TEST(TestROS2TutorialCPP, test_action_example)
{
  rclcpp::init(0, nullptr);

  // TODO(Ryan): Study how to write more general gtest codes!!!
  // Also below hasn't been debugged yet
  // <<< error message
  // The test did not generate a result file.
  // >>>

  EXPECT_ANY_THROW(
    auto node = std::make_shared<action_example::ROS2TutorialCPP>();
  );
}