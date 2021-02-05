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

#include <gtest/gtest.h>

#include <memory>

#include "parameter_tutorial_cpp/parameter.hpp"

TEST(ParameterTest, GetSetParameter)
{
  auto parameter = std::make_shared<parameter_tutorial_cpp::Parameter>();

  // Check if parameters are not updated yet
  EXPECT_FALSE(parameter->parameter_updated_);

  // Spin for five seconds
  rclcpp::Rate loop_rate(1);
  for (int i = 0; i < 5; i++) {
    rclcpp::spin_some(parameter);
    loop_rate.sleep();
  }

  // Check if parameters have been updated
  EXPECT_TRUE(parameter->parameter_updated_);

  // Check if parameters have been updated correctly
  EXPECT_NE(parameter->init_parameter_, parameter->updated_parameter_);
  EXPECT_EQ(parameter->new_parameter_, parameter->updated_parameter_);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
