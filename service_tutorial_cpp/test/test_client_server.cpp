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

#include "service_tutorial_cpp/client.hpp"
#include "service_tutorial_cpp/server.hpp"

TEST(ClientServerTest, TransferService)
{
  // Create node pointers
  auto client = std::make_shared<service_tutorial_cpp::Client>(1, 2);
  auto server = std::make_shared<service_tutorial_cpp::Server>();

  // // Check if data service is not requested and responeded yet
  EXPECT_FALSE(client->srv_requested_);
  EXPECT_FALSE(server->srv_responded_);

  // Spin for five seconds
  rclcpp::Rate loop_rate(1);
  for (int i = 0; i < 5; i++) {
    rclcpp::spin_some(client);
    rclcpp::spin_some(server);
    loop_rate.sleep();
  }

  // Check if data service has been requested and responded
  EXPECT_TRUE(client->srv_requested_);
  EXPECT_TRUE(server->srv_responded_);

  // Check if data service have been requested and responded correctly
  EXPECT_EQ(client->requested_srv_a_, server->requested_srv_a_);
  EXPECT_EQ(client->requested_srv_b_, server->requested_srv_b_);
  EXPECT_EQ(client->responded_srv_, server->responded_srv_);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
