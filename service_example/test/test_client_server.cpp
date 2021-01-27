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

#include <gtest/gtest.h>
#include <memory>

#include "service_example/client.hpp"
#include "service_example/server.hpp"

TEST(ClientServerTest, TransferService)
{
  // Create node pointers
  rclcpp::NodeOptions node_options;

  // Server has to be instanticated first as client will wait for server to start up
  auto server = std::make_shared<service_example::Server>(node_options);
  auto client = std::make_shared<service_example::Client>(node_options);
  EXPECT_FALSE(client->msg_requested);
  EXPECT_FALSE(server->msg_responded);

  // Spin for five seconds and check if messages were transferred
  for (int i = 0; i < 5; i++) {
    rclcpp::Rate loop_rate(1);
    rclcpp::spin_some(client);
    rclcpp::spin_some(server);
    loop_rate.sleep();
  }
  EXPECT_TRUE(client->msg_requested);
  EXPECT_TRUE(server->msg_responded);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
