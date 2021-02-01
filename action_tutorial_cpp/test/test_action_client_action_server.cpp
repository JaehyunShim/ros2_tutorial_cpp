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

#include "action_tutorial_cpp/action_client.hpp"
#include "action_tutorial_cpp/action_server.hpp"

TEST(ActionClientActionServerTest, TransferAction)
{
  // Create node pointers
  auto action_client = std::make_shared<action_tutorial_cpp::ActionClient>();
  auto action_server = std::make_shared<action_tutorial_cpp::ActionServer>();

  // Check if action goal, feedback and result are not sent yet
  EXPECT_FALSE(action_client->goal_sent_);
  EXPECT_FALSE(action_client->feedback_received_);
  EXPECT_FALSE(action_client->result_received_);
  EXPECT_FALSE(action_server->goal_received_);
  EXPECT_FALSE(action_server->action_accepted_);

  // Spin for 20 seconds
  rclcpp::Rate loop_rate(100);  // loop period: 10ms
                                // short loop period to catch feedback
  for (int i = 0; i < 2000; i++) {
    rclcpp::spin_some(action_client);
    rclcpp::spin_some(action_server);
    loop_rate.sleep();
  }

  // Check if action goal, feedback and result has been sent
  EXPECT_TRUE(action_client->goal_sent_);
  EXPECT_TRUE(action_client->feedback_received_);
  EXPECT_TRUE(action_client->result_received_);
  EXPECT_TRUE(action_server->goal_received_);
  EXPECT_TRUE(action_server->action_accepted_);
}

TEST(ActionClientActionServerTest, CancelAction)
{
  // Create node pointers
  auto action_client = std::make_shared<action_tutorial_cpp::ActionClient>();
  auto action_server = std::make_shared<action_tutorial_cpp::ActionServer>();

  // Check if action cancel is not called yet
  EXPECT_FALSE(action_server->action_canceled_);

  // Spin for 10 seconds
  rclcpp::Rate loop_rate(1);
  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(action_client);
    rclcpp::spin_some(action_server);
    loop_rate.sleep();

    // Cancel action when called 5 times
    static int count = 0;
    count++;
    if (count == 5) {
      action_client->get_action_client()->async_cancel_all_goals();
    }
  }

  // Check if action cancel has been called
  EXPECT_TRUE(action_server->action_canceled_);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
