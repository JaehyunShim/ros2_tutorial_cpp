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

#include "topic_tutorial_cpp/publisher_lambda.hpp"
#include "topic_tutorial_cpp/publisher_member_function.hpp"
#include "topic_tutorial_cpp/subscriber_lambda.hpp"
#include "topic_tutorial_cpp/subscriber_member_function.hpp"

// Any reference for gtest??
TEST(PublisherSubscriberTest, TransferTopic)
{
  // Create node pointers
  auto publisher_member_function =
    std::make_shared<topic_tutorial_cpp::PublisherMemberFunction>();
  auto subscriber_member_function =
    std::make_shared<topic_tutorial_cpp::SubscriberMemberFunction>();
  auto publisher_lambda =
    std::make_shared<topic_tutorial_cpp::PublisherLambda>();
  auto subscriber_lambda =
    std::make_shared<topic_tutorial_cpp::SubscriberLambda>();

  // Check if msgs are not transferred yet
  EXPECT_FALSE(publisher_member_function->msg_published_);
  EXPECT_FALSE(subscriber_member_function->msg_received_);
  EXPECT_FALSE(publisher_lambda->msg_published_);
  EXPECT_FALSE(subscriber_lambda->msg_received_);

  // Spin five times
  rclcpp::Rate loop_rate(1);
  for (int i = 0; i < 5; i++) {
    rclcpp::spin_some(publisher_member_function);
    rclcpp::spin_some(publisher_lambda);
    rclcpp::spin_some(subscriber_member_function);
    rclcpp::spin_some(subscriber_lambda);
    loop_rate.sleep();
  }

  // Check if messages have been transferred
  EXPECT_TRUE(publisher_member_function->msg_published_);
  EXPECT_TRUE(subscriber_member_function->msg_received_);
  EXPECT_TRUE(publisher_lambda->msg_published_);
  EXPECT_TRUE(subscriber_lambda->msg_received_);

  // Check if messages have been transferred correctly
  EXPECT_EQ(publisher_member_function->published_msg_, subscriber_member_function->received_msg_);
  EXPECT_EQ(publisher_lambda->published_msg_, subscriber_lambda->received_msg_);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
