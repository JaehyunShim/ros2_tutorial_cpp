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

#include "topic_example/publisher.hpp"
#include "topic_example/subscriber.hpp"

TEST(PublisherSubscriberTest, TransferMessage)
{
  // Create node pointers
  rclcpp::NodeOptions node_options;
  auto publisher = std::make_shared<topic_example::Publisher>(node_options);
  auto subscriber = std::make_shared<topic_example::Subscriber>(node_options);
  EXPECT_FALSE(publisher->msg_published);
  EXPECT_FALSE(subscriber->msg_received);

  // Spin for five seconds and check if messages were transferred
  for (int i = 0; i < 5; i++) {
    rclcpp::Rate loop_rate(1);
    rclcpp::spin_some(publisher);
    rclcpp::spin_some(subscriber);
    loop_rate.sleep();
  }
  EXPECT_TRUE(publisher->msg_published);
  EXPECT_TRUE(subscriber->msg_received);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
