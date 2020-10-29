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

#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rqt_example/rqt_node.hpp"
#include <iostream>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace rqt_example
{
QNode::QNode()
: Node("rqt_example")
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // ROS Publisher & Subscriber
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);
  chatter_sub_ =
    this->create_subscription<std_msgs::msg::String>(
    "chatter", qos,
    std::bind(&QNode::chatter_callback, this, std::placeholders::_1));

  // ROS Timer
  timer_ = this->create_wall_timer(
    1s,
    std::bind(&QNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Initialized rqt example node");
}

QNode::~QNode()
{
  RCLCPP_INFO(this->get_logger(), "Terminated rqt example node");
}

void QNode::timer_callback()
{
  int count = 0;

  if (pub_onoff_ == true) {
    std::stringstream ss;
    ss << "hello world " << count;
    // msg_->data = ss.str();
    // msg_->data = "hello world";
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! ";
    chatter_pub_->publish(message);
    printf("%s \n", message.data.c_str());
  //   // log(Info, std::string("I sent: ") + msg.data);
    ++count;
  }
}

void QNode::chatter_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (sub_onoff_ == true) {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }
}

// void QNode::log(const LogLevel & level, const std::string & msg)
// {
//   logging_model.insertRows(logging_model.rowCount(), 1);
//   std::stringstream logging_model_msg;
//   switch (level) {
//     case (Debug): {
//         RCLCPP_DEBUG_STREAM(node_->get_logger(), msg);
//         logging_model_msg << "[DEBUG]: " << msg;
//         break;
//       }
//     case (Info): {
//         RCLCPP_INFO_STREAM(node_->get_logger(), msg);
//         logging_model_msg << "[INFO]: " << msg;
//         break;
//       }
//     case (Warn): {
//         RCLCPP_WARN_STREAM(node_->get_logger(), msg);
//         logging_model_msg << "[WARN]: " << msg;
//         break;
//       }
//     case (Error): {
//         RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
//         logging_model_msg << "[ERROR]: " << msg;
//         break;
//       }
//     case (Fatal): {
//         RCLCPP_FATAL_STREAM(node_->get_logger(), msg);
//         logging_model_msg << "[FATAL]: " << msg;
//         break;
//       }
//   }
//   QVariant new_row(QString(logging_model_msg.str().c_str()));
//   logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
//   Q_EMIT logging_updated();
// }
}  // namespace rqt_example
