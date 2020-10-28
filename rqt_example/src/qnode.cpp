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

#include "rqt_example/qnode.hpp"

namespace rqt_example
{
QNode::QNode(int argc, char ** argv)
: init_argc(argc),
  init_argv(argv)
{
  init();
}

QNode::~QNode()
{
  rclcpp::shutdown();
}

bool QNode::init()
{
  rclcpp::init(init_argc, init_argv);
  node_ = std::make_shared<rclcpp::Node>("my_node");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  chatter_pub_ = node_->create_publisher<std_msgs::msg::String>("chatter", qos);

  start();

  return true;
}

void QNode::run()
{
  rclcpp::Rate loop_rate(1);
  int count = 0;
  while (rclcpp::ok()) {
    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    chatter_pub_->publish(msg);
    log(Info, std::string("I sent: ") + msg.data);
    rclcpp::spin_some(node_);
    loop_rate.sleep();
    ++count;
  }
  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  this->~QNode();
}

void QNode::log(const LogLevel & level, const std::string & msg)
{
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), msg);
        logging_model_msg << "[DEBUG]: " << msg;
        break;
      }
    case (Info): {
        RCLCPP_INFO_STREAM(node_->get_logger(), msg);
        logging_model_msg << "[INFO]: " << msg;
        break;
      }
    case (Warn): {
        RCLCPP_WARN_STREAM(node_->get_logger(), msg);
        logging_model_msg << "[WARN]: " << msg;
        break;
      }
    case (Error): {
        RCLCPP_ERROR_STREAM(node_->get_logger(), msg);
        logging_model_msg << "[ERROR]: " << msg;
        break;
      }
    case (Fatal): {
        RCLCPP_FATAL_STREAM(node_->get_logger(), msg);
        logging_model_msg << "[FATAL]: " << msg;
        break;
      }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
  Q_EMIT logging_updated();
}
}  // namespace rqt_example
