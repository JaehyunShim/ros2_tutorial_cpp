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
{}

QNode::~QNode()
{
  rclcpp::shutdown();    // explicitly needed since we use ros::start();
  wait();
}

bool QNode::init()
{
  rclcpp::init(init_argc, init_argv);
  auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  chatter_pub_ = node->create_publisher<std_msgs::msg::String>("chatter", qos);

  start();

  return true;
}

void QNode::run()
{
  rclcpp::Rate loop_rate(1);
  auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");
  int count = 0;
  while (rclcpp::ok()) {
    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    chatter_pub_->publish(msg);
    log(Info, std::string("I sent: ") + msg.data);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log(const LogLevel & level, const std::string & msg)
{
  auto node = std::make_shared<rclcpp::Node>("my_node", "my_ns");

  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
        RCLCPP_DEBUG_STREAM(node->get_logger(), msg);
        // logging_model_msg << "[DEBUG] [" << std::string(rclcpp::Clock().now()) << "]: " << msg;
        break;
      }
    case (Info): {
        RCLCPP_INFO_STREAM(node->get_logger(), msg);
        // logging_model_msg << "[INFO] [" << std::string(rclcpp::Clock().now()) << "]: " << msg;
        break;
      }
    case (Warn): {
        RCLCPP_WARN_STREAM(node->get_logger(), msg);
        // logging_model_msg << "[INFO] [" << std::string(rclcpp::Clock().now()) << "]: " << msg;
        break;
      }
    case (Error): {
        RCLCPP_ERROR_STREAM(node->get_logger(), msg);
        // logging_model_msg << "[ERROR] [" << std::string(rclcpp::Clock().now()) << "]: " << msg;
        break;
      }
    case (Fatal): {
        RCLCPP_FATAL_STREAM(node->get_logger(), msg);
        // logging_model_msg << "[FATAL] [" << std::string(rclcpp::Clock().now()) << "]: " << msg;
        break;
      }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}
}  // namespace rqt_example
