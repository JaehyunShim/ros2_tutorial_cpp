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

#ifndef RQT_EXAMPLE__QNODE_HPP_
#define RQT_EXAMPLE__QNODE_HPP_

#ifndef Q_MOC_RUN
  #include "rclcpp/rclcpp.hpp"
#endif

#include <QThread>
#include <QStringListModel>

#include <string>

#include "std_msgs/msg/string.hpp"

namespace rqt_example
{
class QNode : public QThread
{
  Q_OBJECT

public:
  QNode(int argc, char ** argv);
  virtual ~QNode();
  bool init();
  // bool init(const std::string & master_url, const std::string & host_url);
  void run();

  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QStringListModel * loggingModel() {return &logging_model;}
  void log(const LogLevel & level, const std::string & msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char ** init_argv;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
  // rclcpp::Publisher chatter_publisher;
  QStringListModel logging_model;
};

}  // namespace rqt_example
#endif  // RQT_EXAMPLE__QNODE_HPP_
