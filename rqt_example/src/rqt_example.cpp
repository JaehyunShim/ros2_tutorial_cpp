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

#include <pluginlib/class_list_macros.h>

// #include <QtGui>
// #include <QMessageBox>
#include <QStringList>

#include "rqt_example/rqt_example.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>

namespace rqt_example
{
RqtExample::RqtExample()
: rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RQT Example");
}

void RqtExample::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // Access standalone command line arguments
  QStringList argv = context.argv();
  // Create QWidget
  widget_ = new QWidget();
  // Extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // Add widget to the user interface
  context.addWidget(widget_);

  // ui_.view_logging->setModel(qnode.loggingModel());
  // connect(&qnode, SIGNAL(logging_updated()), this, SLOT(ui_.view_logging->scrollToBottom()));

  connect(ui_.pub_on_button, SIGNAL(clicked(bool)), this, SLOT(on_pub_on_button_clicked()));
  connect(ui_.pub_off_button, SIGNAL(clicked(bool)), this, SLOT(on_pub_off_button_clicked()));
  connect(ui_.sub_on_button, SIGNAL(clicked(bool)), this, SLOT(on_sub_on_button_clicked()));
  connect(ui_.sub_off_button, SIGNAL(clicked(bool)), this, SLOT(on_sub_off_button_clicked()));

  // Run a thread for ros_spin
  std::thread t(run_ros_thread);
  t.detach();
}

void RqtExample::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void RqtExample::saveSettings(
  qt_gui_cpp::Settings & plugin_settings,
  qt_gui_cpp::Settings & instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void RqtExample::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings,
  const qt_gui_cpp::Settings & instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

// TODO: Find a better way to run ros_spin
auto qnode_ = std::make_shared<rqt_example::QNode>();
void RqtExample::run_ros_thread()
{
  rclcpp::spin(qnode_);
}

void RqtExample::on_pub_on_button_clicked()
{
  qnode_->pub_onoff_ = true;
}

void RqtExample::on_pub_off_button_clicked()
{
  qnode_->pub_onoff_ = false;
}
void RqtExample::on_sub_on_button_clicked()
{
  qnode_->sub_onoff_ = true;
}

void RqtExample::on_sub_off_button_clicked()
{
  qnode_->sub_onoff_ = false;
}
}  // namespace rqt_example

PLUGINLIB_EXPORT_CLASS(rqt_example::RqtExample, rqt_gui_cpp::Plugin)
