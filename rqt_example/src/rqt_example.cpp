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
// #include <iostream>

#include "rqt_example/rqt_example.hpp"

namespace rqt_example
{
RqtExample::RqtExample()
: rqt_gui_cpp::Plugin(),
  widget_(0)
// qnode(argc, argv)
{
  setObjectName("RQT Example");
}

void RqtExample::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // connect(ui_.action_about_Qt, SIGNAL(triggered(bool)), this, SLOT(aboutQt()));

  // ui_.view_logging->setModel(qnode.loggingModel());
  // connect(&qnode, SIGNAL(logging_updated()), this, SLOT(ui_.view_logging->scrollToBottom()));
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

void RqtExample::on_pub_on_button_clicked()
{
  // qnode.pub_onoff = true;
}

void RqtExample::on_pub_off_button_clicked()
{
  // qnode.pub_onoff = false;
}
void RqtExample::on_sub_on_button_clicked()
{
  // qnode.sub_onoff = true;
}

void RqtExample::on_sub_off_button_clicked()
{
  // qnode.sub_onoff = false;
}
}  // namespace rqt_example

PLUGINLIB_EXPORT_CLASS(rqt_example::RqtExample, rqt_gui_cpp::Plugin)
