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

#ifndef RQT_EXAMPLE__RQT_EXAMPLE_HPP_
#define RQT_EXAMPLE__RQT_EXAMPLE_HPP_

#include <rqt_gui_cpp/plugin.h>
#include <ui_rqt_example.h>
#include <QWidget>
#include <QTimer>
#include <memory>

#include "rqt_example/rqt_node.hpp"

namespace rqt_example
{
class RqtExample
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RqtExample();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

private:
  Ui::RqtExampleWidget ui_;
  QWidget * widget_;
  std::shared_ptr<rqt_example::RqtNode> rqt_node_;
  QTimer * ros_timer_;
  QTimer * display_timer_;

  QString get_pub_onff();
  QString get_sub_onff();

private slots:
  void ros_timer_callback();
  void display_timer_callback();

  void set_pub_on();
  void set_pub_off();
  void set_sub_on();
  void set_sub_off();
};
}  // namespace rqt_example
#endif  // RQT_EXAMPLE__RQT_EXAMPLE_HPP_
