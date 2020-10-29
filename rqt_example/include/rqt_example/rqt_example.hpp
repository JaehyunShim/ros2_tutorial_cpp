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

// #include "rqt_example/qnode.hpp"

namespace rqt_example
{
class RqtExample
  : public rqt_gui_cpp::Plugin
  // : public QMainWindow
{
  Q_OBJECT

public:
  // RqtExample(int argc, char ** argv, QWidget * parent = 0);
  RqtExample();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

public Q_SLOTS:
  void on_pub_on_button_clicked();
  void on_pub_off_button_clicked();
  void on_sub_on_button_clicked();
  void on_sub_off_button_clicked();

private:
  Ui::RqtExampleWidget ui_;
  QWidget * widget_;
// QNode qnode;
};
}  // namespace rqt_example
#endif  // RQT_EXAMPLE__RQT_EXAMPLE_HPP_
