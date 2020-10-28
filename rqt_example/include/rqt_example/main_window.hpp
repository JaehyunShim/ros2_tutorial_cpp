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

#ifndef RQT_EXAMPLE__MAIN_WINDOW_HPP_
#define RQT_EXAMPLE__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QWidget>
#include <ui_main_window.h>
#include <rqt_gui_cpp/plugin.h>

#include "rqt_example/qnode.hpp"

namespace rqt_example
{
class MainWindow
  // : public rqt_gui_cpp::Plugin,
  : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(int argc, char ** argv, QWidget * parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void update_logging_view();
  void on_action_about_triggered();

  void on_checkbox_use_environment_stateChanged(int state);

private:
  Ui::MainWindowDesign ui_;
  QNode qnode;
};
}  // namespace rqt_example
#endif  // RQT_EXAMPLE__MAIN_WINDOW_HPP_
