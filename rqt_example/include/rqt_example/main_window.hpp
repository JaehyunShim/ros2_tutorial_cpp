// Copyright 2020 ROBOTIS CO., LTD.
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
//
// Author: Jaehyun Shim

#ifndef RQT_EXAMPLE__MAIN_WINDOW_HPP_
#define RQT_EXAMPLE__MAIN_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "qnode.hpp"

#include "rqt_example/ui_main_window.h"

namespace rqt_example {

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();  // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check );
  void on_checkbox_use_environment_stateChanged(int state);

  void updateLoggingView();  // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace rqt_example
#endif  // RQT_EXAMPLE__MAIN_WINDOW_HPP_
