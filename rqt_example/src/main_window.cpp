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

#include <QtGui>
#include <QMessageBox>
#include <iostream>

#include "rqt_example/main_window.hpp"

namespace rqt_example
{
using namespace Qt;

MainWindow::MainWindow(int argc, char ** argv, QWidget * parent)
: QMainWindow(parent),
  qnode(argc, argv)
{
  ui_.setupUi(this);

  // qApp is a global variable for the application
  QObject::connect(ui_.action_about_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

  ui_.tab_manager->setCurrentIndex(0);

  ui_.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(logging_updated()), this, SLOT(update_logging_view()));

  if (ui_.checkbox_remember_settings->isChecked()) {
    // on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui_.line_edit_master->setEnabled(enabled);
  ui_.line_edit_host->setEnabled(enabled);
  // ui_.line_edit_topic->setEnabled(enabled);
}

void MainWindow::update_logging_view()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::on_action_about_triggered()
{
  QMessageBox::about(
    this,
    tr("About ..."),
    tr("<h2>Test Program 1.0.0</h2><p>This package needs a package description.</p>"));
}
}  // namespace rqt_example
