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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rqt_example/main_window.hpp"

namespace rqt_example
{
namespace Qt
{
MainWindow::MainWindow(int argc, char ** argv, QWidget * parent)
: QMainWindow(parent), qnode(argc, argv)
{
  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  ui.setupUi(this);
  // qApp is a global variable for the application
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  // ensure the first tab is showing - qt-designer should have this already hardwired
  // but often loses it (settings?).
  ui.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  if (ui.checkbox_remember_settings->isChecked()) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check)
{
  if (ui.checkbox_use_environment->isChecked()) {
    if (!qnode.init()) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if (!qnode.init(
        ui.line_edit_master->text().toStdString(),
        ui.line_edit_host->text().toStdString()))
    {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
    }
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
  bool enabled;
  if (state == 0) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
  // ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::updateLoggingView()
{
  ui.view_logging->scrollToBottom();
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(
    this,
    tr("About ..."),
    tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>This package needs an about description.</p>"));
}

void MainWindow::ReadSettings()
{
  QSettings settings("Qt-Ros Package", "rqt_example");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value(
    "master_url",
    QString("http://192.168.1.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  // QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  // ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
    // ui.line_edit_topic->setEnabled(false);
  }
}

void MainWindow::WriteSettings()
{
  QSettings settings("Qt-Ros Package", "rqt_example");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  // settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue(
    "use_environment_variables",
    QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings", QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}
}  // namespace Qt
}  // namespace rqt_example
