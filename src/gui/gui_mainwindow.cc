//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gui_mainwindow.cc
\brief   GUI for F1/10 car.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gui_mainwindow.h"

#include <string>
#include <vector>

#include <QPushButton>
#include <QBoxLayout>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QWidget>

#include "ros/ros.h"
#include "std_msgs/String.h"

using std::string;

namespace f1tenth_gui {

MainWindow::MainWindow(ros::NodeHandle* node_handle, QWidget* parent) :
    state_(DisplayState::kAdmin),
    time_label_(nullptr),
    vector_display_(nullptr),
    robot_label_(nullptr),
    main_layout_(nullptr) {
  this->setWindowTitle("F1/10 GUI");
  robot_label_ = new QLabel("F1/10");
  QFont font("Arial");
  font.setPointSize(60);
  robot_label_->setFont(font);
  robot_label_->setAlignment(Qt::AlignCenter);

  QPushButton* close_button = new QPushButton("Close");
  close_button->setFocusPolicy(Qt::NoFocus);
  QHBoxLayout* top_bar = new QHBoxLayout();
  time_label_ = new QLabel("00:00 AM");
  QLabel* status_label = new QLabel("Mode: Autonomous");
  top_bar->addWidget(status_label);
  top_bar->addStretch();
  top_bar->addWidget(time_label_);
  top_bar->addStretch();
  top_bar->addWidget(close_button);

  vector_display_ = new QWidget();

  main_layout_ = new QVBoxLayout(this);
  setLayout(main_layout_);
  main_layout_->addLayout(top_bar, Qt::AlignTop);
  main_layout_->addWidget(vector_display_);
  main_layout_->addWidget(robot_label_, Qt::AlignCenter);
  robot_label_->hide();

  connect(close_button, SIGNAL(clicked()), this, SLOT(closeWindow()));
  connect(this,
          SIGNAL(UpdateSignal()),
          this,
          SLOT(UpdateDisplay()));

  QTimer* time_update_timer = new QTimer(this);
  connect(time_update_timer, SIGNAL(timeout()), this, SLOT(UpdateTime()));
  time_update_timer->start(1000);
  UpdateTime();
}

void MainWindow::passwordEntered(QString pw) {
  printf("Password: %s\n", pw.toStdString().c_str());
  if (pw == "7048") {
    state_ = DisplayState::kAdmin;
  } else {
    state_ = DisplayState::kDefault;
  }
  UpdateSignal();
}

void MainWindow::closeWindow() {
  close();
}

void MainWindow::UpdateTime() {
  time_label_->setText(QTime::currentTime().toString("hh:mm AP"));
}


void MainWindow::UpdateDisplay() {
  robot_label_->hide();
  vector_display_->hide();
  switch(state_) {
    case DisplayState::kDefault: {
      robot_label_->show();
    } break;
    case DisplayState::kAdminPassword: {
    } break;
    case DisplayState::kAdmin: {
      vector_display_->show();
    } break;
    case DisplayState::kHumanInteraction: {
    } break;
  }
}


}  // namespace f1tenth_gui
