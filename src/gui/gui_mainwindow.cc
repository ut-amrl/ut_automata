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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#include "gui_mainwindow.h"

#include <string>
#include <vector>

#include <QPushButton>
#include <QBoxLayout>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QWidget>
#include <QGroupBox>
#include <QTabWidget>

#include "std_msgs/String.h"

#include "vector_display.h"

using std::string;
using std::vector;
using vector_display::VectorDisplay;

vector<string> GetIPAddresses() {
  static const bool kGetIPV6 = false;
  vector<string> ips;
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;

  getifaddrs(&ifAddrStruct);

  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
        continue;
    }
    if (ifa->ifa_addr->sa_family == AF_INET) {
      // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      ips.push_back(
          string(ifa->ifa_name) + " : " + string(addressBuffer));
    } else if (kGetIPV6 && ifa->ifa_addr->sa_family == AF_INET6) {
      // is a valid IP6 Address
      tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
      char addressBuffer[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
      ips.push_back(
          string(ifa->ifa_name) + " : " + string(addressBuffer));
    }
  }
  if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
  return ips;
}


namespace f1tenth_gui {

MainWindow::MainWindow(QWidget* parent) :
    time_label_(nullptr),
    tab_widget_(nullptr),
    robot_label_(nullptr),
    main_layout_(nullptr),
    display_(nullptr) {
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
  time_label_->setWordWrap(true);
  QLabel* status_label = new QLabel("Mode: Autonomous");
  top_bar->addWidget(status_label);
  top_bar->addStretch();
  top_bar->addWidget(time_label_);
  top_bar->addStretch();
  top_bar->addWidget(close_button);

  tab_widget_ = new QTabWidget();
  {
    QWidget* ros_group = new QWidget();
    QSizePolicy expanding_policy;
    expanding_policy.setVerticalPolicy(QSizePolicy::Expanding);
    expanding_policy.setHorizontalPolicy(QSizePolicy::Expanding);
    QPushButton* start_ros = new QPushButton("Start roscore");
    QPushButton* stop_ros = new QPushButton("Stop roscore");
    QPushButton* start_car = new QPushButton("Start Car");
    QPushButton* stop_all = new QPushButton("Stop all nodes");
    font.setPointSize(20);
    start_ros->setFont(font);
    stop_ros->setFont(font);
    start_car->setFont(font);
    stop_all->setFont(font);
    start_ros->setSizePolicy(expanding_policy);
    stop_ros->setSizePolicy(expanding_policy);
    start_car->setSizePolicy(expanding_policy);
    stop_all->setSizePolicy(expanding_policy);
    connect(start_car, SIGNAL(clicked()), this, SLOT(StartCar()));
    connect(start_ros, SIGNAL(clicked()), this, SLOT(StartRos()));
    connect(stop_ros, SIGNAL(clicked()), this, SLOT(StopRos()));
    connect(stop_all, SIGNAL(clicked()), this, SLOT(StopAll()));
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(start_ros);
    vbox->addWidget(stop_ros);
    vbox->addWidget(start_car);
    vbox->addWidget(stop_all);
    ros_group->setLayout(vbox);
    display_ = new VectorDisplay();
    tab_widget_->addTab(robot_label_, "Main");
    tab_widget_->addTab(ros_group, tr("Startup / Shutdown"));
    tab_widget_->addTab(display_, tr("Visualizations"));
  }

  main_layout_ = new QVBoxLayout(this);
  setLayout(main_layout_);
  main_layout_->addLayout(top_bar, Qt::AlignTop);
  main_layout_->addWidget(tab_widget_);

  connect(close_button, SIGNAL(clicked()), this, SLOT(closeWindow()));

  QTimer* time_update_timer = new QTimer(this);
  connect(time_update_timer, SIGNAL(timeout()), this, SLOT(UpdateTime()));
  time_update_timer->start(1000);
  UpdateTime();
}

void Execute(const string& cmd) {
  if (system(cmd.c_str()) != 0) {
    printf("Error running '%s'\n", cmd.c_str());
  } else {
    printf("Ran '%s'\n", cmd.c_str());
  }
}

void MainWindow::StartCar() {
  Execute("screen -mdS start_car "
          "/home/amrl_user/f1tenth_course/start_car.bash");
}

void MainWindow::StartRos() {
  Execute("screen -mdS roscore roscore");
}

void MainWindow::StopRos() {
  Execute("killall roscore");
}

void MainWindow::StopAll() {
  Execute("rosnode kill -a");
}

void MainWindow::closeWindow() {
  close();
}

void MainWindow::UpdateTime() {
  const vector<string> ips = GetIPAddresses();
  string s;
  for (const string& ip : ips) {
    s = s + ip + "\n";
  }
  time_label_->setText(QString::fromUtf8(s.c_str()));
  // time_label_->setText(QTime::currentTime().toString("hh:mm AP"));
}



}  // namespace f1tenth_gui
