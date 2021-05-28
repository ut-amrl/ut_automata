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

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>

#include "gui_mainwindow.h"

#include <string>
#include <vector>

#include <QApplication>
#include <QDesktopWidget>
#include <QPushButton>
#include <QBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QWidget>
#include <QGroupBox>
#include <QTabWidget>

#include <ros/master.h>
#include <ros/package.h>

#include "std_msgs/String.h"

#include "vector_display.h"

using std::string;
using std::vector;
using vector_display::VectorDisplay;

namespace {

ut_automata_gui::StatusLed* ros_led_ = nullptr;
ut_automata_gui::StatusLed* drive_led_ = nullptr;
ut_automata_gui::StatusLed* camera_led_ = nullptr;
ut_automata_gui::StatusLed* lidar_led_ = nullptr;
ut_automata_gui::RealStatus* throttle_status_ = nullptr;
ut_automata_gui::RealStatus* steering_status_ = nullptr;

vector<string> GetIPAddresses(bool ignore_lo) {
  static const bool kGetIPV6 = false;
  vector<string> ips;
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;

  getifaddrs(&ifAddrStruct);

  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) continue;
    if (ignore_lo && string(ifa->ifa_name) == string("lo")) continue;
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

}  // namespace

namespace ut_automata_gui {

StatusLed::StatusLed(QString name) : led_(nullptr) {
  QFont font("Arial");
  font.setPointSize(15);
  QLabel* label = new QLabel(name);
  label->setFont(font);
  led_ = new Led();
  led_->setFixedSize(30, 30);
  QHBoxLayout* layout = new QHBoxLayout();
  layout->addWidget(label);
  layout->addWidget(led_);
  setLayout(layout);
}

void StatusLed::SetStatus(bool value) {
  led_->SetStatus(value);
}

RealStatus::RealStatus(bool horizontal) : horizontal_(horizontal), value_(0) {
  if (horizontal) {
    setFixedHeight(48);
  } else {
    setFixedWidth(48);
  }
  setFrameStyle(QFrame::Box | QFrame::Plain);
  setLineWidth(2);
  setMidLineWidth(1);
}


void RealStatus::paintEvent(QPaintEvent *event) {
  static const QBrush kWhiteBrush = QBrush(QColor(255, 255, 255));
  static const QBrush kGreenBrush = QBrush(QColor(0, 225, 0));
  static const QBrush kRedBrush = QBrush(QColor(255, 0, 0));
  QPainter painter;
  painter.begin(this);
  painter.fillRect(QRectF(0, 0, width(), height()), kWhiteBrush);
  static QPen black_pen_(Qt::black);
  black_pen_.setWidth(4);
  painter.setPen(black_pen_);
  if (horizontal_) {
    if (value_ > 0.0) {
      painter.fillRect(QRectF(
          width() / 2.0,
          0,
          value_ * 0.5 * width(),
          height()), 
          kGreenBrush);
    } else {
      painter.fillRect(QRectF(
          width() / 2.0 + value_ * 0.5 * width(),
          0,
          -value_ * 0.5 * width(),
          height()),
          kRedBrush);
    }
    painter.drawLine(width() / 2, 0, width() / 2, height());
  } else {
    if (value_ > 0.0) {
      painter.fillRect(QRectF(
          0, 
          height() / 2 - value_ * 0.5 * height(),
          width(), 
          value_ * 0.5 * height()),
          kGreenBrush);
    } else {
      painter.fillRect(QRectF(
          0,
          height() / 2, 
          width(),
          -value_ * 0.5 * height()), 
          kRedBrush);
    }
    painter.drawLine(0, height() / 2, width(), height() / 2);
  }
  painter.drawLine(0, 0, 0, height());
  painter.drawLine(width(), height(), 0, height());
  painter.drawLine(width(), height(), width(), 0);
  painter.drawLine(0, 0, width(), 0);
  painter.end();
}

MainWindow::MainWindow(QWidget* parent) :
    ipaddr_label_(nullptr),
    tab_widget_(nullptr),
    robot_label_(nullptr),
    main_layout_(nullptr),
    display_(nullptr),
    status_label_(nullptr) {
  this->setWindowTitle("UT AUTOmataGUI");
  robot_label_ = new QLabel("UT AUTOmata");
  QFont font("Arial");
  font.setPointSize(50);
  robot_label_->setFont(font);
  robot_label_->setAlignment(Qt::AlignCenter);

  QPushButton* close_button = new QPushButton("Close");
  close_button->setFocusPolicy(Qt::NoFocus);
  close_button->setFixedHeight(60);
  QHBoxLayout* top_bar = new QHBoxLayout();
  font.setPointSize(20);
  ipaddr_label_ = new QLabel();
  ipaddr_label_->setWordWrap(true);
  status_label_ = new QLabel("Mode: Autonomous\nBattery: 0V");
  status_label_->setFont(font);
  status_label_->setAlignment(Qt::AlignHCenter);
  top_bar->addWidget(ipaddr_label_);
  top_bar->addStretch();
  top_bar->addWidget(status_label_);
  top_bar->addStretch();
  top_bar->addWidget(close_button);

  tab_widget_ = new QTabWidget();
  font.setPointSize(20);
  tab_widget_->setFont(font);
  {
    QWidget* ros_group = new QWidget();
    QSizePolicy expanding_policy;
    expanding_policy.setVerticalPolicy(QSizePolicy::Expanding);
    expanding_policy.setHorizontalPolicy(QSizePolicy::Expanding);
    QPushButton* start_ros = new QPushButton("Start roscore");
    QPushButton* start_camera = new QPushButton("Start Camera");
    QPushButton* start_car = new QPushButton("Start Car");
    QPushButton* stop_all = new QPushButton("Stop all nodes");
    start_ros->setFont(font);
    start_camera->setFont(font);
    start_car->setFont(font);
    stop_all->setFont(font);
    start_ros->setSizePolicy(expanding_policy);
    start_camera->setSizePolicy(expanding_policy);
    start_car->setSizePolicy(expanding_policy);
    stop_all->setSizePolicy(expanding_policy);
    connect(start_car, SIGNAL(clicked()), this, SLOT(StartCar()));
    connect(start_ros, SIGNAL(clicked()), this, SLOT(StartRos()));
    connect(start_camera, SIGNAL(clicked()), this, SLOT(StartCamera()));
    connect(stop_all, SIGNAL(clicked()), this, SLOT(StopAll()));
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(start_ros);
    vbox->addWidget(start_camera);
    vbox->addWidget(start_car);
    vbox->addWidget(stop_all);
    ros_group->setLayout(vbox);
    
    // Grid layout
    QWidget* main_widget = new QWidget();
    ros_led_ = new StatusLed("ROS");
    drive_led_ = new StatusLed("Drive");
    lidar_led_ = new StatusLed("LIDAR");
    camera_led_ = new StatusLed("Camera");
    throttle_status_ = new RealStatus(false);
    steering_status_ = new RealStatus(true);

    QGridLayout* main_layout = new QGridLayout();
    main_layout->addWidget(robot_label_, 0, 0, 4, 4);
    main_layout->addWidget(ros_led_, 0, 5, 1, 1);
    main_layout->addWidget(drive_led_, 0, 6, 1, 1);
    main_layout->addWidget(lidar_led_, 1, 5, 1, 1);
    main_layout->addWidget(camera_led_, 1, 6, 1, 1);
    main_layout->addWidget(throttle_status_, 0, 7, 3, 1);
    main_layout->addWidget(steering_status_, 3, 5, 1, 3);
    main_widget->setLayout(main_layout);

    tab_widget_->addTab(main_widget, "Main");
    tab_widget_->addTab(ros_group, tr("Startup / Shutdown"));
  }

  main_layout_ = new QVBoxLayout(this);
  setLayout(main_layout_);
  main_layout_->addLayout(top_bar, Qt::AlignTop);
  main_layout_->addWidget(tab_widget_);

  connect(close_button, SIGNAL(clicked()), this, SLOT(closeWindow()));

  QTimer* ip_update_timer = new QTimer(this);
  connect(ip_update_timer, SIGNAL(timeout()), this, SLOT(UpdateIP()));
  ip_update_timer->start(1000);
  UpdateIP();

  connect(this,
          SIGNAL(UpdateStatusSignal(int, float, bool, bool, bool, float, float)),
          SLOT(UpdateStatusSlot(int, float, bool, bool, bool, float, float)));
}

std::vector<std::string> Split(const std::string& s) {
  std::vector<std::string> words;
  std::string str;
  for (const char c : s) {
    if (c == ' ' && !str.empty()) {
      words.push_back(str);
      str = "";
    } else {
      str += c;
    }
  }
  if (!str.empty()) words.push_back(str);
  return words;
}

void Exec(const string& cmd) {
  auto params = Split(cmd);
  if (params.empty()) return;
  vector<const char*> param_ptrs;
  for (string& s : params) {
    param_ptrs.push_back(s.c_str());
    printf("'%s',", s.c_str());
  }
  param_ptrs.push_back(NULL);
  printf("\n");
  const int pid = fork();
  if (pid == 0) {
    if (execv(param_ptrs[0], 
              const_cast<char* const*>( param_ptrs.data())) == -1) {
      perror("Error executing command");
      exit(1);
    }
    fprintf(stderr, "ERROR: Reached unreachable statement\n");
  }
}

void MainWindow::StartCar() {
  Exec("roslaunch ut_automata start_car.launch start_gui:=flase");
}

void MainWindow::StartRos() {
  Exec("/usr/bin/screen -mdS roscore roscore");
}

void MainWindow::StartCamera() {
  Exec("roslaunch astra_camera astra.launch > /dev/null &");
}

void MainWindow::StopAll() {
  // two seperate kills to ensure that both
  // autostart and manual start will be stopped
  Exec("/opt/ros/melodic/bin/rosnode kill -a");
  Exec("/usr/bin/pkill roslaunch");
}

void MainWindow::closeWindow() {
  close();
}

void MainWindow::UpdateIP() {
  const vector<string> ips = GetIPAddresses(true);
  string s;
  for (const string& ip : ips) {
    s = s + ip + "\n";
  }
  ipaddr_label_->setText(QString::fromUtf8(s.c_str()));
  ros_led_->SetStatus(ros::master::check());
}

void MainWindow::UpdateStatus(int mode, 
                              float battery,
                              bool drive_okay,
                              bool lidar_okay,
                              bool camera_okay,
                              float throttle,
                              float steering) {
  UpdateStatusSignal(
      mode, battery, drive_okay, lidar_okay, camera_okay, throttle, steering);
}

void MainWindow::UpdateStatusSlot(int mode,
                                  float battery,
                                  bool vesc_okay,
                                  bool lidar_okay,
                                  bool camera_okay,
                                  float throttle,
                                  float steering) {
  QString status("Status: ");
  switch (mode) {
    case 0: {
      status += "Stopped\n";
    } break;
    case 1: {
      status += "Joystick\n";
    } break;
    case 2: {
      status += "Autonomous\n";
    } break;
    case 3: {
      status += "Autonomous\n";
    } break;
    default: {
      status += "UNKNOWN\n";
    } break;
  }
  status += "Battery: " + QString::number(battery, 'g', 4) + "V";
  status_label_->setText(status);
  drive_led_->SetStatus(vesc_okay);
  lidar_led_->SetStatus(lidar_okay);
  camera_led_->SetStatus(camera_okay);
  throttle_status_->SetValue(throttle);
  steering_status_->SetValue(-steering);
  // drive_led_->update();
  // lidar_led_->update();
  // camera_led_->update();
}



}  // namespace ut_automata_gui
