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
\file    gui_main.cc
\brief   GUI for F1/10 car.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <string>
#include <vector>
#include <signal.h>
#include <QApplication>
#include <QPushButton>
#include <QMetaType>

#include "ros/ros.h"

#include "gui_mainwindow.h"

void SignalHandler(int num) {
  if (num == SIGINT) {
    exit(0);
  } else {
    exit(1);
  }
}

int main(int argc, char *argv[]) {
  signal(SIGINT, &SignalHandler);
  qRegisterMetaType<std::vector<std::string> >("std::vector<std::string>");

  ros::init(argc, argv, "f1tenth_gui", ros::init_options::NoSigintHandler);
  QApplication app(argc, argv);
  // ros::NodeHandle node_handle;
  f1tenth_gui::MainWindow main_window;
  main_window.showFullScreen();
  return app.exec();
}
