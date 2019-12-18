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
\file    gui_mainwindow.h
\brief   GUI for F1/10 car.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#ifndef GUI_MAINWINDOW_H
#define GUI_MAINWINDOW_H

#include <string>
#include <vector>
#include <QWidget>

#ifndef Q_MOC_RUN
#include "ros/ros.h"
#endif

class QLabel;
class QVBoxLayout;

namespace f1tenth_gui {

class TouchVectorDisplay;
class AdminPassword;
class HumanInteraction;

class MainWindow : public QWidget {
  Q_OBJECT

public:
  MainWindow(ros::NodeHandle* node_handle, QWidget *parent = 0);
public slots:
  void closeWindow();
  void UpdateTime();

signals:
  void UpdateQuestion(std::string question,
                      std::vector<std::string> responses);
  void UpdateSignal();

private:

  // Used to show the time.
  QLabel* time_label_;

  // Vector display.
  QWidget* vector_display_;

  // Robot name display.
  QLabel* robot_label_;

  // Main layout of the window.
  QVBoxLayout* main_layout_;

  // Question ID.
  int interaction_id_;
};


}  // namespace f1tenth_gui

#endif  // GUI_MAINWINDOW_H
