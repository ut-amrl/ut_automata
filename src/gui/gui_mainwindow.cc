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
#include <sys/statvfs.h>

#include "gui_mainwindow.h"

#include <string>
#include <vector>
#include <algorithm>

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
#include <QPixmap>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSizePolicy>
#include <QProcess>
#include <QDir>
#include <QTextStream>
#include <QFile>
#include <QTemporaryFile>
#include <QMessageBox>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

#include "std_msgs/msg/string.hpp"

#include "vector_display.h"

using std::string;
using std::vector;
using vector_display::VectorDisplay;

namespace {

ut_automata_gui::StatusLed* imu_led_ = nullptr;
ut_automata_gui::StatusLed* drive_led_ = nullptr;
ut_automata_gui::StatusLed* joystick_led_ = nullptr;
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

CameraDisplay::CameraDisplay(QWidget* parent) : QLabel(parent) {
  setScaledContents(false);  // We'll handle scaling manually for better control
  setAlignment(Qt::AlignCenter);
  setStyleSheet("border: 2px solid black; background-color: #f0f0f0;");
  setText("Waiting for Camera Feed...");
  setWordWrap(true);
  
  QFont font("Arial");
  font.setPointSize(16);
  setFont(font);
  
  // Allow the widget to expand
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void CameraDisplay::UpdateImage(const QPixmap& pixmap) {
  current_image_ = pixmap;
  
  // Get the current widget size for scaling
  QSize widget_size = size();

  // Scale the image to fit the widget while maintaining aspect ratio
  QPixmap scaled_pixmap = pixmap.scaled(widget_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  
  setPixmap(scaled_pixmap);
}

void CameraDisplay::resizeEvent(QResizeEvent* event) {
  QLabel::resizeEvent(event);
  
  // If we have a current image, rescale it to the new size
  if (!current_image_.isNull()) {
    QPixmap scaled_pixmap = current_image_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    setPixmap(scaled_pixmap);
  }
}

StatusLed::StatusLed(QString name, bool is_recording_led) : led_(nullptr) {
  QFont font("Arial");
  font.setPointSize(15);
  QLabel* label = new QLabel(name);
  label->setFont(font);
  led_ = new Led(is_recording_led);
  led_->setFixedSize(30, 30);
  QHBoxLayout* layout = new QHBoxLayout();
  layout->addWidget(label);
  layout->addWidget(led_);
  setLayout(layout);
}

void StatusLed::SetStatus(bool value) {
  led_->SetStatus(value);
}

void StatusLed::SetRecordingState(Led::RecordingState state) {
  led_->SetRecordingState(state);
}

DiskSpaceBar::DiskSpaceBar(QWidget* parent) 
    : QWidget(parent), used_gb_(0.0f), total_gb_(0.0f) {
  setFixedHeight(24); // 12pt font tall, with some padding
  
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(5, 2, 5, 2);
  
  used_label_ = new QLabel("0 GB");
  available_label_ = new QLabel("0 GB");
  
  QFont font("Arial", 8); // Small font for compact display
  used_label_->setFont(font);
  available_label_->setFont(font);
  
  layout->addWidget(used_label_);
  layout->addStretch();
  layout->addWidget(available_label_);
  
  setLayout(layout);
  
  // Get initial disk space
  struct statvfs stat;
  if (statvfs("/home", &stat) == 0) {
    total_gb_ = (stat.f_blocks * stat.f_frsize) / (1024.0 * 1024.0 * 1024.0);
    used_gb_ = ((stat.f_blocks - stat.f_bavail) * stat.f_frsize) / (1024.0 * 1024.0 * 1024.0);
    float available_gb = (stat.f_bavail * stat.f_frsize) / (1024.0 * 1024.0 * 1024.0);
    
    used_label_->setText(QString::number(used_gb_, 'f', 1) + " GB");
    available_label_->setText(QString::number(available_gb, 'f', 1) + " GB");
  }
}

void DiskSpaceBar::UpdateDiskSpace(float used_gb, float total_gb) {
  used_gb_ = used_gb;
  total_gb_ = total_gb;
  float available_gb = total_gb_ - used_gb_;
  
  used_label_->setText(QString::number(used_gb_, 'f', 1) + " GB");
  available_label_->setText(QString::number(available_gb, 'f', 1) + " GB");
  
  update(); // Trigger repaint
}

void DiskSpaceBar::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  
  // Draw background
  painter.fillRect(rect(), QColor(240, 240, 240));
  
  // Draw border
  painter.setPen(QPen(Qt::black, 1));
  painter.drawRect(rect().adjusted(0, 0, -1, -1));
  
  if (total_gb_ > 0) {
    float usage_ratio = used_gb_ / total_gb_;
    int bar_width = width() - 2; // Account for border
    int used_width = static_cast<int>(bar_width * usage_ratio);
    
    // Choose color based on usage
    QColor bar_color;
    if (usage_ratio < 0.8) {
      bar_color = QColor(0, 200, 0); // Green
    } else if (usage_ratio < 0.9) {
      bar_color = QColor(255, 165, 0); // Orange
    } else {
      bar_color = QColor(255, 0, 0); // Red
    }
    
    // Draw usage bar
    painter.fillRect(1, 1, used_width, height() - 2, bar_color);
  }
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
    camera_display_(nullptr),
    main_layout_(nullptr),
    display_(nullptr),
    status_label_(nullptr),
    disk_space_bar_(nullptr),
    stop_config_button_(nullptr),
    recording_led_(nullptr),
    recorder_widget_(nullptr) {
  this->setWindowTitle("UT AUTOmataGUI");
  
  // Ensure window takes full screen space
  setMinimumSize(800, 480);  // Set a reasonable minimum size
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  
  camera_display_ = new CameraDisplay();
  
  QPushButton* close_button = new QPushButton("Close");
  close_button->setFocusPolicy(Qt::NoFocus);
  close_button->setFixedHeight(60);
  QHBoxLayout* top_bar = new QHBoxLayout();
  QFont font("Arial");
  font.setPointSize(20);
  ipaddr_label_ = new QLabel();
  ipaddr_label_->setWordWrap(true);
  status_label_ = new QLabel("Mode: Autonomous\nBattery: 0V");
  status_label_->setFont(font);
  status_label_->setAlignment(Qt::AlignHCenter);
  
  // Add recording indicator
  recording_led_ = new StatusLed("REC", true);  // true = use grey when off
  recording_led_->setFixedHeight(60);
  recording_led_->setVisible(false);  // Initially hidden until recorder node is detected
  
  top_bar->addWidget(ipaddr_label_);
  top_bar->addStretch();
  top_bar->addWidget(recording_led_);
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
    
    // Create main horizontal layout
    QHBoxLayout* main_config_layout = new QHBoxLayout();
    
    // Left side - Tmux configurations
    QWidget* left_widget = new QWidget();
    QVBoxLayout* left_layout = new QVBoxLayout();
    QLabel* config_label = new QLabel("Available Configurations:");
    config_label->setFont(font);
    left_layout->addWidget(config_label);
    
    // Get tmux configuration names (excluding "sim")
    tmux_config_names_.clear();
    tmux_config_buttons_.clear();
    QDir tmux_dir("/home/orin/roboracer_ws/tmux");
    QStringList subdirs = tmux_dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    
    for (const QString& subdir : subdirs) {
      if (subdir != "sim") {
        tmux_config_names_.push_back(subdir.toStdString());
        QPushButton* config_button = new QPushButton(subdir);
        config_button->setFont(font);
        config_button->setSizePolicy(expanding_policy);
        config_button->setCheckable(true);
        connect(config_button, SIGNAL(clicked()), this, SLOT(StartTmuxConfiguration()));
        tmux_config_buttons_.push_back(config_button);
        left_layout->addWidget(config_button);
      }
    }
    
    left_layout->addStretch();
    left_widget->setLayout(left_layout);
    
    // Right side - Control buttons
    QWidget* right_widget = new QWidget();
    QVBoxLayout* right_layout = new QVBoxLayout();
    
    stop_config_button_ = new QPushButton("Stop Configuration");
    stop_config_button_->setFont(font);
    stop_config_button_->setSizePolicy(expanding_policy);
    stop_config_button_->setEnabled(false);  // Initially disabled
    connect(stop_config_button_, SIGNAL(clicked()), this, SLOT(StopTmuxConfiguration()));
    
    right_layout->addWidget(stop_config_button_);
    right_layout->addStretch();  // Fill remaining space
    right_widget->setLayout(right_layout);
    
    // Add left and right widgets to main layout
    main_config_layout->addWidget(left_widget, 2);  // Give more space to left side
    main_config_layout->addWidget(right_widget, 1);
    
    ros_group->setLayout(main_config_layout);
    
    // Grid layout
    QWidget* main_widget = new QWidget();
    imu_led_ = new StatusLed("IMU");
    drive_led_ = new StatusLed("Drive");
    lidar_led_ = new StatusLed("LIDAR");
    joystick_led_ = new StatusLed("Joystick");
    throttle_status_ = new RealStatus(false);
    steering_status_ = new RealStatus(true);

    QGridLayout* main_layout = new QGridLayout();
    main_layout->setContentsMargins(5, 5, 5, 5);  // Minimal margins
    main_layout->setSpacing(5);  // Minimal spacing
    main_layout->addWidget(camera_display_, 0, 0, 4, 4);
    main_layout->addWidget(imu_led_, 0, 5, 1, 1);
    main_layout->addWidget(drive_led_, 0, 6, 1, 1);
    main_layout->addWidget(lidar_led_, 1, 5, 1, 1);
    main_layout->addWidget(joystick_led_, 1, 6, 1, 1);
    main_layout->addWidget(throttle_status_, 0, 7, 3, 1);
    main_layout->addWidget(steering_status_, 3, 5, 1, 3);
    
    // Set column stretch factors to ensure proper expansion
    main_layout->setColumnStretch(0, 4);  // Camera takes most space
    main_layout->setColumnStretch(1, 4);
    main_layout->setColumnStretch(2, 4);
    main_layout->setColumnStretch(3, 4);
    main_layout->setColumnStretch(4, 0);  // Control columns get minimal space
    main_layout->setColumnStretch(5, 1);
    main_layout->setColumnStretch(6, 1);
    main_layout->setColumnStretch(7, 1);
    
    main_widget->setLayout(main_layout);

    tab_widget_->addTab(main_widget, "Main");
    tab_widget_->addTab(ros_group, tr("Configurations"));
    
    // Create Recorder Tab
    recorder_widget_ = new QWidget();
    QVBoxLayout* recorder_layout = new QVBoxLayout();
    
    QLabel* recorder_title = new QLabel("Recording Topics");
    recorder_title->setFont(font);
    recorder_layout->addWidget(recorder_title);
    
    // Create scroll area for topics
    QWidget* topics_container = new QWidget();
    QVBoxLayout* topics_layout = new QVBoxLayout();
    topics_container->setLayout(topics_layout);
    
    // Initialize default topics
    selected_topics_ = {
      "/scan",
      "/camera_0/image_raw",
      "/ackermann_curvature_drive",
      "/car_status",
      "/joystick",
      "/imu",
      "/odom"
    };
    
    available_topics_ = {
      "/scan",
      "/camera_0/image_raw",
      "/ackermann_curvature_drive",
      "/car_status",
      "/joystick",
      "/imu",
      "/odom",
      "/tf",
      "/tf_static"
    };
    
    topic_buttons_.clear();
    
    for (const auto& topic : available_topics_) {
      QPushButton* topic_btn = new QPushButton(QString::fromStdString(topic));
      topic_btn->setFont(font);
      topic_btn->setCheckable(true);
      
      // Check if topic is in selected list
      bool is_selected = std::find(selected_topics_.begin(), 
                                   selected_topics_.end(), 
                                   topic) != selected_topics_.end();
      topic_btn->setChecked(is_selected);
      
      connect(topic_btn, SIGNAL(clicked()), this, SLOT(ToggleTopicRecording()));
      topic_buttons_.push_back(topic_btn);
      topics_layout->addWidget(topic_btn);
    }
    
    topics_layout->addStretch();
    
    recorder_layout->addWidget(topics_container);
    recorder_widget_->setLayout(recorder_layout);
    
    // Don't add tab initially, wait for recorder node
    // tab_widget_->addTab(recorder_widget_, tr("Recorder"));
  }

  main_layout_ = new QVBoxLayout(this);
  main_layout_->setContentsMargins(5, 5, 5, 5);  // Minimal margins
  main_layout_->setSpacing(5);  // Minimal spacing between elements
  setLayout(main_layout_);
  main_layout_->addLayout(top_bar, 0);  // Top bar gets minimal space
  main_layout_->addWidget(tab_widget_, 1);  // Tab widget gets most space (stretch factor 1)
  
  // Add disk space indicator at the bottom
  disk_space_bar_ = new DiskSpaceBar();
  main_layout_->addWidget(disk_space_bar_, 0);  // Bottom bar gets minimal space

  connect(close_button, SIGNAL(clicked()), this, SLOT(closeWindow()));

  QTimer* ip_update_timer = new QTimer(this);
  connect(ip_update_timer, SIGNAL(timeout()), this, SLOT(UpdateIP()));
  ip_update_timer->start(1000);
  UpdateIP();

  connect(this,
          SIGNAL(UpdateStatusSignal(int, float, bool, bool, bool, bool, float, float)),
          SLOT(UpdateStatusSlot(int, float, bool, bool, bool, bool, float, float)));
  
  connect(this,
          SIGNAL(UpdateCameraSignal(const QPixmap&)),
          SLOT(UpdateCameraSlot(const QPixmap&)));
  
  connect(this,
          SIGNAL(UpdateRecordingStatusSignal(bool)),
          SLOT(UpdateRecordingStatusSlot(bool)));
  
  connect(this,
          SIGNAL(UpdateRecordingStateSignal(Led::RecordingState)),
          SLOT(UpdateRecordingStateSlot(Led::RecordingState)));

  connect(this,
          SIGNAL(UpdateAvailableTopicsSignal(const std::vector<std::string>&)),
          SLOT(UpdateAvailableTopicsSlot(const std::vector<std::string>&)));
}

MainWindow::~MainWindow() {
  // Clean shutdown - ensure Qt widgets are properly cleaned up
  // The main thread should handle ROS cleanup before destroying the GUI
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
  if (cmd.empty()) return;
  printf("Executing: %s\n", cmd.c_str());
  const int pid = fork();
  if (pid == 0) {
    if (execl("/bin/bash", "bash", "-c", cmd.c_str(), NULL) == -1) {
      perror("Error executing command");
      exit(1);
    }
    fprintf(stderr, "ERROR: Reached unreachable statement\n");
  }
}

void MainWindow::StartCar() {
  Exec("cd $HOME/roboracer_ws/tmux/teleop && tmuxinator");
}

void MainWindow::StartRos() {
  Exec("/usr/bin/screen -mdS roscore roscore");
}

void MainWindow::StartCamera() {
  Exec("roslaunch astra_camera astra.launch > /dev/null &");
}

void MainWindow::closeWindow() {
  QApplication::quit();
}

void MainWindow::UpdateIP() {
  const vector<string> ips = GetIPAddresses(true);
  string s;
  for (const string& ip : ips) {
    s = s + ip + "\n";
  }
  ipaddr_label_->setText(QString::fromUtf8(s.c_str()));
  
  // Update disk space
  struct statvfs stat;
  if (statvfs("/home", &stat) == 0) {
    float total_gb = (stat.f_blocks * stat.f_frsize) / (1024.0 * 1024.0 * 1024.0);
    float used_gb = ((stat.f_blocks - stat.f_bavail) * stat.f_frsize) / (1024.0 * 1024.0 * 1024.0);
    disk_space_bar_->UpdateDiskSpace(used_gb, total_gb);
  }
  
  UpdateTmuxConfigurations();
}

void MainWindow::UpdateStatus(int mode, 
                              float battery,
                              bool drive_okay,
                              bool lidar_okay,
                              bool joystick_okay,
                              bool imu_okay,
                              float throttle,
                              float steering) {
  UpdateStatusSignal(
      mode, battery, drive_okay, lidar_okay, joystick_okay, imu_okay, throttle, steering);
}

void MainWindow::UpdateCamera(const QPixmap& image) {
  UpdateCameraSignal(image);
}

void MainWindow::UpdateStatusSlot(int mode,
                                  float battery,
                                  bool vesc_okay,
                                  bool lidar_okay,
                                  bool joystick_okay,
                                  bool imu_okay,
                                  float throttle,
                                  float steering) {
  QString status("Status: ");
  switch (mode) {
    case 0: {
      status += "Stop\n";
    } break;
    case 1: {
      status += "Joy \n";
    } break;
    case 2: {
      status += "Auto\n";
    } break;
    case 3: {
      status += "Auto\n";
    } break;
    default: {
      status += "UNKN\n";
    } break;
  }
  status += "Battery: " + QString::number(battery, 'f', 1) + "V";
  status_label_->setText(status);
  drive_led_->SetStatus(vesc_okay);
  lidar_led_->SetStatus(lidar_okay);
  joystick_led_->SetStatus(joystick_okay);
  imu_led_->SetStatus(imu_okay);
  throttle_status_->SetValue(throttle);
  steering_status_->SetValue(-steering);
  // drive_led_->update();
  // lidar_led_->update();
  // joystick_led_->update();
}

void MainWindow::UpdateCameraSlot(const QPixmap& image) {
  camera_display_->UpdateImage(image);
}

void MainWindow::UpdateTmuxConfigurations() {
  // Get list of running tmux sessions
  QProcess process;
  process.start("tmux", QStringList() << "ls");
  process.waitForFinished(1000);
  
  std::vector<std::string> running_sessions;
  bool any_session_running = false;
  
  if (process.exitCode() == 0) {
    QString output = process.readAllStandardOutput();
    QStringList sessions = output.split('\n', Qt::SkipEmptyParts);
    
    for (const QString& session : sessions) {
      // Session name is everything before the first colon
      QString session_name = session.split(':').first();
      running_sessions.push_back(session_name.toStdString());
      any_session_running = true;
    }
  }
  
  // Update button states
  for (size_t i = 0; i < tmux_config_buttons_.size(); ++i) {
    bool is_running = std::find(running_sessions.begin(), running_sessions.end(), 
                               tmux_config_names_[i]) != running_sessions.end();
    tmux_config_buttons_[i]->setChecked(is_running);
    
    // Change button text to indicate running state
    QString button_text = QString::fromStdString(tmux_config_names_[i]);
    if (is_running) {
      button_text += " (Running)";
    }
    tmux_config_buttons_[i]->setText(button_text);
  }
  
  // Enable/disable stop button based on whether any session is running
  stop_config_button_->setEnabled(any_session_running);
}

bool MainWindow::IsGuiNodeRunning() {
  QProcess process;
  process.start("ros2", QStringList() << "node" << "list");
  process.waitForFinished(3000);
  
  if (process.exitCode() == 0) {
    QString output = process.readAllStandardOutput();
    return output.contains("/ut_automata_gui");
  }
  return false;
}

std::string MainWindow::CreateTmuxConfigWithoutGui(const std::string& config_name) {
  QString original_path = QString("/home/orin/roboracer_ws/tmux/%1/.tmuxinator.yaml").arg(QString::fromStdString(config_name));
  
  QFile original_file(original_path);
  if (!original_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return ""; // Return empty string on error
  }
  
  QTextStream in(&original_file);
  QString content = in.readAll();
  original_file.close();
  
  // Remove lines containing "ros2 run ut_automata gui"
  QStringList lines = content.split('\n');
  QStringList filtered_lines;
  
  for (const QString& line : lines) {
    if (!line.contains("ros2 run ut_automata gui")) {
      filtered_lines.append(line);
    }
  }
  
  // Create temporary file
  QTemporaryFile* temp_file = new QTemporaryFile();
  temp_file->setFileTemplate(QString("/tmp/tmux_%1_nogui_XXXXXX.yaml").arg(QString::fromStdString(config_name)));
  temp_file->setAutoRemove(false); // We'll clean it up manually later
  
  if (!temp_file->open()) {
    delete temp_file;
    return "";
  }
  
  QTextStream out(temp_file);
  out << filtered_lines.join('\n');
  temp_file->close();
  
  std::string temp_path = temp_file->fileName().toStdString();
  delete temp_file;
  return temp_path;
}

void MainWindow::StartTmuxConfiguration() {
  QPushButton* button = qobject_cast<QPushButton*>(sender());
  if (!button) return;
  
  // Find which configuration was clicked
  for (size_t i = 0; i < tmux_config_buttons_.size(); ++i) {
    if (tmux_config_buttons_[i] == button) {
      std::string config_name = tmux_config_names_[i];
      
      // Check if GUI node is already running
      bool gui_running = IsGuiNodeRunning();
      
      if (gui_running) {
        // Create a temporary config without GUI
        std::string temp_config_path = CreateTmuxConfigWithoutGui(config_name);
        if (!temp_config_path.empty()) {
          std::string command = "tmuxinator start -p " + temp_config_path + 
                               " --name " + config_name + 
                               " && rm " + temp_config_path; // Clean up temp file
          Exec(command);
        } else {
          printf("Error: Could not create temporary config without GUI\n");
        }
      } else {
        // Use original config
        std::string command = "cd /home/orin/roboracer_ws/tmux/" + config_name + " && tmuxinator start";
        Exec(command);
      }
      break;
    }
  }
}

void MainWindow::StopTmuxConfiguration() {
  // Get list of running tmux sessions and kill each one individually
  QProcess process;
  process.start("tmux", QStringList() << "ls");
  process.waitForFinished(1000);
  
  if (process.exitCode() == 0) {
    QString output = process.readAllStandardOutput();
    QStringList sessions = output.split('\n', Qt::SkipEmptyParts);
    
    for (const QString& session : sessions) {
      // Session name is everything before the first colon
      QString session_name = session.split(':').first();
      std::string kill_command = "tmux kill-session -t " + session_name.toStdString();
      Exec(kill_command);
    }
  }
}

void MainWindow::UpdateRecordingStatus(bool recording) {
  UpdateRecordingStatusSignal(recording);
}

void MainWindow::UpdateRecordingStatusSlot(bool recording) {
  if (recording_led_) {
    recording_led_->SetStatus(recording);
  }
}

void MainWindow::UpdateRecordingState(Led::RecordingState state) {
  UpdateRecordingStateSignal(state);
}

void MainWindow::UpdateRecordingStateSlot(Led::RecordingState state) {
  if (recording_led_) {
    if (state == Led::INACTIVE) {
      recording_led_->setVisible(false);
      // Remove Recorder tab if present
      int index = tab_widget_->indexOf(recorder_widget_);
      if (index != -1) {
        tab_widget_->removeTab(index);
      }
    } else {
      recording_led_->setVisible(true);
      recording_led_->SetRecordingState(state);
      // Add Recorder tab if not present
      if (tab_widget_->indexOf(recorder_widget_) == -1) {
        tab_widget_->addTab(recorder_widget_, tr("Recorder"));
      }
    }
  }
}

void MainWindow::UpdateAvailableTopics(const std::vector<std::string>& topics) {
  UpdateAvailableTopicsSignal(topics);
}

void MainWindow::UpdateAvailableTopicsSlot(const std::vector<std::string>& topics) {
  // Check if topics actually changed to avoid unnecessary rebuilds
  if (topics == available_topics_) return;
  
  available_topics_ = topics;
  
  // Get layout to update buttons
  if (!recorder_widget_) return;
  QVBoxLayout* recorder_layout = qobject_cast<QVBoxLayout*>(recorder_widget_->layout());
  if (!recorder_layout || recorder_layout->count() < 2) return;
  
  QWidget* topics_container = recorder_layout->itemAt(1)->widget();
  if (!topics_container) return;
  
  QVBoxLayout* topics_layout = qobject_cast<QVBoxLayout*>(topics_container->layout());
  if (!topics_layout) return;
  
  // Clear existing buttons
  QLayoutItem* item;
  while ((item = topics_layout->takeAt(0)) != nullptr) {
    delete item->widget();
    delete item;
  }
  topic_buttons_.clear();
  
  // Add new buttons
  QFont font("Arial");
  font.setPointSize(20);
  
  for (const auto& topic : available_topics_) {
    QPushButton* topic_btn = new QPushButton(QString::fromStdString(topic));
    topic_btn->setFont(font);
    topic_btn->setCheckable(true);
    
    // Check if topic is in selected list
    bool is_selected = std::find(selected_topics_.begin(), 
                                 selected_topics_.end(), 
                                 topic) != selected_topics_.end();
    topic_btn->setChecked(is_selected);
    
    connect(topic_btn, SIGNAL(clicked()), this, SLOT(ToggleTopicRecording()));
    topic_buttons_.push_back(topic_btn);
    topics_layout->addWidget(topic_btn);
  }
  
  topics_layout->addStretch();
}

void MainWindow::ToggleTopicRecording() {
  QPushButton* button = qobject_cast<QPushButton*>(sender());
  if (!button) return;
  
  // Find which topic was toggled
  for (size_t i = 0; i < topic_buttons_.size(); ++i) {
    if (topic_buttons_[i] == button) {
      std::string topic = available_topics_[i];
      
      // Update selected topics list
      auto it = std::find(selected_topics_.begin(), selected_topics_.end(), topic);
      if (button->isChecked()) {
        if (it == selected_topics_.end()) {
          selected_topics_.push_back(topic);
        }
      } else {
        if (it != selected_topics_.end()) {
          selected_topics_.erase(it);
        }
      }
      
      // TODO: Publish updated topic list to recorder node
      UpdateTopicsToRecord();
      break;
    }
  }
}

void MainWindow::UpdateTopicsToRecord() {
  // This will be called when topics are toggled
  // For now, we'll log the change
  std::string topics_str = "Selected topics: ";
  for (const auto& topic : selected_topics_) {
    topics_str += topic + ", ";
  }
  printf("%s\n", topics_str.c_str());
  
  // TODO: Publish to /recorder/set_topics topic
}

}  // namespace ut_automata_gui
