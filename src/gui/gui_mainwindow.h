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
#include <QBrush>
#include <QFrame>
#include <QPainter>
#include <QWidget>
#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>
#include <QPushButton>

class QVBoxLayout;
class QTabWidget;
class QHBoxLayout;
class QGridLayout;
class QProcess;
class QDir;

namespace vector_display {
class VectorDisplay;
}  // namespace vector_display

namespace ut_automata_gui {

class TouchVectorDisplay;
class AdminPassword;
class HumanInteraction;

class CameraDisplay : public QLabel {
  Q_OBJECT

 public:
  CameraDisplay(QWidget* parent = nullptr);
  void UpdateImage(const QPixmap& pixmap);

 protected:
  void resizeEvent(QResizeEvent* event) override;

 private:
  QPixmap current_image_;
};

class Led : public QWidget {
  Q_OBJECT
  
 public:
  enum RecordingState {
    INACTIVE,    // Node not running - Grey
    READY,       // Node active, not recording - Red
    RECORDING    // Recording in progress - Green
  };
  
  Led(bool is_recording_led = false) : 
      status_on_(false), 
      is_recording_led_(is_recording_led),
      recording_state_(INACTIVE) {}
  
  void SetStatus(bool value) {
    if (status_on_ == value) return;
    status_on_ = value;
    update();
  }
  
  void SetRecordingState(RecordingState state) {
    if (recording_state_ == state) return;
    recording_state_ = state;
    update();
  }

 protected:
  void paintEvent(QPaintEvent *event) override {
    static const QBrush kGreenBrush = QBrush(QColor(0, 225, 0));
    static const QBrush kRedBrush = QBrush(QColor(255, 0, 0));
    static const QBrush kGreyBrush = QBrush(QColor(128, 128, 128));
    QPainter painter;
    painter.begin(this);
    
    if (is_recording_led_) {
      // Recording LED: Grey (inactive), Red (ready), Green (recording)
      switch (recording_state_) {
        case INACTIVE:
          painter.fillRect(QRectF(0, 0, width(), height()), kGreyBrush);
          break;
        case READY:
          painter.fillRect(QRectF(0, 0, width(), height()), kRedBrush);
          break;
        case RECORDING:
          painter.fillRect(QRectF(0, 0, width(), height()), kGreenBrush);
          break;
      }
    } else {
      // Normal status LED: Green when okay (on), Red when not okay (off)
      if (status_on_) {
        painter.fillRect(QRectF(0, 0, width(), height()), kGreenBrush);
      } else {
        painter.fillRect(QRectF(0, 0, width(), height()), kRedBrush);
      }
    }
    painter.end();
  }

 private:
  bool status_on_;
  bool is_recording_led_;
  RecordingState recording_state_;
};

class StatusLed : public QFrame {
  Q_OBJECT
  
 public:
  explicit StatusLed(QString name, bool is_recording_led = false);
  void SetStatus(bool value);
  void SetRecordingState(Led::RecordingState state);

 private:
  bool status_on_;
  Led* led_;
};

class DiskSpaceBar : public QWidget {
  Q_OBJECT

 public:
  explicit DiskSpaceBar(QWidget* parent = nullptr);
  void UpdateDiskSpace(float used_gb, float total_gb);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  float used_gb_;
  float total_gb_;
  QLabel* used_label_;
  QLabel* available_label_;
};

class RealStatus : public QFrame {
  Q_OBJECT

 public:
  explicit RealStatus(bool horizontal);

  void SetValue(float v) { 
    value_ = v; 
    update(); 
  }

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  bool horizontal_;
  float value_;
};

class MainWindow : public QWidget {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();
  void UpdateStatus(int mode, 
                    float battery,
                    bool drive_okay,
                    bool lidar_okay,
                    bool joystick_okay,
                    bool imu_okay,
                    float throttle,
                    float steering);
  void UpdateCamera(const QPixmap& image);
  void UpdateRecordingStatus(bool recording);
  void UpdateRecordingState(Led::RecordingState state);
  void UpdateAvailableTopics(const std::vector<std::string>& topics);

public slots:
  void closeWindow();
  void UpdateIP();
  void StartRos();
  void StartCar();
  void StartCamera();
  void UpdateStatusSlot(int mode, 
                        float battery,
                        bool drive_okay,
                        bool lidar_okay,
                        bool joystick_okay,
                        bool imu_okay,
                        float throttle,
                        float steering);
  void UpdateCameraSlot(const QPixmap& image);
  void UpdateTmuxConfigurations();
  void StartTmuxConfiguration();
  void StopTmuxConfiguration();
  void UpdateRecordingStatusSlot(bool recording);
  void UpdateRecordingStateSlot(Led::RecordingState state);
  void UpdateTopicsToRecord();
  void ToggleTopicRecording();
  void UpdateAvailableTopicsSlot(const std::vector<std::string>& topics);

signals:
  void UpdateQuestion(std::string question,
                      std::vector<std::string> responses);
  void UpdateSignal();
  void UpdateStatusSignal(int mode, 
                          float battery,
                          bool drive_okay,
                          bool lidar_okay,
                          bool joystick_okay,
                          bool imu_okay,
                          float throttle,
                          float steering);
  void UpdateCameraSignal(const QPixmap& image);
  void UpdateRecordingStatusSignal(bool recording);
  void UpdateRecordingStateSignal(Led::RecordingState state);
  void UpdateAvailableTopicsSignal(const std::vector<std::string>& topics);

private:

  // Used to show the time.
  QLabel* ipaddr_label_;

  // Vector display.
  QTabWidget* tab_widget_;

  // Camera image display.
  CameraDisplay* camera_display_;

  // Main layout of the window.
  QVBoxLayout* main_layout_;

  // Vector display.
  vector_display::VectorDisplay* display_;

  // Question ID.
  int interaction_id_;

  // Robot status display.
  QLabel* status_label_;
  
  // Disk space indicator
  DiskSpaceBar* disk_space_bar_;
  
  // Tmux configuration widgets
  std::vector<QPushButton*> tmux_config_buttons_;
  QPushButton* stop_config_button_;
  std::vector<std::string> tmux_config_names_;
  
  // Recording status
  StatusLed* recording_led_;
  QWidget* recorder_widget_;
  
  // Recording topic selection
  std::vector<std::string> available_topics_;
  std::vector<std::string> selected_topics_;
  std::vector<QPushButton*> topic_buttons_;
  
  // Helper functions for GUI-aware tmux configuration
  bool IsGuiNodeRunning();
  std::string CreateTmuxConfigWithoutGui(const std::string& config_name);
};


}  // namespace ut_automata_gui

#endif  // GUI_MAINWINDOW_H
