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
 * \file    websocket.h
 * \brief   Lightweight interface to send data from a robot
 *          to a web-based visualization page.
 * \author  Joydeep Biswas, (C) 2020
 */
//========================================================================
#ifndef ECHOSERVER_H
#define ECHOSERVER_H

#include <stdint.h>
#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/point2_d.hpp"
#include "amrl_msgs/msg/colored_point2_d.hpp"
#include "amrl_msgs/msg/colored_line2_d.hpp"
#include "amrl_msgs/msg/colored_arc2_d.hpp"
#include "amrl_msgs/msg/colored_text.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class QWebSocketServer;
class QWebSocket;

struct MessageHeader {
  MessageHeader() : nonce(42) {}
  uint32_t nonce;                     // 1
  uint32_t num_points;                // 2
  uint32_t num_lines;                 // 3
  uint32_t num_arcs;                  // 4
  uint32_t num_text_annotations;      // 5 
  uint32_t num_laser_rays;            // 6
  uint32_t num_local_points;          // 7
  uint32_t num_local_lines;           // 8
  uint32_t num_local_arcs;            // 9
  uint32_t num_local_text_annotations;// 10
  float laser_min_angle;              // 11
  float laser_max_angle;              // 12
  float loc_x;                        // 13
  float loc_y;                        // 14
  float loc_r;                        // 15
  char map[32];                       //
  size_t GetByteLength() const {
    const size_t len = 15 * 4 + 32 + // header fields + map data
        num_laser_rays * 4 +   // each ray is uint32_t
        num_points * 3 * 4 +   // x, y, color
        num_lines * 5 * 4 +    // x1, y1, x2, y2, color
        num_arcs * 6 * 4 +      // x, y, radius, start_angle, end_angle, color
        num_text_annotations * 4 * 4 * 32; // x, y, color, size, msg
    return len;
  }
};

struct ColoredTextNative {
  amrl_msgs::msg::Point2D start;
  uint32_t color;
  float size_em;
  char text[32];
};

struct DataMessage {
  MessageHeader header;
  std::vector<uint32_t> laser_scan;
  std::vector<amrl_msgs::msg::ColoredPoint2D> points;
  std::vector<amrl_msgs::msg::ColoredLine2D> lines;
  std::vector<amrl_msgs::msg::ColoredArc2D> arcs;
  std::vector<ColoredTextNative> text_annotations;
  QByteArray ToByteArray() const;
  static DataMessage FromRosMessages(
      const sensor_msgs::msg::LaserScan& laser_msg,
      const amrl_msgs::msg::VisualizationMsg& local_msg,
      const amrl_msgs::msg::VisualizationMsg& global_msg,
      const amrl_msgs::msg::Localization2DMsg& localization_msg);
};

class RobotWebSocket : public QObject {
  Q_OBJECT
public:
  explicit RobotWebSocket(uint16_t port);
  ~RobotWebSocket();
  void Send(const amrl_msgs::msg::VisualizationMsg& local_vis,
            const amrl_msgs::msg::VisualizationMsg& global_vis,
            const sensor_msgs::msg::LaserScan& laser_scan,
            const amrl_msgs::msg::Localization2DMsg& localization);

Q_SIGNALS:
  void closed();
  void SendDataSignal();
  void SetInitialPoseSignal(float x, float y, float theta, QString map);
  void SetNavGoalSignal(float x, float y, float theta, QString map);

private Q_SLOTS:
  void onNewConnection();
  void processTextMessage(QString message);
  void processBinaryMessage(QByteArray message);
  void socketDisconnected();
  void SendDataSlot();

private:
  void ProcessCallback(const QJsonObject& json);
  void SendError(const QString& error_val);

private:
  QWebSocketServer* ws_server_;
  std::vector<QWebSocket*> clients_;

  QMutex data_mutex_;
  amrl_msgs::msg::VisualizationMsg local_vis_;
  amrl_msgs::msg::VisualizationMsg global_vis_;
  sensor_msgs::msg::LaserScan laser_scan_;
  amrl_msgs::msg::Localization2DMsg localization_;
};

#endif //ECHOSERVER_H
