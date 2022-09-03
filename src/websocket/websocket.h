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

class QWebSocketServer;
class QWebSocket;

struct MessageHeader {
  MessageHeader() : nonce(42) {}
  uint32_t nonce;                 // 1
  uint32_t num_points;            // 2
  uint32_t num_lines;             // 3
  uint32_t num_arcs;              // 4
  uint32_t num_laser_rays;        // 5
  uint32_t num_local_points;      // 6
  uint32_t num_local_lines;       // 7
  uint32_t num_local_arcs;        // 8
  float laser_min_angle;          // 9
  float laser_max_angle;          // 10
  float loc_x;                    // 11
  float loc_y;                    // 12
  float loc_r;                    // 13
  char map[32];                   //
  size_t GetByteLength() const {
    const size_t len = 13 * 4 + 32 +
        num_laser_rays * 4 +   // each ray is uint32_t
        num_points * 3 * 4 +   // x, y, color
        num_lines * 5 * 4 +    // x1, y1, x2, y2, color
        num_arcs * 6 * 4;      // x, y, radius, start_angle, end_angle, color
    return len;
  }
};

struct DataMessage {
  MessageHeader header;
  std::vector<uint32_t> laser_scan;
  std::vector<amrl_msgs::ColoredPoint2D> points;
  std::vector<amrl_msgs::ColoredLine2D> lines;
  std::vector<amrl_msgs::ColoredArc2D> arcs;
  QByteArray ToByteArray() const;
  template <typename LaserScanType, typename VisualizationType, typename LocalizationType>
  static DataMessage FromRosMessages(
      const LaserScanType& laser_msg,
      const VisualizationType& local_msg,
      const VisualizationType& global_msg,
      const LocalizationType& localization_msg);
};

class RobotWebSocket : public QObject {
  Q_OBJECT
public:
  explicit RobotWebSocket(uint16_t port);
  ~RobotWebSocket();
  void Send(const amrl_msgs::VisualizationMsg& local_vis,
            const amrl_msgs::VisualizationMsg& global_vis,
            const sensor_msgs::LaserScan& laser_scan,
            const amrl_msgs::Localization2DMsg& localization);

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
  amrl_msgs::VisualizationMsg local_vis_;
  amrl_msgs::VisualizationMsg global_vis_;
  sensor_msgs::LaserScan laser_scan_;
  amrl_msgs::Localization2DMsg localization_;
};

template <typename LaserScanType,
          typename VisualizationType,
          typename LocalizationType>
DataMessage DataMessage::FromRosMessages(
      const LaserScanType& laser_msg,
      const VisualizationType& local_msg,
      const VisualizationType& global_msg,
      const LocalizationType& localization_msg) {
static const bool kDebug = false;
  DataMessage msg;
  for (size_t i = 0; i < sizeof(msg.header.map); ++i) {
    msg.header.map[i] = 0;
  }
  msg.header.loc_x = localization_msg.pose.x;
  msg.header.loc_y = localization_msg.pose.y;
  msg.header.loc_r = localization_msg.pose.theta;
  strncpy(msg.header.map,
          localization_msg.map.data(),
          std::min(sizeof(msg.header.map) - 1, localization_msg.map.size()));
  msg.header.laser_min_angle = laser_msg.angle_min;
  msg.header.laser_max_angle = laser_msg.angle_max;
  msg.header.num_laser_rays = laser_msg.ranges.size();
  msg.laser_scan.resize(laser_msg.ranges.size());
  for (size_t i = 0; i < laser_msg.ranges.size(); ++i) {
    if (laser_msg.ranges[i] <= laser_msg.range_min ||
        laser_msg.ranges[i] >= laser_msg.range_max) {
      msg.laser_scan[i] = 0;
    } else {
      msg.laser_scan[i] = static_cast<uint32_t>(laser_msg.ranges[i] * 1000.0);
    }
  }

  msg.points = local_msg.points;
  msg.header.num_local_points = local_msg.points.size();
  msg.points.insert(msg.points.end(),
                    global_msg.points.begin(),
                    global_msg.points.end());

  msg.lines = local_msg.lines;
  msg.header.num_local_lines = local_msg.lines.size();
  msg.lines.insert(msg.lines.end(),
                   global_msg.lines.begin(),
                   global_msg.lines.end());

  msg.arcs = local_msg.arcs;
  msg.header.num_local_arcs = local_msg.arcs.size();
  msg.arcs.insert(msg.arcs.end(),
                  global_msg.arcs.begin(),
                  global_msg.arcs.end());

  msg.header.num_points = msg.points.size();
  msg.header.num_lines = msg.lines.size();
  msg.header.num_arcs = msg.arcs.size();

  if (kDebug) {
    printf("nonce: %d "
           "num_points: %d "
           "num_lines: %d "
           "num_arcs: %d "
           "num_laser_rays: %d "
           "num_local_points: %d "
           "num_local_lines: %d "
           "num_local_arcs: %d\n",
           msg.header.nonce,
           msg.header.num_points,
           msg.header.num_lines,
           msg.header.num_arcs,
           msg.header.num_laser_rays,
           msg.header.num_local_points,
           msg.header.num_local_lines,
           msg.header.num_local_arcs);
  }
  return msg;
}
#endif //ECHOSERVER_H
