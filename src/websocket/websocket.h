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

#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/Point2D.h"
#include "f1tenth_course/ColoredPoint2D.h"
#include "f1tenth_course/ColoredLine2D.h"
#include "f1tenth_course/ColoredArc2D.h"
#include "f1tenth_course/PathVisualization.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "sensor_msgs/LaserScan.h"

class QWebSocketServer;
class QWebSocket;

struct MessageHeader {
  MessageHeader() : nonce(42) {}
  uint32_t nonce;                 // 1
  uint32_t num_particles;         // 2
  uint32_t num_path_options;      // 3
  uint32_t num_points;            // 4
  uint32_t num_lines;             // 5
  uint32_t num_arcs;              // 6
  uint32_t num_laser_rays;        // 7
  uint32_t num_local_points;      // 8
  uint32_t num_local_lines;       // 9
  uint32_t num_local_arcs;        // 10
  float laser_min_angle;          // 11
  float laser_max_angle;          // 12
  float sim_loc_x;                // 13
  float sim_loc_y;                // 14
  float sim_loc_r;                // 15
  size_t GetByteLength() const {
    const size_t len = 15 * 4 +
        num_particles * 3 * 4 +     // x, y, theta
        num_path_options * 3 * 4 +  // curvature, distance, clearance
        num_points * 3 * 4 +        // x, y, color
        num_lines * 5 * 4 +         // x1, y1, x2, y2, color
        num_arcs * 6 * 4 +          // x, y, radius, start_angle, end_angle, color
        num_laser_rays * 2;         // each ray is uint16_t
    return len;
  }
};

struct DataMessage {
  MessageHeader header;
  std::vector<uint16_t> laser_scan;
  std::vector<f1tenth_course::Pose2Df> particles;
  std::vector<f1tenth_course::PathVisualization> path_options;
  std::vector<f1tenth_course::ColoredPoint2D> points;
  std::vector<f1tenth_course::ColoredLine2D> lines;
  std::vector<f1tenth_course::ColoredArc2D> arcs;
  QByteArray ToByteArray() const;
  static DataMessage FromRosMessages(
      const sensor_msgs::LaserScan& laser_msg,
      const f1tenth_course::VisualizationMsg& local_msg,
      const f1tenth_course::VisualizationMsg& global_msg,
      float sim_loc_x,
      float sim_loc_y,
      float sim_loc_r);
};

class RobotWebSocket : public QObject {
  Q_OBJECT
public:
  explicit RobotWebSocket(uint16_t port);
  ~RobotWebSocket();
  void Send(const f1tenth_course::VisualizationMsg& local_vis,
            const f1tenth_course::VisualizationMsg& global_vis,
            const sensor_msgs::LaserScan& laser_scan,
            float sim_loc_x,
            float sim_loc_y,
            float sim_loc_r);

Q_SIGNALS:
  void closed();
  void SendDataSignal();
  void SetInitialPoseSignal(float x, float y, float theta);
  void SetNavGoalSignal(float x, float y, float theta);

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
  f1tenth_course::VisualizationMsg local_vis_;
  f1tenth_course::VisualizationMsg global_vis_;
  sensor_msgs::LaserScan laser_scan_;
  float sim_loc_x_;
  float sim_loc_y_;
  float sim_loc_r_;
};

#endif //ECHOSERVER_H
