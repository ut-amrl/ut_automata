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
 * \file    websocket.cc
 * \brief   Lightweight interface to send data from a robot
 *          to a web-based visualization page.
 * \author  Joydeep Biswas, (C) 2020
 */
//========================================================================
#include "websocket.h"

#include <iostream>
#include <vector>

#include "glog/logging.h"
#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QtCore/QDebug>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/Point2D.h"
#include "f1tenth_course/ColoredPoint2D.h"
#include "f1tenth_course/ColoredLine2D.h"
#include "f1tenth_course/ColoredArc2D.h"
#include "f1tenth_course/PathVisualization.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "sensor_msgs/LaserScan.h"

using f1tenth_course::Pose2Df;
using f1tenth_course::Point2D;
using f1tenth_course::ColoredArc2D;
using f1tenth_course::ColoredLine2D;
using f1tenth_course::ColoredPoint2D;
using f1tenth_course::VisualizationMsg;
using f1tenth_course::PathVisualization;
using sensor_msgs::LaserScan;
using std::vector;

QT_USE_NAMESPACE

RobotWebSocket::RobotWebSocket(uint16_t port) :
    QObject(nullptr),
    ws_server_(new QWebSocketServer(("Echo Server"),
        QWebSocketServer::NonSecureMode, this)),
    client_(nullptr) {
  connect(this,
          &RobotWebSocket::SendDataSignal,
          this,
          &RobotWebSocket::SendDataSlot);
  if (ws_server_->listen(QHostAddress::Any, port)) {
      qDebug() << "Listening on port" << port;
      connect(ws_server_,
              &QWebSocketServer::newConnection,
              this, &RobotWebSocket::onNewConnection);
      connect(ws_server_,
              &QWebSocketServer::closed, this, &RobotWebSocket::closed);
  }
}

RobotWebSocket::~RobotWebSocket() {
  ws_server_->close();
  if(client_) {
    delete client_;
    client_ = nullptr;
  }
}

void RobotWebSocket::onNewConnection() {
  QWebSocket *new_client = ws_server_->nextPendingConnection();
  if (client_ != nullptr) {
    // We already have a client.
    new_client->sendTextMessage(
        "{ \"error\": \"Too many clients\" }");
    qInfo() << "Ignoring new client" << new_client
            << ", we already have one:" << client_;
    delete new_client;
    return;
  }
  client_ = new_client;
  qInfo() << "New client: " << client_;
  connect(client_,
          &QWebSocket::textMessageReceived,
          this,
          &RobotWebSocket::processTextMessage);
  connect(client_,
          &QWebSocket::binaryMessageReceived,
          this,
          &RobotWebSocket::processBinaryMessage);
  connect(client_,
          &QWebSocket::disconnected,
          this,
          &RobotWebSocket::socketDisconnected);
}

template <typename T>
char* WriteElement(const T& x, char* const buf) {
  *reinterpret_cast<T*>(buf) = x;
  return (buf + sizeof(x));
}

template <typename T>
char* WriteElementVector(const std::vector<T>& v, char* const buf) {
  const size_t len = v.size() * sizeof(T);
  memcpy(buf, v.data(), len);
  return (buf + len);
}

DataMessage GenerateTestData(const MessageHeader& h) {
  DataMessage msg;
  msg.header = h;
  msg.laser_scan.resize(h.num_laser_rays);
  msg.particles.resize(h.num_particles);
  msg.path_options.resize(h.num_path_options);
  msg.points.resize(h.num_points);
  msg.lines.resize(h.num_lines);
  msg.arcs.resize(h.num_arcs);
  for (size_t i = 0; i < msg.laser_scan.size(); ++i) {
    msg.laser_scan[i] = 10 * i;
  }
  for (size_t i = 0; i < msg.particles.size(); ++i) {
    msg.particles[i].x = 1.0 * static_cast<float>(i) + 0.1;
    msg.particles[i].y = 2.0 * static_cast<float>(i) + 0.2;
    msg.particles[i].theta = 3.0 * static_cast<float>(i) + 0.3;
  }
  for (size_t i = 0; i < msg.path_options.size(); ++i) {
    msg.path_options[i].curvature = 1.0 * static_cast<float>(i) + 0.1;
    msg.path_options[i].distance = 2.0 * static_cast<float>(i) + 0.2;
    msg.path_options[i].clearance = 3.0 * static_cast<float>(i) + 0.3;
  }
  for (size_t i = 0; i < msg.points.size(); ++i) {
    msg.points[i].point.x = 1.0 * static_cast<float>(i) + 0.1;
    msg.points[i].point.y = 2.0 * static_cast<float>(i) + 0.2;
    const uint8_t x = static_cast<uint8_t>(i);
    msg.points[i].color = (x << 16) | (x << 8) | x;
  }
  for (size_t i = 0; i < msg.lines.size(); ++i) {
    msg.lines[i].p0.x = 0.1 * i;
    msg.lines[i].p0.y = 0.01 * i;
    msg.lines[i].p1.x = 1.0 * i;
    msg.lines[i].p1.y = 10.0 * i;
    const uint8_t x = static_cast<uint8_t>(i);
    msg.lines[i].color = (x << 16) | (x << 8) | x;
  }
  for (size_t i = 0; i < msg.arcs.size(); ++i) {
    msg.arcs[i].center.x = 1.0 * i;
    msg.arcs[i].center.y = 2.0 * i;
    msg.arcs[i].radius = i;
    msg.arcs[i].start_angle = 2.0 * i;
    msg.arcs[i].end_angle = 3.0 * i;
    if (i == 0) {
      msg.arcs[i].radius = 1.0 / 0.0;
      msg.arcs[i].start_angle = 0.0 / 0.0;
      msg.arcs[i].end_angle = -10.0 / 0.0;
    }
    const uint8_t x = static_cast<uint8_t>(i);
    msg.arcs[i].color = (x << 16) | (x << 8) | x;
  }
  return msg;
}

QByteArray DataMessage::ToByteArray() const {
  QByteArray data;
  data.resize(header.GetByteLength());
  char* buf = data.data();
  buf = WriteElement(header, buf);
  buf = WriteElementVector(laser_scan, buf);
  buf = WriteElementVector(particles, buf);
  buf = WriteElementVector(path_options, buf);
  buf = WriteElementVector(points, buf);
  buf = WriteElementVector(lines, buf);
  buf = WriteElementVector(arcs, buf);
  return data;
}

DataMessage DataMessage::FromRosMessages(
      const LaserScan& laser_msg,
      const VisualizationMsg& local_msg,
      const VisualizationMsg& global_msg) {
  DataMessage msg;
  msg.header.laser_min_angle = laser_msg.angle_min;
  msg.header.laser_max_angle = laser_msg.angle_max;
  msg.header.num_laser_rays = laser_msg.ranges.size();
  msg.laser_scan.resize(laser_msg.ranges.size());
  for (size_t i = 0; i < laser_msg.ranges.size(); ++i) {
    msg.laser_scan[i] = static_cast<uint16_t>(laser_msg.ranges[i] * 1000.0);
  }
  msg.particles = global_msg.particles;
  msg.path_options = local_msg.path_options;

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

  msg.header.num_particles = msg.particles.size();
  msg.header.num_path_options = msg.path_options.size();
  msg.header.num_points = msg.points.size();
  msg.header.num_lines = msg.lines.size();
  msg.header.num_arcs = msg.arcs.size();
  return msg;
}

void RobotWebSocket::SendError(const QString& error_val) {
  CHECK_NOTNULL(client_);
  client_->sendTextMessage(
        "{ \"error\": \"" + error_val + "\" }");
}

bool AllNumericalKeysPresent(const QStringList& expected,
                             const QJsonObject& json) {
  for (const QString& key : expected) {
    if (!json.contains(key)) return false;
    const QJsonValue val = json.value(key);
    if (!val.isDouble()) return false;
  }
  return true;
}

void RobotWebSocket::ProcessCallback(const QJsonObject& json) {
  static const bool kDebug = true;
  if (kDebug) {
    qInfo() << "Callback JSON:\n" << json;
  }
  if (!json.contains("type")) {
    SendError("Malformed request");
    return;
  }
  const auto type = json.value("type");
  if (type == "set_initial_pose") {
    if (!AllNumericalKeysPresent({"x", "y", "theta"}, json)) {
      SendError("Invalid set_initial_pose parameters");
    }
    SetInitialPoseSignal(json.value("x").toDouble(),
                         json.value("y").toDouble(),
                         json.value("theta").toDouble());
   } else if (type == "set_nav_goal") {
    if (!AllNumericalKeysPresent({"x", "y", "theta"}, json)) {
      SendError("Invalid set_nav_goal parameters");
    }
    SetNavGoalSignal(json.value("x").toDouble(),
                     json.value("y").toDouble(),
                     json.value("theta").toDouble());
  } else {
    SendError("Unrecognized request type");
  }
}

void RobotWebSocket::processTextMessage(QString message) {
  static const bool kSendTestMessage = false;
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  CHECK_EQ(client, client_);
  CHECK_NOTNULL(client_);
  QJsonDocument doc = QJsonDocument::fromJson(message.toLocal8Bit());
  QJsonObject json = doc.object();
  ProcessCallback(json);

  if (kSendTestMessage) {
    MessageHeader header;
    header.num_particles = 100;
    header.num_path_options = 20;
    header.num_points = 40;
    header.num_lines = 100;
    header.num_arcs = 100;
    header.num_laser_rays = 270 * 4;
    header.laser_min_angle = -135;
    header.laser_max_angle = 135;
    printf("Test message data size: %lu\n", header.GetByteLength());
    const DataMessage data_msg = GenerateTestData(header);
    client_->sendBinaryMessage(data_msg.ToByteArray());
  }
}

void RobotWebSocket::processBinaryMessage(QByteArray message) {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  qDebug() << "Binary Message received:" << message;
  if (client) {
    client->sendBinaryMessage(message);
  }
}


void RobotWebSocket::socketDisconnected() {
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  if (client == client_) {
    qDebug() << "socketDisconnected:" << client;
    delete client;
    client_ = nullptr;
  } else {
    // Should never happen!
    CHECK(false);
  }
}

void RobotWebSocket::SendDataSlot() {
  if (client_ == nullptr) return;
  data_mutex_.lock();
  const auto data =
      DataMessage::FromRosMessages(laser_scan_, local_vis_, global_vis_);
  const auto buffer = data.ToByteArray();
  CHECK_EQ(data.header.GetByteLength(), buffer.size());
  client_->sendBinaryMessage(buffer);
  data_mutex_.unlock();
}

void RobotWebSocket::Send(const VisualizationMsg& local_vis,
                          const VisualizationMsg& global_vis,
                          const LaserScan& laser_scan) {
  data_mutex_.lock();
  local_vis_ = local_vis;
  global_vis_ = global_vis;
  laser_scan_ = laser_scan;
  data_mutex_.unlock();
  SendDataSignal();
}

