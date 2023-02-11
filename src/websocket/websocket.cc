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

#include <string.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"

#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QtCore/QDebug>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/Point2D.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredText.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "sensor_msgs/LaserScan.h"

using amrl_msgs::Localization2DMsg;
using amrl_msgs::Point2D;
using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::ColoredText;
using amrl_msgs::VisualizationMsg;
using sensor_msgs::LaserScan;
using std::vector;

DEFINE_uint64(max_connections, 4, "Maximum number of websocket connections");

QT_USE_NAMESPACE

RobotWebSocket::RobotWebSocket(uint16_t port) :
    QObject(nullptr),
    ws_server_(new QWebSocketServer(("Robot Websocket Server"),
        QWebSocketServer::NonSecureMode, this)) {
  localization_.header.stamp = ros::Time(0);
  localization_.header.seq = 0;
  localization_.pose.x = 0;
  localization_.pose.y = 0;
  localization_.pose.theta = 0;
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
  if(clients_.size() > 0) {
    for (auto c : clients_) {
      delete c;
    }
  }
  clients_.clear();
}


void RobotWebSocket::onNewConnection() {
  QWebSocket *new_client = ws_server_->nextPendingConnection();
  
  if (clients_.size() >= FLAGS_max_connections) {
    // We already have the max number of clients
    new_client->sendTextMessage(
        "{ \"error\": \"Too many clients\" }");
    qInfo() << "Ignoring new client" << new_client
            << ", too many existing clients:" << clients_.size();
    delete new_client;
    return;
  }
  clients_.push_back(new_client);
  qInfo() << "New client: " << new_client << ", "
          << clients_.size() << "/" << FLAGS_max_connections;
  connect(new_client,
          &QWebSocket::textMessageReceived,
          this,
          &RobotWebSocket::processTextMessage);
  connect(new_client,
          &QWebSocket::binaryMessageReceived,
          this,
          &RobotWebSocket::processBinaryMessage);
  connect(new_client,
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
  msg.points.resize(h.num_points);
  msg.lines.resize(h.num_lines);
  msg.arcs.resize(h.num_arcs);
  msg.messages.resize(h.num_messages);
  for (size_t i = 0; i < msg.laser_scan.size(); ++i) {
    msg.laser_scan[i] = 10 * i;
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

    for(size_t i = 0; i < msg.messages.size(); i++) {
      msg.messages[i].start.x = 1.0 * i;
      msg.messages[i].start.y = 2.0 * i;
      msg.messages[i].color = (x << 16) | (x << 8) | x;
      msg.messages[i].size_em = 3.0 * i;
      const char *s = std::to_string(i).c_str();
      strncpy(msg.messages[i].text, s, i/10);
    }
  }
  return msg;
}

QByteArray DataMessage::ToByteArray() const {
  QByteArray data;
  data.resize(header.GetByteLength());
  char* buf = data.data();
  buf = WriteElement(header, buf);
  buf = WriteElementVector(laser_scan, buf);
  buf = WriteElementVector(points, buf);
  buf = WriteElementVector(lines, buf);
  buf = WriteElementVector(arcs, buf);
  buf = WriteElementVector(messages, buf);
  return data;
}

DataMessage DataMessage::FromRosMessages(
      const LaserScan& laser_msg,
      const VisualizationMsg& local_msg,
      const VisualizationMsg& global_msg,
      const Localization2DMsg& localization_msg) {
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

  // msg.messages.resize(local_msg.messages.size() + global_msg.messages.size());
  msg.header.num_local_messages = local_msg.messages.size();
  msg.header.num_messages = local_msg.messages.size() + global_msg.messages.size();
  for(amrl_msgs::ColoredText text : local_msg.messages) {
    ColoredTextNative localText;
    localText.start = text.start;
    localText.color = text.color;
    localText.size_em = text.size_em;
    size_t size = std::min(sizeof(localText.text) - 1, text.text.size());
    strncpy(localText.text, text.text.data(), size);
    localText.text[size] = 0;
    msg.messages.push_back(localText);
  }
  for(amrl_msgs::ColoredText text : global_msg.messages) {
    ColoredTextNative localText;
    localText.start = text.start;
    localText.color = text.color;
    localText.size_em = text.size_em;
    size_t size = std::min(sizeof(localText.text) - 1, text.text.size());
    strncpy(localText.text, text.text.data(), size);
    localText.text[size] = 0;
    msg.messages.push_back(localText);
  }

  if (kDebug) {
    printf("nonce: %d "
           "num_points: %d "
           "num_lines: %d "
           "num_arcs: %d "
           "num_messages: %d "
           "num_laser_rays: %d "
           "num_local_points: %d "
           "num_local_lines: %d "
           "num_local_arcs: %d "
           "num_local_messages: %d\n"
           ,
           msg.header.nonce,
           msg.header.num_points,
           msg.header.num_lines,
           msg.header.num_arcs,
           msg.header.num_messages,
           msg.header.num_laser_rays,
           msg.header.num_local_points,
           msg.header.num_local_lines,
           msg.header.num_local_arcs,
           msg.header.num_local_messages);
  }
  return msg;
}

void RobotWebSocket::SendError(const QString& error_val) {
  for (auto c: clients_) {
    CHECK_NOTNULL(c);
    c->sendTextMessage("{ \"error\": \"" + error_val + "\" }");
  }
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

bool StringKeyPresent(const QString& key,
                      const QJsonObject& json) {
  if (!json.contains(key)) return false;
  const QJsonValue val = json.value(key);
  return val.isString();
}

void RobotWebSocket::ProcessCallback(const QJsonObject& json) {
  static const bool kDebug = false;
  if (kDebug) {
    qInfo() << "Callback JSON:\n" << json;
  }
  if (!json.contains("type")) {
    SendError("Malformed request");
    return;
  }
  const auto type = json.value("type");
  if (type == "set_initial_pose") {
    if (!AllNumericalKeysPresent({"x", "y", "theta"}, json) ||
        !StringKeyPresent("map", json)) {
      SendError("Invalid set_initial_pose parameters");
    }
    SetInitialPoseSignal(json.value("x").toDouble(),
                         json.value("y").toDouble(),
                         json.value("theta").toDouble(),
                         json.value("map").toString());
   } else if (type == "set_nav_goal") {
    if (!AllNumericalKeysPresent({"x", "y", "theta"}, json) ||
        !StringKeyPresent("map", json)) {
      SendError("Invalid set_nav_goal parameters");
    }
    SetNavGoalSignal(json.value("x").toDouble(),
                     json.value("y").toDouble(),
                     json.value("theta").toDouble(),
                     json.value("map").toString());
  } else {
    SendError("Unrecognized request type");
  }
}

void RobotWebSocket::processTextMessage(QString message) {
  static const bool kSendTestMessage = false;
  QWebSocket *client = qobject_cast<QWebSocket *>(sender());
  CHECK_NOTNULL(client);
  QJsonDocument doc = QJsonDocument::fromJson(message.toLocal8Bit());
  QJsonObject json = doc.object();
  ProcessCallback(json);

  if (kSendTestMessage) {
    MessageHeader header;
    header.num_points = 40;
    header.num_lines = 100;
    header.num_arcs = 100;
    header.num_messages = 5;
    header.num_laser_rays = 270 * 4;
    header.laser_min_angle = -135;
    header.laser_max_angle = 135;
    const DataMessage data_msg = GenerateTestData(header);
    printf("Test message data size: %lu\n", header.GetByteLength());
    client->sendBinaryMessage(data_msg.ToByteArray());
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
  QWebSocket* client = qobject_cast<QWebSocket *>(sender());
  auto client_iter = std::find(clients_.begin(), clients_.end(), client);
  if (client_iter == clients_.end()) {
    fprintf(stderr, "ERROR: unknown socket disconnected!\n");
    delete client;
    return;
  }
  qDebug() << "socketDisconnected:" << client;
  delete client;
  clients_.erase(client_iter);
}

void RobotWebSocket::SendDataSlot() {
  if (clients_.empty()) return;
  data_mutex_.lock();
  const auto data = DataMessage::FromRosMessages(
      laser_scan_, local_vis_, global_vis_, localization_);
  const auto buffer = data.ToByteArray();
  CHECK_EQ(data.header.GetByteLength(), buffer.size());
  for (auto c : clients_) {
    c->sendBinaryMessage(buffer);
  }
  data_mutex_.unlock();
}

void RobotWebSocket::Send(const VisualizationMsg& local_vis,
                          const VisualizationMsg& global_vis,
                          const LaserScan& laser_scan,
                          const Localization2DMsg& localization) {
  data_mutex_.lock();
  localization_ = localization;
  local_vis_ = local_vis;
  global_vis_ = global_vis;
  laser_scan_ = laser_scan;
  data_mutex_.unlock();
  SendDataSignal();
}

