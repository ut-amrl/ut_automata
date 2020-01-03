/****************************************************************************
**
** Copyright (C) 2016 Kurt Pattyn <pattyn.kurt@gmail.com>.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtWebSockets module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
#include "websocket.h"

#include <vector>

#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QtCore/QDebug>

#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/Point2D.h"
#include "f1tenth_course/ColoredPoint2D.h"
#include "f1tenth_course/ColoredLine2D.h"
#include "f1tenth_course/ColoredArc2D.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "f1tenth_course/PathVisualization.h"

using f1tenth_course::Pose2Df;
using f1tenth_course::Point2D;
using f1tenth_course::ColoredArc2D;
using f1tenth_course::ColoredLine2D;
using f1tenth_course::ColoredPoint2D;
using f1tenth_course::VisualizationMsg;
using f1tenth_course::PathVisualization;
using std::vector;

QT_USE_NAMESPACE

//! [constructor]
EchoServer::EchoServer(quint16 port,
                       bool debug) :
    QObject(nullptr),
    m_pWebSocketServer(new QWebSocketServer(
        QStringLiteral("Echo Server"),
        QWebSocketServer::NonSecureMode,
        this)),
    m_debug(true) {
    if (m_pWebSocketServer->listen(QHostAddress::Any, port)) {
        if (m_debug) {
            qDebug() << "Echoserver listening on port" << port;
        }
        connect(m_pWebSocketServer,
                &QWebSocketServer::newConnection,
                this, &EchoServer::onNewConnection);
        connect(m_pWebSocketServer,
                &QWebSocketServer::closed, this, &EchoServer::closed);
    }
}
//! [constructor]

EchoServer::~EchoServer() {
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

//! [onNewConnection]
void EchoServer::onNewConnection() {
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket,
            &QWebSocket::textMessageReceived,
            this,
            &EchoServer::processTextMessage);
    connect(pSocket,
            &QWebSocket::binaryMessageReceived,
            this,
            &EchoServer::processBinaryMessage);
    connect(pSocket,
            &QWebSocket::disconnected,
            this,
            &EchoServer::socketDisconnected);
    m_clients << pSocket;
}
//! [onNewConnection]

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

//! [processTextMessage]
void EchoServer::processTextMessage(QString message) {
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug) {
        qDebug() << "Message received:" << message;
    }
    if (pClient) {
      QByteArray data;
      MessageHeader msg;
      msg.num_particles = 80;
      msg.num_path_options = 20;
      msg.num_points = 10;
      msg.num_lines = 0;
      msg.num_arcs = 0;
      msg.num_laser_rays = 270 * 4;
      msg.laser_min_angle = -135;
      msg.laser_max_angle = 135;
      data.resize(msg.GetByteLength());
      printf("Data size: %lu\n", msg.GetByteLength());

      char* buf = data.data();
      buf = WriteElement(msg, buf);

      // =============================================================
      // Initialize the data.
      vector<uint16_t> laser(msg.num_laser_rays, 0);
      for (size_t i = 0; i < laser.size(); ++i) {
        laser[i] = 10 * i;
      }
      vector<Pose2Df> particles(msg.num_particles);
      for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].x = 1.0 * static_cast<float>(i) + 0.1;
        particles[i].y = 2.0 * static_cast<float>(i) + 0.2;
        particles[i].theta = 3.0 * static_cast<float>(i) + 0.3;
      }
      vector<PathVisualization> path_options(msg.num_path_options);
      for (size_t i = 0; i < path_options.size(); ++i) {
        path_options[i].curvature = 1.0 * static_cast<float>(i) + 0.1;
        path_options[i].distance = 2.0 * static_cast<float>(i) + 0.2;
        path_options[i].clearance = 3.0 * static_cast<float>(i) + 0.3;
      }
      vector<ColoredPoint2D> points(msg.num_points);
      for (size_t i = 0; i < points.size(); ++i) {
        points[i].point.x = 1.0 * static_cast<float>(i) + 0.1;
        points[i].point.y = 2.0 * static_cast<float>(i) + 0.2;
        const uint8_t x = static_cast<uint8_t>(i);
        points[i].color = (x << 16) | (x << 8) | x;
      }
      vector<ColoredLine2D> lines(msg.num_lines);
      vector<ColoredArc2D> arcs(msg.num_arcs);
      // =============================================================

      buf = WriteElementVector(laser, buf);
      buf = WriteElementVector(particles, buf);
      buf = WriteElementVector(path_options, buf);
      buf = WriteElementVector(points, buf);
      buf = WriteElementVector(lines, buf);
      buf = WriteElementVector(arcs, buf);

      pClient->sendBinaryMessage(data);
    }
}
//! [processTextMessage]

//! [processBinaryMessage]
void EchoServer::processBinaryMessage(QByteArray message) {
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug) {
        qDebug() << "Binary Message received:" << message;
    }
    if (pClient) {
        pClient->sendBinaryMessage(message);
    }
}
//! [processBinaryMessage]

//! [socketDisconnected]
void EchoServer::socketDisconnected() {
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug) {
        qDebug() << "socketDisconnected:" << pClient;
    }
    if (pClient) {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}
//! [socketDisconnected]
