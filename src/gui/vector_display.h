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
\file    vector_display.h
\brief   OpenGL Vector Display
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "QGLWidget"
#include "QMutex"

#ifndef VECTOR_DISPLAY_H
#define VECTOR_DISPLAY_H

namespace vector_display {
class VectorDisplay : public QGLWidget {
  Q_OBJECT

 public:
  explicit VectorDisplay(QWidget *parent = 0);


 public slots:
  void RedrawSlot();

 signals:
  void RedrawSignal();

 protected:
  void paintEvent(QPaintEvent * event);
  void resizeEvent(QResizeEvent * event);
  void initializeGL();
  void resizeGL(int width, int height);
  void SetupViewport();

 private:
  QMutex mutex_;
  float view_scale_;
  float min_z_value_;
  float max_z_value_;
};

}  // namespace vector_display

#endif  // VECTOR_DISPLAY_H
