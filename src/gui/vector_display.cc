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
\file    vector_display.cc
\brief   OpenGL Vector Display
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <GL/gl.h>

#include "QGLWidget"

#include "vector_display.h"

namespace vector_display {

VectorDisplay::VectorDisplay(QWidget *parent) :
    QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    view_scale_(1),
    min_z_value_(-10),
    max_z_value_(10) {
  connect(this, SIGNAL(RedrawSignal()), this, SLOT(RedrawSlot()));
}

void VectorDisplay::initializeGL() {
  glEnable(GL_MULTISAMPLE);
}

void VectorDisplay::paintEvent(QPaintEvent* event) {
  // FunctionTimer ft(__FUNCTION__);
  mutex_.lock();
  makeCurrent();
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  // glEnable(GL_DEPTH_TEST);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glEnable(GL_LINE_SMOOTH);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



//   drawLines(lineThickness * view_scale_);
//   drawCircles(lineThickness * view_scale_);
//   drawPoints(pointsSize * view_scale_);
//   drawQuads();
//   drawTextStrings();


  swapBuffers();
  mutex_.unlock();
}


void VectorDisplay::resizeEvent(QResizeEvent* event) {
  QGLWidget::resizeEvent(event);
  RedrawSignal();
}


void VectorDisplay::resizeGL(int width, int height) {
  SetupViewport();
}

void VectorDisplay::RedrawSlot() {
}

void VectorDisplay::SetupViewport() {
  const int window_width = width();
  const int window_height = height();
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5*view_scale_*window_width,
          0.5*view_scale_*window_width,
          -0.5*view_scale_*window_height,
          0.5*view_scale_*window_height,
          min_z_value_, max_z_value_);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

}  // namespace vector_display
