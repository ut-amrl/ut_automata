// Copyright 2017 - 2018 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include <stdint.h>
#include <string>
#include <vector>

#ifndef JOYSTICK_H
#define JOYSTICK_H

namespace joystick {

// Interface to kernel module for Joystick driver.
// Reference:
//    https://www.kernel.org/doc/html/latest/input/joydev/joystick-api.html
class Joystick {
 public:
  struct Model {
    unsigned int vbutton;  // virtual button mask
    unsigned char vaxis;   // virtual axis
    unsigned char type;    // 0=axis 1=button
    unsigned char index;   // axis or button indexuint8_t
    int min_value;         // minimum threshold value before
    float weight, bias;    // weight and bias to apply in transformation
  };

  Joystick();

  bool IsOpen() { return (fd != -1); }
  std::string GetName();

  bool Open(const char* dev);
  bool Open(int joy_idx);

  // -1 means wait indefinitely
  int ProcessEvents(int max_wait_time_ms);
  void Close();

  float GetAxis(unsigned int idx);
  void GetAllAxes(std::vector<float>* axes);
  int32_t GetButton(unsigned int idx);
  void GetAllButtons(std::vector<int32_t>* buttons);

  const int MaxAxisVal;
  const double MaxAxisValInv;

 private:
  int fd;

  static const int MaxModelSize = 1024;

  const int kDeadzone = 6000;

  const Model *model;
  int model_size;

 public:
  std::string name_;
  std::vector<int32_t> buttons_;
  std::vector<float> axes_;
};
}  // namespace joystick

#endif  // JOYSTICK_H
