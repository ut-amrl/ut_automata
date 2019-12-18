// Copyright 2017-2018 slane@cs.umass.edu
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

#include <string.h>
#include <linux/joystick.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "joystick.h"

using std::string;
using std::vector;

namespace joystick {
static const bool kDebug = false;

Joystick::Joystick() : MaxAxisVal(32767), MaxAxisValInv(1.0 / MaxAxisVal) {
  fd = -1;
  model = NULL;
  model_size = 0;
}

bool Joystick::Open(const char *dev) {
  Close();
  fd = ::open(dev, O_RDONLY);
  if (fd < 0) return(false);

  char num_axes = 0;
  char num_buttons = 0;
  char name[256];
  ioctl(fd, JSIOCGAXES,               &num_axes);
  ioctl(fd, JSIOCGBUTTONS,            &num_buttons);
  ioctl(fd, JSIOCGNAME(sizeof(name)), name);
  // if (num_axes < 0) num_axes = 0;
  // if (num_buttons < 0) num_buttons = 0;
  axes_.resize(num_axes, 0);
  buttons_.resize(num_buttons, 0);
  name_ = string(name);

  printf("Opened Joystick '%s' with %d axes, %d buttons\n",
         name_.c_str(), num_axes, num_buttons);
  return(true);
}

bool Joystick::Open(int joy_idx) {
  static const int DevMax = 32;
  char dev[DevMax];

  snprintf(dev, DevMax, "/dev/input/js%d", joy_idx);
  if (Open(dev)) return(true);

  snprintf(dev, DevMax, "/dev/js%d", joy_idx);
  if (Open(dev)) return(true);

  return(false);
}

void Joystick::GetAllAxes(std::vector<float>* axes) {
  *axes = axes_;
}

void Joystick::GetAllButtons(std::vector<int32_t>* buttons) {
  *buttons = buttons_;
}

float Joystick::GetAxis(unsigned int idx) {
  if (idx + 1 > axes_.size()) {
    fprintf(stderr, "ERROR: Invalid axis %d\n", idx);
    return 0;
  }
  return axes_[idx];
}

int32_t Joystick::GetButton(unsigned int idx) {
  if (idx + 1 > buttons_.size()) {
    fprintf(stderr, "ERROR: Invalid button %d\n", idx);
    return 0;
  }
  return buttons_[idx];
}

std::string Joystick::GetName() {
  return name_;
}

int Joystick::ProcessEvents(int max_wait_time_ms) {
  struct pollfd polls[1];
  int total_num_events = 0;
  int num_events;
  struct js_event jse;

  if (fd == -1) return(-1);

  polls[0].fd = fd;
  polls[0].events = POLLIN | POLLPRI;
  polls[0].revents = 0;

  while ((num_events = poll(polls, 1, max_wait_time_ms)) > 0) {
    total_num_events += num_events;
    if (polls[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
      fprintf(stderr, "error returned from poll on joystick\n");
      return(-1);
    }

    if (polls[0].revents & (POLLIN | POLLPRI)) {
      if (read(fd, &jse, sizeof(jse)) != sizeof(jse)) {
        perror("read failed on pollable joystick\n");
        return(-1);
      }

      switch (jse.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
          if (kDebug) printf("axis %d now has value %d\n", jse.number,
                            jse.value);
          if (jse.number + 1 > static_cast<int>(axes_.size())) {
            fprintf(stderr,
                    "ERROR: Received event for invalid axis %d\n",
                    jse.number);
          }
          axes_[jse.number] = static_cast<float>(jse.value) / 32767.0;
          break;
        case JS_EVENT_BUTTON:
          if (kDebug) printf("button %d now has value %d\n", jse.number,
                           jse.value);
          if (jse.number + 1 > static_cast<int>(buttons_.size())) {
            fprintf(stderr,
                    "ERROR: Received event for invalid button %d\n",
                    jse.number);
          }
          buttons_[jse.number] = jse.value;
          break;
      }
    }
  }

  if (num_events < 0) {
    perror("Unable to poll joystick");
    return(-1);
  }

  return(total_num_events);
}

void Joystick::Close() {
  if (fd >= 0) {
    ::close(fd);
  }
}

}  // namespace joystick
