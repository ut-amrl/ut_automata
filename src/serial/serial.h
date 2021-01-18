// Copyright 2002 James Bruce, 2017 slane@cs.umass.edu
//
//
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
// ========================================================================

#ifndef SRC_SERIAL_SERIAL_H_
#define SRC_SERIAL_SERIAL_H_

#include <unistd.h>

class Serial{
 private:
  int fd;

 public:
  Serial() {fd = 0;}
  ~Serial() {close();}

  int getFd()
    {return(fd);}

  bool open(const char *device, int baud,
            int flags = 0, bool flow_control = false);
  void close();
  bool isOpen()
    {return(fd > 0);}

  int read(void *buf, int size)
    {return(::read(fd, buf, size));}
  int write(const void *buf, int size);
  void dump(const void *buf, int size);

  bool waitForInput(int timeout_msec);  // wait for input
  bool waitForOutput(int timeout_msec);  // wait to be able to write
};

#endif  // SRC_SERIAL_SERIAL_H_
