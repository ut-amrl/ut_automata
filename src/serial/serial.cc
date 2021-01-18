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

#include "serial/serial.h"

#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <termios.h>
#include <string.h>


static const int NumBaudRates = 20;
static const unsigned BaudRates[NumBaudRates][2] = {
  {      B0,      0 },
  {     B50,     50 },
  {     B75,     75 },
  {    B110,    110 },
  {    B134,    134 },
  {    B150,    150 },
  {    B200,    200 },
  {    B300,    300 },
  {    B600,    600 },
  {   B1200,   1200 },
  {   B1800,   1800 },
  {   B2400,   2400 },
  {   B4800,   4800 },
  {   B9600,   9600 },
  {  B19200,  19200 },
  {  B38400,  38400 },
  {  B57600,  57600 },
  { B115200, 115200 },
  { B230400, 230400 },
  { B460800, 460800 }
};

speed_t BaudRateToFlag(unsigned speed) {
  int i;

  for (i = 0; i < NumBaudRates; i++) {
    if (BaudRates[i][1] == speed) return(BaudRates[i][0]);
  }

  return(0);
}

unsigned BaudFlagToRate(speed_t s) {
  int i;

  for (i = 0; i < NumBaudRates; i++) {
    if (BaudRates[i][0] == s) return(BaudRates[i][1]);
  }

  return(0);
}

bool Serial::open(const char *device, int baud, int flags, bool flow_control) {
  static const bool debug = true;
  struct termios tio;
  speed_t bf;
  int baud_in, baud_out;

  // open device
  if (!flags) flags = O_RDWR | O_SYNC | O_NONBLOCK;
  fd = ::open(device, flags);
  if (debug) printf("  dev=%s flags=0x%X fd=%d\n", device, flags, fd);
  if (fd < 0) {
    ::close(fd);
    perror("Error opening serial port");
    return(false);
  }

  // set the parameters we need
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);                      // set up for binary I/O
  tio.c_cflag &= ~(CRTSCTS|IXON|IXOFF);  // disable flow control
  tio.c_cflag &= ~(PARENB|PARODD);  // disable parity
  tio.c_cflag &= ~(CSTOPB);             // only one stop bit

  if (flow_control) {
    // for RoboDragons' radio only
    tio.c_cflag |= CRTSCTS;  // enable flow-control
    // tio.c_cflag |= IGNPAR|PARENB; // enable parity
    // tio.c_cflag |= PARODD; // odd parity (default is even)
  }

  // set serial port speed
  bf = BaudRateToFlag(baud);
  cfsetispeed(&tio, bf);
  cfsetospeed(&tio, bf);
  tcsetattr(fd, TCSANOW, &tio);

  // test what actually got set
  memset(&tio, 0, sizeof(tio));
  tcgetattr(fd, &tio);
  baud_in  = BaudFlagToRate(cfgetispeed(&tio));
  baud_out = BaudFlagToRate(cfgetospeed(&tio));

  if (debug) printf("  baud: %d %d cf:0x%X\n", baud_in, baud_out, tio.c_cflag);

  return(baud_in == baud && baud_out == baud);
}


// bool Serial::ready_for_recv(void)
// {
//  fd_set rfds;
//  struct timeval tv;
//
//  FD_ZERO(&rfds);
//  FD_SET(fd, &rfds);
//  tv.tv_sec = 0; tv.tv_usec = 0;
//  return (select(1 + fd, &rfds, NULL, NULL, &tv));
// }


void Serial::close() {
  if (fd > 0) ::close(fd);
  fd = 0;
}

int Serial::write(const void *buf, int size) {
  int n;

  // tcflush(fd,TCOFLUSH);
  // tcdrain(fd);
  n = ::write(fd, buf, size);
  // tcdrain(fd);

  // printf("Serial::write: n=%d/%d\n",n,size);

  return(n);
}

void Serial::dump(const void *buf, int size) {
  unsigned char *_buf = (unsigned char*)buf;
  printf("%d:", size);
  for (int i = 0; i < size; i++) {
    unsigned char ch = _buf[i];
    if (ch < 32 || ch > 127) ch='-';
    printf(" [%d](%c:%02X)", i, ch, _buf[i]);
  }
  printf("\n");
}

bool Serial::waitForInput(int timeout_msec) {
  pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  poll(&pfd, 1, timeout_msec);

  return(pfd.revents & POLLIN);
}

bool Serial::waitForOutput(int timeout_msec) {
  pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLOUT;
  pfd.revents = 0;

  poll(&pfd, 1, timeout_msec);

  return(pfd.revents & POLLOUT);
}
