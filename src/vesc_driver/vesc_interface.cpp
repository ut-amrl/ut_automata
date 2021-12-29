// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_interface.h"

#include <pthread.h>
#include <fcntl.h>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <boost/crc.hpp>

#include "glog/logging.h"

#include "vesc_driver/vesc_packet_factory.h"
#include "serial.h"
#include "shared/util/timer.h"


namespace vesc_driver {

static const bool kDebug = false;
pthread_t rx_thread_;
bool rx_thread_run_;
VescInterface::PacketHandlerFunction packet_handler_;
Serial serial_;

uint8_t read_buffer[4096];

void* rxThread(void*) {
  Buffer buffer;
  buffer.reserve(4096);

  while (rx_thread_run_) {
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer.empty()) {
      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end()) {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
            VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter) {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(
              iter, buffer.end(), &bytes_needed, &error);
          if (packet) {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              fprintf(stderr, 
                  "Out-of-sync with VESC, unknown data leading "   
                  "valid frame. Discarding %ld bytes.\n",
                  std::distance(iter_begin, iter));
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break; // for (iter_sof...
          } else {
            // else, this was not a packet, move on to next byte
          }
        }
        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end()) {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        fprintf(stderr, 
                "Out-of-sync with VESC, unknown data leading "   
                "valid frame. Discarding %ld bytes.\n",
                std::distance(iter_begin, iter));
      }
      buffer.erase(buffer.begin(), iter);
    }

    // attempt to read from the serial port
    serial_.waitForInput(100);
    int bytes_read = serial_.read(read_buffer, sizeof(read_buffer));
    if (bytes_read == -1) {
      perror("Error reading from serial port");
    }
    if (bytes_read > 0) {
      if (kDebug) printf("Bytes read: %d\n", bytes_read);
      for (int i = 0; i < bytes_read; ++i) {
        buffer.push_back(read_buffer[i]);
      }
    }
    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
      fprintf(stderr, 
              "Possibly out-of-sync with VESC, "
              "read timout in the middle of a frame.");
    }
  }
  return NULL;
}


VescInterface::VescInterface(const std::string& port,
                             const PacketHandlerFunction& packet_handler) {
  packet_handler_ = packet_handler;
  // attempt to conect if the port is specified
  if (!port.empty())
    connect(port);
}

VescInterface::~VescInterface() {
  disconnect();
}


bool VescInterface::connect(const std::string& port) {
  // todo - mutex?

  CHECK(!isConnected()) << "Already connected to VESC on port " << port;

  // connect to serial port
  if (!serial_.open(port.c_str(), 115200)) {
    fprintf(stderr, "Failed to open the serial port %s to the VESC\n",
        port.c_str());
    return false;
  }

  // start up a monitoring thread
  rx_thread_run_ = true;
  int result = pthread_create(&rx_thread_,
                              NULL,
                              &rxThread,
                              NULL);
  CHECK_EQ(result, 0) << "pthread_create failed with error code " << result;
  return true;
}

void VescInterface::disconnect() {
  // todo - mutex?

  if (isConnected()) {
    // bring down read thread
    rx_thread_run_ = false;
    int result = pthread_join(rx_thread_, NULL);
    if (result != 0) {
      fprintf(stderr,
              "ERROR: pthread_join failed with error code %d\n",
              result);
      exit(1);
    }
    serial_.close();
  }
}

bool VescInterface::isConnected() const {
  return serial_.isOpen();
}

void VescInterface::send(const VescPacket& packet) {
  const std::vector<uint8_t>& buffer = packet.frame();
  size_t written = serial_.write(buffer.data(), buffer.size());
  if (kDebug) {
    printf("Wrote %d bytes: ", static_cast<int>(written));
    for (size_t i = 0; i < buffer.size(); ++i) {
      printf("0x%X ", buffer[i]);
    }
    printf("\n");
  }
  CHECK_EQ(written, packet.frame().size()) 
      << "Wrote " << written << " bytes, expected " 
      << packet.frame().size() << ".";
}

void VescInterface::requestFWVersion() {
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState() {
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle) {
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current) {
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake) {
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed) {
  send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position) {
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo) {
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
