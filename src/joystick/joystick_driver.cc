// Copyright 2017 slane@cs.umass.edu
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

// Joystick Driver main file

#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "gflags/gflags.h"
#include "joystick/joystick.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"

CONFIG_STRING(joystick_name_, "joystick_name");
CONFIG_STRING(joystick_port_, "joystick_port");

DEFINE_string(config_dir, "config", "Directory containing joystick.lua config file.");

using sensor_msgs::Joy;
using std::string;
using std::vector;
using joystick::Joystick;

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Load config.
  config_reader::ConfigReader reader({
    FLAGS_config_dir + "/joystick.lua"
  });
  ros::init(argc, argv, "joystick");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<sensor_msgs::Joy>("joystick", 1);
  Joystick joystick = Joystick(joystick_name_);
  if (!joystick.Open(joystick_port_.c_str())) {
    fprintf(stderr, "ERROR: Unable to open joystick!\n");
    return(1);
  }

  vector<int32_t> buttons;
  vector<float> axes;
  Joy msg;
  msg.header.frame_id = "joystick";
  msg.header.seq = 0;

  RateLoop rate_loop(30);
  while (ros::ok()) {
    joystick.ProcessEvents(2);
    joystick.GetAllAxes(&axes);
    joystick.GetAllButtons(&buttons);
    msg.header.stamp = ros::Time::now();
    msg.axes = axes;
    msg.buttons = buttons;
    publisher.publish(msg);
    rate_loop.Sleep();
  }

  joystick.Close();
  return 0;
}

