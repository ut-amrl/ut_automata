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
 * \file    simulator_main.cpp
 * \brief   A simple simulator.
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <iostream>

#include "gflags/gflags.h"
#include "ros/ros.h"

#include "shared/util/timer.h"
#include "simulator/simulator.h"


int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  printf("\nF1/10 Simulator\n\n");

  ros::init(argc, argv, "F1Tenth_Simulator");
  ros::NodeHandle n;

  Simulator simulator;
  simulator.init(n);

  // main loop
  RateLoop rate(40.0);
  while (ros::ok()){
    ros::spinOnce();
    simulator.Run();
    rate.Sleep();
  }

  printf("closing.\n");

  return(0);
}
