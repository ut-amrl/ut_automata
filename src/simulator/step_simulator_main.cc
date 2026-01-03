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
 * \brief   A simple simulator (ROS 2 Version).
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "simulator/simulator.h"  // You will need to port this header and its source file as well.
#include "shared/util/timer.h"    // Assuming this is a custom timer and not ROS-specific.

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto simulator_node = rclcpp::Node::make_shared("ut_automata_simulator");

  // Declare the 'fps' parameter with a default value of 120.0
  simulator_node->declare_parameter<double>("fps", 120.0);
  double fps = simulator_node->get_parameter("fps").as_double();

  RCLCPP_INFO(simulator_node->get_logger(), "\nUT AUTOmata F1/10 Simulator\n");
  RCLCPP_INFO(simulator_node->get_logger(), "Simulator frame rate set to: %.2f", fps);

  Simulator simulator;
  // The Init function now needs to accept a ROS 2 node pointer.
  // Example: simulator.Init(simulator_node);
  simulator.Init(simulator_node);
  simulator.SetStepMode(true);

  // Create a rate object for the main loop
  rclcpp::Rate rate(fps);

  // Main loop
  while (rclcpp::ok()) {
    // Process any pending ROS 2 callbacks
    rclcpp::spin_some(simulator_node);

    // Wait for a character to be pressed
    getchar();

    simulator.RunIteration();
    rate.sleep();
  }

  RCLCPP_INFO(simulator_node->get_logger(), "closing.");

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}