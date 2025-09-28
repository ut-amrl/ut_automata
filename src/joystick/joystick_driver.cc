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
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <dirent.h>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "gflags/gflags.h"
#include "joystick/joystick.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"

CONFIG_STRING(joystick_name_, "joystick_name");
CONFIG_STRING(joystick_port_, "joystick_port");

DEFINE_string(config_dir, "/home/orin/roboracer_ws/src/ut_automata/config", "Directory containing joystick.lua config file.");

using sensor_msgs::msg::Joy;
using std::string;
using std::vector;
using joystick::Joystick;

// Function to check if we're running on a Jetson device
bool IsJetsonDevice() {
  std::ifstream tegra_file("/etc/nv_tegra_release");
  if (tegra_file.is_open()) {
    tegra_file.close();
    return true;
  }
  
  std::ifstream device_tree_file("/proc/device-tree/model");
  if (device_tree_file.is_open()) {
    string model;
    std::getline(device_tree_file, model);
    device_tree_file.close();
    std::transform(model.begin(), model.end(), model.begin(), ::tolower);
    return model.find("jetson") != string::npos || 
           model.find("tegra") != string::npos ||
           model.find("nvidia") != string::npos;
  }
  
  return false;
}

// Forward declarations
bool IsBluetoothControllerConnected();
void EnableBluetoothPairingMode();
bool WaitForBluetoothController(int timeout_seconds);

struct JoystickInfo {
  string device_path;
  string name;
  int num_axes;
  int num_buttons;
};

// Function to check if a device name indicates it's likely a real joystick
bool IsLikelyJoystick(const string& name) {
  // Convert to lowercase for comparison
  string lower_name = name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
  
  // List of patterns that indicate non-joystick devices
  vector<string> non_joystick_patterns = {
    "mouse", "touchpad", "trackpad", "keyboard", "synaptics", 
    "elantech", "alps", "bcm", "touchscreen", "touch", "ilitek",
    "wacom", "stylus", "pen"
  };
  
  // Check if the device name contains any non-joystick patterns
  for (const string& pattern : non_joystick_patterns) {
    if (lower_name.find(pattern) != string::npos) {
      return false;
    }
  }
  
  // List of patterns that indicate likely joystick devices
  vector<string> joystick_patterns = {
    "gamepad", "controller", "joystick", "xbox", "playstation", 
    "dualshock", "logitech", "thrustmaster", "saitek", "stick",
    "wireless controller", "sony interactive", "microsoft",
    "nintendo", "switch pro", "joy-con", "dualsense"
  };
  
  // Check if the device name contains any joystick patterns
  for (const string& pattern : joystick_patterns) {
    if (lower_name.find(pattern) != string::npos) {
      return true;
    }
  }
  
  // If no specific patterns matched, consider it potentially a joystick
  // but with lower confidence
  return true;
}

// Function to list all available joystick devices
vector<JoystickInfo> ListAvailableJoysticks() {
  vector<JoystickInfo> joysticks;
  
  // Check /dev/input/js* devices
  for (int i = 0; i < 32; ++i) {
    string device_path = "/dev/input/js" + std::to_string(i);
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) {
      // Try alternative path
      device_path = "/dev/js" + std::to_string(i);
      fd = open(device_path.c_str(), O_RDONLY);
      if (fd < 0) {
        continue;
      }
    }
    
    JoystickInfo info;
    info.device_path = device_path;
    
    char num_axes = 0;
    char num_buttons = 0;
    char name[256];
    
    ioctl(fd, JSIOCGAXES, &num_axes);
    ioctl(fd, JSIOCGBUTTONS, &num_buttons);
    ioctl(fd, JSIOCGNAME(sizeof(name)), name);
    
    info.name = string(name);
    info.num_axes = num_axes;
    info.num_buttons = num_buttons;
    
    close(fd);
    joysticks.push_back(info);
  }
  
  return joysticks;
}

void PrintJoystickList(const vector<JoystickInfo>& joysticks) {
  std::cout << "\n=== Available Input Devices ===" << std::endl;
  std::cout << "Found " << joysticks.size() << " input devices:" << std::endl;
  std::cout << "(Note: Bluetooth joysticks appear here just like USB joysticks)" << std::endl;
  
  for (size_t i = 0; i < joysticks.size(); ++i) {
    bool likely_joystick = IsLikelyJoystick(joysticks[i].name);
    std::cout << "[" << i << "] " << joysticks[i].device_path << std::endl;
    std::cout << "    Name: '" << joysticks[i].name << "'" << std::endl;
    std::cout << "    Axes: " << joysticks[i].num_axes 
              << ", Buttons: " << joysticks[i].num_buttons << std::endl;
    std::cout << "    Likely joystick: " << (likely_joystick ? "YES" : "NO") << std::endl;
    
    // Add hints about common Bluetooth joystick names
    string lower_name = joysticks[i].name;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
    if (lower_name.find("wireless") != string::npos || 
        lower_name.find("bluetooth") != string::npos) {
      std::cout << "    Connection: Likely Bluetooth" << std::endl;
    } else if (lower_name.find("sony") != string::npos && 
               lower_name.find("controller") != string::npos) {
      std::cout << "    Connection: Likely PlayStation controller (USB or Bluetooth)" << std::endl;
    }
    
    std::cout << std::endl;
  }
}

// Function to check if any Bluetooth controllers are connected
bool IsBluetoothControllerConnected() {
  // Check if any connected Bluetooth devices appear to be controllers
  int ret = system("bluetoothctl devices Connected 2>/dev/null | grep -i -E '(controller|gamepad|joystick|xbox|playstation|dualshock|dualsense|joy-con|nintendo|wireless controller)' >/dev/null");
  if (ret == 0) {
    return true;
  }
  
  // Also check if there are any joystick devices that might be Bluetooth
  vector<JoystickInfo> joysticks = ListAvailableJoysticks();
  for (const auto& joy : joysticks) {
    if (IsLikelyJoystick(joy.name)) {
      string lower_name = joy.name;
      std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
      if (lower_name.find("wireless") != string::npos || 
          lower_name.find("bluetooth") != string::npos ||
          lower_name.find("sony") != string::npos ||
          lower_name.find("xbox") != string::npos) {
        return true;
      }
    }
  }
  
  return false;
}

// Function to put system in Bluetooth discoverable/pairable mode
void EnableBluetoothPairingMode() {
  std::cout << "\n=== Bluetooth Controller Setup ===" << std::endl;
  std::cout << "No Bluetooth controller detected. Enabling pairing mode..." << std::endl;
  
  // Enable Bluetooth if it's not already on
  system("bluetoothctl power on 2>/dev/null");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  // Make discoverable and pairable
  system("bluetoothctl discoverable on 2>/dev/null");
  system("bluetoothctl pairable on 2>/dev/null");
  
  std::cout << "Bluetooth adapter is now discoverable and pairable." << std::endl;
  std::cout << "Please put your controller in pairing mode now." << std::endl;
  std::cout << "For PS4/PS5 controllers: Hold Share + PS button until light flashes." << std::endl;
  std::cout << "For Xbox controllers: Hold Xbox + Connect button until light flashes." << std::endl;
  std::cout << "Waiting for controller connection..." << std::endl;
}

// Function to wait for a Bluetooth controller to connect
bool WaitForBluetoothController(int timeout_seconds = 60) {
  auto start_time = std::chrono::steady_clock::now();
  auto timeout_duration = std::chrono::seconds(timeout_seconds);
  
  std::cout << "Scanning for controller connections..." << std::endl;
  
  while (std::chrono::steady_clock::now() - start_time < timeout_duration) {
    if (IsBluetoothControllerConnected()) {
      std::cout << "Bluetooth controller connected successfully!" << std::endl;
      // Give it a moment to create the joystick device
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      return true;
    }
    
    // Print a status update every 10 seconds
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count();
    static int last_printed = -1;
    if (elapsed > 0 && elapsed % 10 == 0 && elapsed != last_printed) {
      std::cout << "Still waiting for controller... (" << elapsed << "s elapsed)" << std::endl;
      last_printed = elapsed;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
  std::cout << "Timeout waiting for Bluetooth controller connection." << std::endl;
  return false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  
  // Check if we're on a Jetson device
  bool is_jetson = IsJetsonDevice();
  if (is_jetson) {
    std::cout << "Running on Jetson device - checking for Bluetooth controllers..." << std::endl;
    
    // Check if a Bluetooth controller is already connected
    if (!IsBluetoothControllerConnected()) {
      std::cout << "No Bluetooth controller connected." << std::endl;
      EnableBluetoothPairingMode();
      
      // Wait for a controller to connect
      if (!WaitForBluetoothController()) {
        std::cout << "Proceeding to check for wired controllers..." << std::endl;
      }
    } else {
      std::cout << "Bluetooth controller already connected." << std::endl;
    }
  }
  
  // List all available joystick devices
  vector<JoystickInfo> available_joysticks = ListAvailableJoysticks();
  PrintJoystickList(available_joysticks);
  
  // Find the first likely joystick device to connect to
  string selected_device = "";
  string selected_name = "";
  for (const auto& joy : available_joysticks) {
    if (IsLikelyJoystick(joy.name)) {
      selected_device = joy.device_path;
      selected_name = joy.name;
      break;
    }
  }
  
  if (selected_device.empty()) {
    std::cout << "ERROR: No suitable joystick devices found!" << std::endl;
    if (is_jetson) {
      std::cout << "Try pairing a Bluetooth controller or connecting a USB controller." << std::endl;
    }
    return 1;
  }
  
  std::cout << "=== Connecting to selected joystick ===" << std::endl;
  std::cout << "Selected device: " << selected_device << std::endl;
  std::cout << "Device name: '" << selected_name << "'" << std::endl;
  
  // Load config.
  config_reader::ConfigReader reader({
    FLAGS_config_dir + "/joystick.lua"
  });
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joystick");
  auto publisher = node->create_publisher<Joy>("joystick",
      rclcpp::QoS(10));
  Joystick joystick = Joystick(joystick_name_);
  
  // Connect to the selected device instead of using config
  if (!joystick.Open(selected_device.c_str())) {
    fprintf(stderr, "ERROR: Unable to open joystick device: %s\n", selected_device.c_str());
    return(1);
  }

  vector<int32_t> buttons;
  vector<float> axes;
  Joy msg;
  msg.header.frame_id = "joystick";

  RateLoop rate_loop(30);
  while (rclcpp::ok()) {
    joystick.ProcessEvents(2);
    joystick.GetAllAxes(&axes);
    joystick.GetAllButtons(&buttons);
    // populate header.stamp from node clock
    {
      uint64_t nsec = node->get_clock()->now().nanoseconds();
      msg.header.stamp.sec = static_cast<int32_t>(nsec / 1000000000ULL);
      msg.header.stamp.nanosec = static_cast<uint32_t>(nsec % 1000000000ULL);
    }
    msg.axes = axes;
    msg.buttons = buttons;
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rate_loop.Sleep();
  }

  joystick.Close();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}

