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
#include <errno.h>
#include <string.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "gflags/gflags.h"
#include "joystick/joystick.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"

CONFIG_STRING(joystick_name_, "joystick_name");
CONFIG_STRING(joystick_port_, "joystick_port");

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
string GetPairedControllerMAC();
string GetConnectedControllerMAC();
bool ConnectToController(const string& mac_address);
void SaveControllerMAC(const string& mac_address);
string LoadSavedControllerMAC();
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

// Function to get the MAC address of a paired controller
string GetPairedControllerMAC() {
  FILE* pipe = popen("bluetoothctl devices Paired 2>/dev/null | grep -i -E '(controller|gamepad|joystick|xbox|playstation|dualshock|dualsense|joy-con|nintendo|wireless controller)' | head -n 1 | awk '{print $2}'", "r");
  if (!pipe) return "";
  
  char buffer[128];
  string result = "";
  if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result = buffer;
    // Remove trailing newline
    result.erase(result.find_last_not_of(" \n\r\t") + 1);
  }
  pclose(pipe);
  return result;
}

// Function to get the MAC address of a connected controller
string GetConnectedControllerMAC() {
  FILE* pipe = popen("bluetoothctl devices Connected 2>/dev/null | grep -i -E '(controller|gamepad|joystick|xbox|playstation|dualshock|dualsense|joy-con|nintendo|wireless controller)' | head -n 1 | awk '{print $2}'", "r");
  if (!pipe) return "";
  
  char buffer[128];
  string result = "";
  if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result = buffer;
    // Remove trailing newline
    result.erase(result.find_last_not_of(" \n\r\t") + 1);
  }
  pclose(pipe);
  return result;
}

// Function to connect to a specific controller by MAC address
bool ConnectToController(const string& mac_address) {
  if (mac_address.empty()) return false;
  
  std::cout << "Attempting to connect to saved controller: " << mac_address << std::endl;
  
  // Trust the device to ensure automatic connection
  string trust_cmd = "bluetoothctl trust " + mac_address + " 2>/dev/null";
  system(trust_cmd.c_str());
  
  // Try to connect
  string connect_cmd = "bluetoothctl connect " + mac_address + " 2>/dev/null";
  int ret = system(connect_cmd.c_str());
  
  if (ret == 0) {
    std::cout << "Successfully connected to controller." << std::endl;
    // Disable discoverable mode to prevent other controllers from connecting
    system("bluetoothctl discoverable off 2>/dev/null");
    return true;
  }
  
  return false;
}

// Function to save the controller MAC address to a file
void SaveControllerMAC(const string& mac_address) {
  if (mac_address.empty()) return;
  
  string config_file = string(getenv("HOME")) + "/roboracer_ws/car/joystick_controller_mac.txt";
  std::ofstream file(config_file);
  if (file.is_open()) {
    file << mac_address << std::endl;
    file.close();
    std::cout << "Saved controller MAC address: " << mac_address << std::endl;
  }
}

// Function to load the saved controller MAC address from a file
string LoadSavedControllerMAC() {
  string config_file = string(getenv("HOME")) + "/roboracer_ws/car/joystick_controller_mac.txt";
  std::ifstream file(config_file);
  if (file.is_open()) {
    string mac_address;
    std::getline(file, mac_address);
    file.close();
    // Remove any whitespace
    mac_address.erase(mac_address.find_last_not_of(" \n\r\t") + 1);
    if (!mac_address.empty()) {
      std::cout << "Found saved controller MAC address: " << mac_address << std::endl;
      return mac_address;
    }
  }
  return "";
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
  std::cout << "\nNOTE: Once paired, this computer will remember THIS controller only." << std::endl;
  std::cout << "To pair a different controller, delete: ~/roboracer_ws/car/joystick_controller_mac.txt" << std::endl;
}

// Function to wait for a Bluetooth controller to connect
bool WaitForBluetoothController(int timeout_seconds = 60) {
  auto start_time = std::chrono::steady_clock::now();
  auto timeout_duration = std::chrono::seconds(timeout_seconds);
  
  std::cout << "Scanning for controller connections..." << std::endl;
  
  while (std::chrono::steady_clock::now() - start_time < timeout_duration) {
    if (IsBluetoothControllerConnected()) {
      std::cout << "Bluetooth controller connected successfully!" << std::endl;
      
      // Get the MAC address of the newly connected controller
      string mac_address = GetConnectedControllerMAC();
      if (!mac_address.empty()) {
        // Save it for future use
        SaveControllerMAC(mac_address);
        
        // Trust this controller for automatic reconnection
        string trust_cmd = "bluetoothctl trust " + mac_address + " 2>/dev/null";
        system(trust_cmd.c_str());
        
        // Disable discoverable mode to prevent other controllers from pairing
        system("bluetoothctl discoverable off 2>/dev/null");
        std::cout << "Discoverable mode disabled. This computer will only connect to this controller." << std::endl;
      }
      
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
  // Init rclcpp before the device-wait loop below: that loop spins on
  // rclcpp::ok(), which is false before init, so waiting for a controller
  // used to exit(1) immediately instead of waiting.
  rclcpp::init(argc, argv);

  // Check if we're on a Jetson device
  bool is_jetson = IsJetsonDevice();

  // If a usable joystick device already exists, skip Bluetooth setup: it is
  // unnecessary (wired/dongle controllers, or one paired before launch), and
  // inside a container bluetoothctl is unavailable, so the pairing wait would
  // burn its 60 s timeout before the /dev/input scan below ever runs.
  bool joystick_already_present = false;
  for (const auto& joy : ListAvailableJoysticks()) {
    if (IsLikelyJoystick(joy.name)) {
      joystick_already_present = true;
      break;
    }
  }

  if (is_jetson && !joystick_already_present) {
    std::cout << "Running on Jetson device - checking for Bluetooth controllers..." << std::endl;
    
    // First, check if we have a saved controller MAC address
    string saved_mac = LoadSavedControllerMAC();
    
    if (!saved_mac.empty()) {
      // We have a saved controller - try to connect to it specifically
      std::cout << "This computer is configured to use controller: " << saved_mac << std::endl;
      
      if (!IsBluetoothControllerConnected()) {
        std::cout << "Saved controller not connected. Attempting to reconnect..." << std::endl;
        
        // Try to connect to the saved controller
        if (ConnectToController(saved_mac)) {
          std::cout << "Successfully reconnected to saved controller." << std::endl;
        } else {
          std::cout << "Could not reconnect to saved controller." << std::endl;
          std::cout << "Make sure the controller is powered on and in range." << std::endl;
          std::cout << "To pair a different controller, delete: ~/roboracer_ws/car/joystick_controller_mac.txt" << std::endl;
        }
      } else {
        std::cout << "Saved controller is already connected." << std::endl;
        // Ensure discoverable mode is off to prevent other controllers from connecting
        system("bluetoothctl discoverable off 2>/dev/null");
      }
    } else {
      // No saved controller - check if one is already connected or enable pairing
      if (!IsBluetoothControllerConnected()) {
        std::cout << "No saved controller found and no controller connected." << std::endl;
        EnableBluetoothPairingMode();
        
        // Wait for a controller to connect
        if (!WaitForBluetoothController()) {
          std::cout << "Proceeding to check for wired controllers..." << std::endl;
        }
      } else {
        // A controller is connected but not saved - save it now
        std::cout << "Bluetooth controller already connected." << std::endl;
        string connected_mac = GetConnectedControllerMAC();
        if (!connected_mac.empty()) {
          SaveControllerMAC(connected_mac);
          system("bluetoothctl discoverable off 2>/dev/null");
          std::cout << "Saved this controller for future use." << std::endl;
        }
      }
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
    std::cout << "No suitable joystick devices found. Waiting for connection..." << std::endl;
    if (is_jetson) {
      std::cout << "Try pairing a Bluetooth controller or connecting a USB controller." << std::endl;
    }
    
    // Loop until a device is found
    while (selected_device.empty() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      
      // Re-scan for devices
      available_joysticks = ListAvailableJoysticks();
      
      // Check if we found any likely joysticks
      for (const auto& joy : available_joysticks) {
        if (IsLikelyJoystick(joy.name)) {
          selected_device = joy.device_path;
          selected_name = joy.name;
          break;
        }
      }
      
      if (!selected_device.empty()) {
        std::cout << "Joystick detected!" << std::endl;
        PrintJoystickList(available_joysticks);
      } else {
        // Optional: Print a dot or keep silent to avoid log spam
        // std::cout << "." << std::flush;
      }
    }
  }
  
  std::cout << "=== Connecting to selected joystick ===" << std::endl;
  std::cout << "Selected device: " << selected_device << std::endl;
  std::cout << "Device name: '" << selected_name << "'" << std::endl;
  
  // Load config from the installed package share directory, falling back to
  // the legacy source-checkout path for setups that never ran colcon install.
  string config_dir;
  try {
    config_dir =
        ament_index_cpp::get_package_share_directory("ut_automata") + "/config";
  } catch (const std::exception&) {
    const string home_dir = getenv("HOME") ? string(getenv("HOME")) : "";
    config_dir = home_dir + "/roboracer_ws/src/ut_automata/config";
  }
  config_reader::ConfigReader reader({
    config_dir + "/joystick.lua"
  });
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

