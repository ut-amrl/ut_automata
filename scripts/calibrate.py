#!/usr/bin/env python3

"""
VESC Driver Calibration Script

This script calibrates the following parameters:
1. steering_angle_to_servo_offset: Ensures the car drives straight when commanded zero steering
2. steering_angle_to_servo_gain: Maps steering angle commands to actual turn radius
3. speed_to_erpm_offset: Compensates for motor deadband
4. speed_to_erpm_gain: Maps speed commands to actual velocity

Usage:
    ros2 run ut_automata calibrate
    # or
    python3 -m ut_automata.calibrate

Requirements:
    - Car must be on a flat surface with plenty of space
    - Measuring tape or known distance markers
    - Obstacles cleared for turning tests
"""

import rclpy
from rclpy.node import Node
from amrl_msgs.msg import AckermannCurvatureDriveMsg
from nav_msgs.msg import Odometry
from ut_automata.msg import VescStateStamped
from sensor_msgs.msg import Joy
import time
import math
import numpy as np
import os
from pathlib import Path
from geometry_msgs.msg import Twist

# Config files are in the source tree, not install directory
# Use ~/roboracer_ws/src/ut_automata/config/ for config files
CONFIG_DIR = Path(os.path.expanduser("~/roboracer_ws/src/ut_automata/config"))
DEFAULT_CAR_CONFIG = str(CONFIG_DIR / "car.lua")
DEFAULT_VESC_CONFIG = str(CONFIG_DIR / "vesc.lua")

class VESCCalibrator(Node):
    def __init__(self):
        super().__init__('vesc_calibrator')
        
        # Publishers
        self.drive_pub = self.create_publisher(
            AckermannCurvatureDriveMsg, 
            '/ackermann_curvature_drive', 
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.vesc_state_sub = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.vesc_state_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joystick',
            self.joy_callback,
            10
        )
        
        # State variables
        self.current_odom = None
        self.start_odom = None
        self.vesc_state = None
        self.current_joy = None
        self.steering_corrections = []  # Store steering corrections during calibration
        self.recording_corrections = False
        
        # Load max_steering_angle from config for joystick corrections
        self.max_steering_angle, source = self.read_max_steering_angle()
        if source:
            self.get_logger().info(f"Loaded max_steering_angle from {source}: {self.max_steering_angle:.4f} rad")
        else:
            self.get_logger().warn(f"No max_steering_angle found in car.lua or vesc.lua, using default: {self.max_steering_angle:.4f} rad")
        
        # Calibration results
        self.results = {
            'steering_offset': None,
            'steering_gain': None,
            'speed_offset': None,
            'speed_gain': None,
            'max_steering_angle': None
        }
        
        self.get_logger().info('VESC Calibrator initialized')
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def vesc_state_callback(self, msg):
        self.vesc_state = msg
        
    def joy_callback(self, msg):
        self.current_joy = msg
        
        # Record steering corrections during calibration
        if self.recording_corrections and len(msg.axes) > 0:
            # Use left stick horizontal (axes[0]) for steering, same as vesc_driver
            # Negative because that's how vesc_driver does it
            steer_input = -msg.axes[0] if len(msg.axes) > 0 else 0.0
            # Scale by max steering angle from config (default 0.286 rad)
            max_steering_angle = getattr(self, 'max_steering_angle', 0.286)
            steering_correction = steer_input * max_steering_angle
            self.steering_corrections.append(steering_correction)
            
    def send_drive_command(self, velocity, curvature):
        """Send drive command to the car"""
        msg = AckermannCurvatureDriveMsg()
        msg.velocity = velocity
        msg.curvature = curvature
        self.drive_pub.publish(msg)
        
    def stop(self):
        """Stop the car"""
        self.send_drive_command(0.0, 0.0)
        time.sleep(0.5)
        
    def wait_for_data(self):
        """Wait until we receive odometry and VESC state data"""
        self.get_logger().info('Waiting for sensor data...')
        rate = self.create_rate(10)
        while rclpy.ok():
            if self.current_odom is not None and self.vesc_state is not None:
                self.get_logger().info('Sensor data received!')
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        
    def get_position(self):
        """Get current position from odometry"""
        if self.current_odom is None:
            return None
        return (
            self.current_odom.pose.pose.position.x,
            self.current_odom.pose.pose.position.y
        )
        
    def get_yaw(self):
        """Get current yaw from odometry"""
        if self.current_odom is None:
            return None
        q = self.current_odom.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def get_distance_traveled(self):
        """Calculate distance traveled since start"""
        if self.start_odom is None or self.current_odom is None:
            return 0.0
        x0, y0 = self.start_odom
        x1, y1 = self.get_position()
        return math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    
    def read_max_steering_angle(self, car_config_path=None,
                                vesc_config_path=None,
                                wheelbase=0.324, verbose=False):
        """Read max_steering_angle from car.lua (or vesc.lua as fallback)
        
        Args:
            car_config_path: Path to car.lua config file (defaults to ../config/car.lua)
            vesc_config_path: Path to vesc.lua config file (defaults to ../config/vesc.lua)
            wheelbase: Wheelbase in meters (for computing turn radius)
            verbose: If True, print detailed information about loaded value
            
        Returns:
            tuple: (max_steering_angle in radians, source filename or None)
        """
        import re
        
        if car_config_path is None:
            car_config_path = DEFAULT_CAR_CONFIG
        if vesc_config_path is None:
            vesc_config_path = DEFAULT_VESC_CONFIG
        
        max_steering_angle = None
        source = None
        
        config_paths = [car_config_path, vesc_config_path]
        
        for config_path in config_paths:
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
                match = re.search(r'max_steering_angle\s*=\s*([-0-9.]+)', content)
                if match:
                    max_steering_angle = float(match.group(1))
                    source = config_path.split('/')[-1]  # Get filename only
                    break
            except FileNotFoundError:
                continue
            except Exception as e:
                if verbose:
                    print(f"Warning: Error reading {config_path}: {e}")
                else:
                    self.get_logger().warn(f"Error reading {config_path}: {e}")
                continue
        
        # Use default if not found
        if max_steering_angle is None:
            # default for ~0.75m turn radius
            # this should match the vesc.lua default
            # note this is intentionally allows a tighter turn radius than the planner's default 0.8m
            # this is to ensure the car can make turns within the planner's default 0.8m turn radius even after calibration
            max_steering_angle = 0.425
            source = None
        
        # Print information if verbose
        if verbose:
            if source:
                print(f"✓ Loaded max_steering_angle from {source}:")
                print(f"  {max_steering_angle:.4f} rad ({math.degrees(max_steering_angle):.2f}°)")
                print(f"  Corresponds to ~{wheelbase / math.tan(max_steering_angle):.2f}m turning radius")
            else:
                print(f"⚠️  No max_steering_angle found in car.lua or vesc.lua")
                print(f"   Using default: {max_steering_angle:.4f} rad ({math.degrees(max_steering_angle):.2f}°)")
                print(f"   This corresponds to ~{wheelbase / math.tan(max_steering_angle):.2f}m turning radius")
        
        return max_steering_angle, source
    
    def read_vesc_config(self, car_config_path=None,
                         vesc_config_path=None):
        """Read steering offset and gain from car.lua (or vesc.lua as fallback) config file"""
        import re
        
        if car_config_path is None:
            car_config_path = DEFAULT_CAR_CONFIG
        if vesc_config_path is None:
            vesc_config_path = DEFAULT_VESC_CONFIG
        
        offset = 0.5  # default
        gain = -0.9015  # default
        
        # Try car.lua first, then vesc.lua as fallback
        config_paths = [car_config_path, vesc_config_path]
        found_offset = False
        found_gain = False
        
        for config_path in config_paths:
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
                    
                # Match steering_angle_to_servo_offset = value; (with optional semicolon and comment)
                if not found_offset:
                    offset_match = re.search(r'steering_angle_to_servo_offset\s*=\s*([0-9.]+)', content)
                    if offset_match:
                        offset = float(offset_match.group(1))
                        found_offset = True
                    
                # Match steering_angle_to_servo_gain = value; (with optional semicolon and comment)
                if not found_gain:
                    gain_match = re.search(r'steering_angle_to_servo_gain\s*=\s*([-0-9.]+)', content)
                    if gain_match:
                        gain = float(gain_match.group(1))
                        found_gain = True
                    
                # If we found both values, we're done
                if found_offset and found_gain:
                    break
                    
            except FileNotFoundError:
                continue  # Try next config file
            except Exception as e:
                self.get_logger().warn(f"Error reading config file {config_path}: {e}")
                continue
        
        if offset == 0.5 and gain == -0.9015:
            self.get_logger().warn(f"Could not read config from {car_config_path} or {vesc_config_path}, using defaults")
            
        return offset, gain
    
    def read_speed_config(self, car_config_path='default',
                          vesc_config_path='default'):
        """Read speed offset and gain from car.lua (or vesc.lua as fallback) config file
        
        Args:
            car_config_path: Path to car.lua, 'default' for default path, or None to skip
            vesc_config_path: Path to vesc.lua, 'default' for default path, or None to skip
        """
        import re
        
        if car_config_path == 'default':
            car_config_path = DEFAULT_CAR_CONFIG
        if vesc_config_path == 'default':
            vesc_config_path = DEFAULT_VESC_CONFIG
        
        offset = 180.0  # default
        gain = 5356.0  # default
        
        # Build list of config paths to check (skip None values)
        config_paths = []
        if car_config_path is not None:
            config_paths.append(car_config_path)
        if vesc_config_path is not None:
            config_paths.append(vesc_config_path)
        
        found_offset = False
        found_gain = False
        
        for config_path in config_paths:
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
                    
                # Match speed_to_erpm_offset = value; (with optional semicolon and comment)
                if not found_offset:
                    offset_match = re.search(r'speed_to_erpm_offset\s*=\s*([-0-9.]+)', content)
                    if offset_match:
                        offset = float(offset_match.group(1))
                        found_offset = True
                    
                # Match speed_to_erpm_gain = value; (with optional semicolon and comment)
                if not found_gain:
                    gain_match = re.search(r'speed_to_erpm_gain\s*=\s*([-0-9.]+)', content)
                    if gain_match:
                        gain = float(gain_match.group(1))
                        found_gain = True
                    
                # If we found both values, we're done
                if found_offset and found_gain:
                    break
                    
            except FileNotFoundError:
                continue  # Try next config file
            except Exception as e:
                self.get_logger().warn(f"Error reading config file {config_path}: {e}")
                continue
        
        if offset == 180.0 and gain == 5356.0 and len(config_paths) > 0:
            paths_str = " or ".join([p for p in [car_config_path, vesc_config_path] if p is not None])
            self.get_logger().warn(f"Could not read speed config from {paths_str}, using defaults")
            
        return offset, gain
    
    def write_steering_offset(self, offset, config_path=None):
        """Write steering offset to car.lua config file"""
        import re
        
        if config_path is None:
            config_path = DEFAULT_CAR_CONFIG
        
        try:
            # Read existing content or create new file
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
            except FileNotFoundError:
                # Create new file with car_name if it doesn't exist
                content = 'car_name = "orin12";\n\n'
            
            # Check if steering_angle_to_servo_offset already exists in the file
            pattern = r'(steering_angle_to_servo_offset\s*=\s*)([0-9.]+)([;]?[^\n]*)'
            if re.search(pattern, content):
                # Replace existing value (preserve semicolon and comments)
                replacement = f'\\g<1>{offset:.4f}\\g<3>'
                new_content = re.sub(pattern, replacement, content)
            else:
                # Append to file if it doesn't exist
                new_content = content.rstrip() + f'\nsteering_angle_to_servo_offset = {offset:.4f}; -- should be between 0.4-0.6\n'
            
            with open(config_path, 'w') as f:
                f.write(new_content)
                
            self.get_logger().info(f"Updated steering_offset to {offset:.4f} in {config_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing config file: {e}")
            return False
    
    def write_steering_gain(self, gain, config_path=None):
        """Write steering gain to car.lua config file"""
        import re
        
        if config_path is None:
            config_path = DEFAULT_CAR_CONFIG
        
        try:
            # Read existing content or create new file
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
            except FileNotFoundError:
                # Create new file with car_name if it doesn't exist
                content = 'car_name = "orin12";\n\n'
            
            # Check if steering_angle_to_servo_gain already exists in the file
            pattern = r'(steering_angle_to_servo_gain\s*=\s*)([-0-9.]+)([;]?[^\n]*)'
            if re.search(pattern, content):
                # Replace existing value (preserve semicolon and comments)
                replacement = f'\\g<1>{gain:.4f}\\g<3>'
                new_content = re.sub(pattern, replacement, content)
            else:
                # Append to file if it doesn't exist
                new_content = content.rstrip() + f'\nsteering_angle_to_servo_gain = {gain:.4f};\n'
            
            with open(config_path, 'w') as f:
                f.write(new_content)
                
            self.get_logger().info(f"Updated steering_gain to {gain:.4f} in {config_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing config file: {e}")
            return False
    
    def write_speed_offset(self, offset, config_path=None):
        """Write speed offset to car.lua config file"""
        import re
        
        if config_path is None:
            config_path = DEFAULT_CAR_CONFIG
        
        try:
            # Read existing content or create new file
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
            except FileNotFoundError:
                # Create new file with car_name if it doesn't exist
                content = 'car_name = "orin12";\n\n'
            
            # Check if speed_to_erpm_offset already exists in the file
            pattern = r'(speed_to_erpm_offset\s*=\s*)([-0-9.]+)([;]?[^\n]*)'
            if re.search(pattern, content):
                # Replace existing value (preserve semicolon and comments)
                replacement = f'\\g<1>{offset:.1f}\\g<3>'
                new_content = re.sub(pattern, replacement, content)
            else:
                # Append to file if it doesn't exist
                new_content = content.rstrip() + f'\nspeed_to_erpm_offset = {offset:.1f}; -- should be between 160-200\n'
            
            with open(config_path, 'w') as f:
                f.write(new_content)
                
            self.get_logger().info(f"Updated speed_to_erpm_offset to {offset:.1f} in {config_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing config file: {e}")
            return False
    
    def write_speed_gain(self, gain, config_path=None):
        """Write speed gain to car.lua config file"""
        import re
        
        if config_path is None:
            config_path = DEFAULT_CAR_CONFIG
        
        try:
            # Read existing content or create new file
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
            except FileNotFoundError:
                # Create new file with car_name if it doesn't exist
                content = 'car_name = "orin12";\n\n'
            
            # Check if speed_to_erpm_gain already exists in the file
            pattern = r'(speed_to_erpm_gain\s*=\s*)([-0-9.]+)([;]?[^\n]*)'
            if re.search(pattern, content):
                # Replace existing value (preserve semicolon and comments)
                replacement = f'\\g<1>{gain:.1f}\\g<3>'
                new_content = re.sub(pattern, replacement, content)
            else:
                # Append to file if it doesn't exist
                new_content = content.rstrip() + f'\nspeed_to_erpm_gain = {gain:.1f};\n'
            
            with open(config_path, 'w') as f:
                f.write(new_content)
                
            self.get_logger().info(f"Updated speed_to_erpm_gain to {gain:.1f} in {config_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing config file: {e}")
            return False
    
    def write_max_steering_angle(self, angle, config_path=None):
        """Write max_steering_angle to car.lua config file"""
        import re
        
        if config_path is None:
            config_path = DEFAULT_CAR_CONFIG
        
        try:
            # Read existing content or create new file
            try:
                with open(config_path, 'r') as f:
                    content = f.read()
            except FileNotFoundError:
                # Create new file with car_name if it doesn't exist
                content = 'car_name = "orin12";\n\n'
            
            # Check if max_steering_angle already exists in the file
            pattern = r'(max_steering_angle\s*=\s*)([-0-9.]+)([;]?[^\n]*)'
            if re.search(pattern, content):
                # Replace existing value (preserve semicolon and comments)
                replacement = f'\\g<1>{angle:.4f}\\g<3>'
                new_content = re.sub(pattern, replacement, content)
            else:
                # Append to file if it doesn't exist
                new_content = content.rstrip() + f'\nmax_steering_angle = {angle:.4f}; -- radians, calibrated from turn radius\n'
            
            with open(config_path, 'w') as f:
                f.write(new_content)
                
            self.get_logger().info(f"Updated max_steering_angle to {angle:.4f} rad ({math.degrees(angle):.2f} deg) in {config_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing config file: {e}")
            return False
        
    def calibrate_steering_offset(self):
        """
        Calibrate steering offset by letting user correct drift with joystick.
        The average correction is used to adjust the offset.
        """
        print("\n" + "="*60)
        print("STEP 1: CALIBRATING STEERING OFFSET")
        print("="*60)
        print("\nThis test will help find the servo offset that makes the car")
        print("drive straight when commanded zero steering angle.")
        print("\nThe car will drive forward at 0.5 m/s for 10 seconds.")
        print("Use the LEFT JOYSTICK (horizontal) to keep the car going straight.")
        print("The script will measure how much correction you apply.")
        print("\nPress ENTER to start the test (or 'q' to skip)...")
        
        response = input()
        if response.lower() == 'q':
            print("Skipping steering offset calibration")
            return
        
        # Read current offset and gain from car.lua (or vesc.lua as fallback) config file
        current_offset, current_gain = self.read_vesc_config()
        print(f"\n📋 Current configuration:")
        print(f"   Steering offset: {current_offset:.4f}")
        print(f"   Steering gain: {current_gain:.4f}")
        
        iteration = 0
        max_iterations = 10
        
        print("\n TIP: Use small, smooth joystick movements to keep the car straight.")
        
        while iteration < max_iterations:
            iteration += 1
            self.steering_corrections = []  # Clear previous corrections
            
            print(f"\n--- Iteration {iteration}: Testing with servo offset: {current_offset:.4f} ---")
            print("Starting in 3 seconds... Hold R1 to allow the car to move")
            time.sleep(3)
            
            # Start recording corrections
            self.recording_corrections = True
            
            # Drive straight
            self.start_odom = self.get_position()
            start_time = time.time()
            
            while time.time() - start_time < 10.0:
                print(f"Driving forward at 0.5 m/s for 10 seconds")
                # Get joystick steering input
                steering_correction = 0.0
                if self.current_joy is not None and len(self.current_joy.axes) > 0:
                    # Use left stick horizontal (axes[0]) for steering
                    steer_input = -self.current_joy.axes[0]
                    # Scale by max steering angle from config
                    steering_correction = steer_input * self.max_steering_angle
                
                # Command forward with user's steering correction
                # curvature = 1/turn_radius, for small angles: curvature ≈ steering_angle / wheelbase
                wheelbase = 0.324  # from vesc.lua
                curvature = steering_correction / wheelbase if wheelbase > 0 else 0.0
                
                self.send_drive_command(0.5, curvature)
                rclpy.spin_once(self, timeout_sec=0.02)
            
            # Stop recording corrections
            self.recording_corrections = False
            self.stop()
            
            # Calculate average steering correction
            if len(self.steering_corrections) > 0:
                avg_correction = np.mean(self.steering_corrections)
                std_correction = np.std(self.steering_corrections)
                max_correction = np.max(np.abs(self.steering_corrections))
                
                print(f"\n📊 Steering Correction Analysis:")
                print(f"   Average correction: {avg_correction:+.4f} rad ({math.degrees(avg_correction):+.2f}°)")
                print(f"   Std deviation: {std_correction:.4f} rad ({math.degrees(std_correction):.2f}°)")
                print(f"   Max correction: {max_correction:.4f} rad ({math.degrees(max_correction):.2f}°)")
                print(f"   Samples collected: {len(self.steering_corrections)}")
            else:
                print("\n⚠️  No steering corrections detected!")
                print("   Make sure the joystick is connected and you're moving the left stick.")
                response = input("\nRetry this iteration? (y/n): ").strip().lower()
                if response == 'y':
                    iteration -= 1  # Don't count this iteration
                    continue
                else:
                    break
            
            # Check if correction is small enough (within 0.01 radians = ~0.6 degrees)
            if abs(avg_correction) < 0.01:
                # Write the final offset to car.lua config file
                if self.write_steering_offset(current_offset):
                    print(f"\n✅ Updated car.lua with final offset: {current_offset:.4f}")
                self.results['steering_offset'] = current_offset
                print(f"\n✅ Steering offset calibrated: {current_offset:.4f}")
                print(f"   Average correction is now within acceptable range ({math.degrees(avg_correction):.2f}°)")
                print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
                print("   You can do this by:")
                print("   1. Finding which tmux pane contains the vesc_driver.")
                print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
                break
            
            # Calculate new offset based on average correction
            # servo = gain * angle + offset
            # If user applies positive correction, servo needs to shift by (gain * correction)
            offset_adjustment = current_gain * avg_correction
            new_offset = current_offset + offset_adjustment
            
            # Clamp offset to reasonable range (0.3 to 0.7)
            new_offset = max(0.3, min(0.7, new_offset))
            
            print(f"\n🔧 Adjustment:")
            print(f"   Old offset: {current_offset:.4f}")
            print(f"   Offset change: {offset_adjustment:+.4f}")
            print(f"   New offset: {new_offset:.4f}")
            
            current_offset = new_offset
            
            # Write the updated offset to car.lua config file
            if self.write_steering_offset(current_offset):
                print(f"\n✅ Updated car.lua with new offset: {current_offset:.4f}")
                print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
                print("   You can do this by:")
                print("   1. Finding which tmux pane contains the vesc_driver.")
                print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
                print("\n   Press ENTER after restarting the VESC driver to continue...")
                input()
            else:
                print("\n⚠️  Failed to update car.lua. Please update it manually.")
                print(f"   Set steering_angle_to_servo_offset = {current_offset:.4f};")
            
            # Ask user if they want to continue or accept current value
            print("\nOptions:")
            print("  1. Run another test iteration with the adjusted offset")
            print("  2. Accept current offset and continue")
            print("  q. Quit steering offset calibration")
            
            response = input("\nEnter choice (1/2/q): ").strip()
            
            if response == '2':
                # Write the accepted offset to car.lua config file
                if self.write_steering_offset(current_offset):
                    print(f"\n✅ Updated car.lua with accepted offset: {current_offset:.4f}")
                self.results['steering_offset'] = current_offset
                print(f"\n✓ Steering offset accepted: {current_offset:.4f}")
                print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
                print("   You can do this by:")
                print("   1. Finding which tmux pane contains the vesc_driver.")
                print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
                break
            elif response.lower() == 'q':
                print("Steering offset calibration cancelled")
                break
            # Otherwise continue with next iteration (response == '1' or invalid)
        
        if iteration >= max_iterations:
            # Write the final offset to car.lua config file
            if self.write_steering_offset(current_offset):
                print(f"\n✅ Updated car.lua with final offset: {current_offset:.4f}")
            print(f"\n️ Maximum iterations ({max_iterations}) reached.")
            print(f"   Using best offset found: {current_offset:.4f}")
            self.results['steering_offset'] = current_offset
            print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
            print("   You can do this by:")
            print("   1. Finding which tmux pane contains the vesc_driver.")
            print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
                
    def calibrate_steering_gain(self):
        """
        Calibrate steering gain by commanding full lock turns and measuring radius.
        Uses the bicycle model: steering_angle = atan(wheelbase / turn_radius)
        """
        print("\n" + "="*60)
        print("STEP 2: CALIBRATING STEERING GAIN")
        print("="*60)
        print("\nThis test will measure the turning radius at maximum steering.")
        print("The car will make a full left turn, then a full right turn.")
        print("\nYou will need:")
        print("  - A measuring tape")
        print("  - Space for the car to make a complete circle")
        print("\nPress ENTER to start (or 'q' to skip)...")
        
        response = input()
        if response.lower() == 'q':
            print("Skipping steering gain calibration")
            return
        
        wheelbase = 0.324  # meters (from vesc.lua)
        
        # Read current max_steering_angle from config (try car.lua first, then vesc.lua)
        max_steering_angle, source = self.read_max_steering_angle(wheelbase=wheelbase, verbose=True)
        
        # Calculate maximum curvature from max steering angle
        # From vesc_driver: steering_angle = atan(wheelbase / turn_radius)
        # turn_radius = velocity / (velocity * curvature) = 1 / curvature
        # So: steering_angle = atan(wheelbase * curvature)
        # For max steering: curvature = tan(max_steering_angle) / wheelbase
        max_curvature = math.tan(max_steering_angle) / wheelbase
        
        # Test both left and right turns
        turn_radii = []
        
        for direction, curvature in [('LEFT', max_curvature), ('RIGHT', -max_curvature)]:
            print(f"\n--- {direction} TURN TEST ---")
            print(f"The car will make a {direction.lower()} turn.")
            print("After the test, measure the diameter of the circle it traced.")
            print("\nPress ENTER when ready...")
            input()
            
            print("Starting in 3 seconds... Hold R1 to allow the car to move")
            time.sleep(3)
            
            # Record starting position and yaw
            start_pos = self.get_position()
            start_yaw = self.get_yaw()
            
            # Drive in a circle until we complete ~360 degrees or 10 seconds
            start_time = time.time()
            max_duration = 15.0
            
            while time.time() - start_time < max_duration:
                self.send_drive_command(1.0, curvature)
                rclpy.spin_once(self, timeout_sec=0.02)
                
                # Check if we've completed the circle (yaw changed by ~2*pi)
                current_yaw = self.get_yaw()
                yaw_change = abs(current_yaw - start_yaw)
                if yaw_change > 2 * math.pi * 0.9:  # 90% of full circle
                    break
                    
            self.stop()
            
            print("\nTest complete!")
            print("Please measure the DIAMETER (width) of the circle the car traced.")
            print("Enter the diameter in METERS (e.g., 2.5 for 2.5 meters):")
            
            while True:
                try:
                    diameter = float(input("Diameter (m): "))
                    radius = diameter / 2.0
                    turn_radii.append(radius)
                    print(f"Recorded turn radius: {radius:.3f} m")
                    break
                except ValueError:
                    print("Invalid input. Please enter a number.")
        
        if len(turn_radii) == 0:
            print("No turn measurements recorded")
            return
        
        # Calculate average turn radius
        avg_radius = np.mean(turn_radii)
        print(f"\n📊 Turn Radius Measurements:")
        print(f"   Average turn radius: {avg_radius:.3f} m")
        if len(turn_radii) > 1:
            print(f"   Individual radii: {', '.join([f'{r:.3f}m' for r in turn_radii])}")
            print(f"   Variation: ±{np.std(turn_radii):.3f} m")
        
        # Calculate actual steering angle from measured radius
        # steering_angle = atan(wheelbase / turn_radius)
        actual_steering_angle = math.atan(wheelbase / avg_radius)
        
        # This is the max_steering_angle the vehicle can physically achieve
        measured_max_steering_angle = actual_steering_angle
        
        # Calculate gain: commanded_angle / actual_angle gives us the gain
        # But we command curvature, so we need to work backwards
        # curvature = 1/turn_radius, steering_angle = atan(wheelbase * curvature)
        # The servo value = gain * steering_angle + offset
        # For max curvature (0.25), the steering angle should be max_steering_angle
        
        # The gain relates servo position to steering angle
        # servo = gain * angle + offset
        # At max deflection: servo_max = gain * max_angle + offset
        # We know the relationship: measured angle corresponds to a curvature command
        
        # From the driver: steering_angle = atan(wheelbase / (velocity / curvature))
        # But when velocity = curvature * turn_radius, we get: steering_angle = atan(wheelbase / turn_radius)
        
        # Use the maximum curvature we commanded
        commanded_curvature = max_curvature
        # The commanded steering angle for this curvature at 1 m/s:
        # turn_radius_commanded = 1.0 / max_curvature
        # steering_angle_commanded = atan(wheelbase / turn_radius_commanded)
        # Or directly: steering_angle = atan(wheelbase * curvature)
        commanded_steering_angle = math.atan(wheelbase * commanded_curvature)
        
        print(f"\nCommanded steering angle: {commanded_steering_angle:.4f} rad ({math.degrees(commanded_steering_angle):.2f}°)")
        print(f"Actual steering angle: {actual_steering_angle:.4f} rad ({math.degrees(actual_steering_angle):.2f}°)")
        
        # The current gain from vesc.lua
        current_gain = -0.9015
        
        # Calculate correction factor
        # If actual < commanded, car turns tighter → servo is moving more → gain magnitude is too high
        # If actual > commanded, car turns wider → servo is moving less → gain magnitude is too low
        correction_factor = commanded_steering_angle / actual_steering_angle
        new_gain = current_gain * correction_factor
        
        self.results['steering_gain'] = new_gain
        self.results['max_steering_angle'] = measured_max_steering_angle
        
        # Write the gain to car.lua config file
        if self.write_steering_gain(new_gain):
            print(f"\n✅ Updated car.lua with steering gain: {new_gain:.4f}")
        else:
            print(f"\n⚠️  Failed to update car.lua. Please update it manually.")
            print(f"   Set steering_angle_to_servo_gain = {new_gain:.4f};")
        
        # Write max_steering_angle to car.lua config file
        if self.write_max_steering_angle(measured_max_steering_angle):
            print(f"✅ Updated car.lua with max_steering_angle: {measured_max_steering_angle:.4f} rad ({math.degrees(measured_max_steering_angle):.2f}°)")
            print(f"   This corresponds to minimum turning radius: {avg_radius:.3f} m")
        else:
            print(f"\n⚠️  Failed to update car.lua. Please update it manually.")
            print(f"   Set max_steering_angle = {measured_max_steering_angle:.4f};")
        
        print(f"\n✓ Steering gain calibrated: {new_gain:.4f}")
        print(f"  (Correction factor: {correction_factor:.3f})")
        print(f"✓ Max steering angle calibrated: {measured_max_steering_angle:.4f} rad ({math.degrees(measured_max_steering_angle):.2f}°)")
        print(f"  Minimum turning radius: {avg_radius:.3f} m")
        print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
        print("   You can do this by:")
        print("   1. Finding which tmux pane contains the vesc_driver.")
        print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
        
    def calibrate_speed_offset(self):
        """
        Calibrate speed offset by finding the minimum speed needed to move the car.
        User holds R1 button and releases it when the car starts moving.
        """
        print("\n" + "="*60)
        print("STEP 3: CALIBRATING SPEED OFFSET")
        print("="*60)
        print("\nThis test finds the minimum motor command needed to overcome")
        print("friction and make the car start moving.")
        print("\nPress ENTER to start (or 'q' to skip)...")
        
        response = input()
        if response.lower() == 'q':
            print("Skipping speed offset calibration")
            return
        
        print("\n" + "="*60)
        print("INSTRUCTIONS:")
        print("="*60)
        print("1. HOLD the R1 button (right shoulder button)")
        print("2. The motor power will gradually increase")
        print("3. Watch the car carefully")
        print("4. RELEASE R1 as soon as the car starts moving")
        print("5. The script will record the speed offset at that moment")
        print("="*60)
        print("\nPress ENTER when ready...")
        input()
        
        # Wait for R1 to be pressed
        print("\nWaiting for you to press and hold R1...")
        r1_pressed = False
        while not r1_pressed:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joy is not None and len(self.current_joy.buttons) > 5:
                if self.current_joy.buttons[5] == 1:  # R1 button (kAutonomousDriveButton)
                    r1_pressed = True
                    print("✓ R1 detected! Starting speed ramp...")
                    break
        
        # Start with low ERPM and gradually increase
        current_erpm = 100
        erpm_step = 10  # Smaller steps for finer control
        max_erpm = 500
        threshold_erpm = None
        
        print("\nGradually increasing motor power...")
        print("RELEASE R1 when the car starts moving!\n")
        
        start_time = time.time()
        
        while current_erpm <= max_erpm:
            # Check if R1 is still pressed
            rclpy.spin_once(self, timeout_sec=0.02)
            
            if self.current_joy is None or len(self.current_joy.buttons) <= 5:
                print("\n⚠️  Joystick not detected. Stopping test.")
                break
            
            # If R1 is released, capture the current ERPM
            if self.current_joy.buttons[5] == 0:  # R1 released
                threshold_erpm = current_erpm
                print(f"\n✓ R1 RELEASED! Captured speed offset at ERPM: {current_erpm}")
                break
            
            # Display current ERPM every second
            elapsed = time.time() - start_time
            if int(elapsed * 2) % 2 == 0:  # Print twice per second
                print(f"  Current ERPM: {current_erpm} (Hold R1, release when car moves)", end='\r')
            
            # Apply motor command
            # Scale speed command with current ERPM test value
            test_speed = 0.1 * (current_erpm / 200.0)
            self.send_drive_command(test_speed, 0.0)
            
            # Gradually increase ERPM
            current_erpm += erpm_step
            time.sleep(0.5)  # Wait 0.5 seconds between increments
        
        # Stop the car
        self.stop()
        rclpy.spin_once(self, timeout_sec=0.1)
        
        if threshold_erpm is None:
            print("\n⚠️  Could not determine speed offset (R1 not released or max ERPM reached).")
            print("Using default: 180.0")
            self.results['speed_offset'] = 180.0
        else:
            # Use the captured ERPM as the offset
            self.results['speed_offset'] = float(threshold_erpm)
            print(f"\n✓ Speed offset calibrated: {self.results['speed_offset']:.1f}")
            print(f"  This is the minimum ERPM needed to overcome friction.")
            
            # Write the offset to car.lua config file
            if self.write_speed_offset(self.results['speed_offset']):
                print(f"\n✅ Updated car.lua with speed offset: {self.results['speed_offset']:.1f}")
            else:
                print(f"\n⚠️  Failed to update car.lua. Please update it manually.")
                print(f"    Add this line: speed_to_erpm_offset = {self.results['speed_offset']:.1f};")

            print("⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
            print("   You can do this by:")
            print("   1. Finding which tmux pane contains the vesc_driver.")
            print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
            print("\n   Press ENTER after restarting the VESC driver to continue...")
            input()
            
    def calibrate_speed_gain(self):
        """
        Calibrate speed gain by driving a known distance at a constant speed
        and measuring the actual velocity.
        """
        print("\n" + "="*60)
        print("STEP 4: CALIBRATING SPEED GAIN")
        print("="*60)
        print("\nThis test measures the relationship between speed commands")
        print("and actual velocity.")
        
        # Check if current speed config values are reasonable
        current_offset, current_gain = self.read_speed_config()
        print(f"\n📋 Current speed configuration:")
        print(f"   speed_to_erpm_offset: {current_offset:.1f}")
        print(f"   speed_to_erpm_gain: {current_gain:.1f}")
        
        # Warn if the gain seems way off (should be ~5000-6000 for typical RC car)
        if current_gain < 100 or current_gain > 10000:
            print(f"\n⚠️  WARNING: Current speed_to_erpm_gain ({current_gain:.1f}) seems incorrect!")
            print(f"   Expected range: 5000-6000 for typical RC car")
            print(f"   This will cause incorrect speed commands during calibration.")
            print(f"\n   The vesc.lua defaults are:")
            # Read only from vesc.lua (skip car.lua)
            vesc_offset, vesc_gain = self.read_speed_config(car_config_path=None, vesc_config_path='default')
            print(f"     speed_to_erpm_offset: {vesc_offset:.1f}")
            print(f"     speed_to_erpm_gain: {vesc_gain:.1f}")
            print(f"\n   Options:")
            print(f"   1. Use vesc.lua defaults for this calibration (recommended)")
            print(f"   2. Continue with current values from car.lua")
            print(f"   q. Skip speed gain calibration")
            
            choice = input("\n   Enter choice (1/2/q): ").strip()
            if choice == '1':
                print(f"\n   ⚠️  You need to fix car.lua BEFORE continuing:")
                print(f"   1. Comment out or delete these lines from car.lua:")
                print(f"      # speed_to_erpm_offset = {current_offset:.1f};")
                print(f"      # speed_to_erpm_gain = {current_gain:.1f};")
                print(f"   2. Restart the VESC driver node so it uses vesc.lua defaults")
                print(f"   3. Then come back and press ENTER to continue calibration")
                print(f"\n   Press ENTER after fixing car.lua and restarting the VESC driver...")
                input()
                # Store the vesc.lua offset for use in gain calculation
                self.results['speed_offset'] = vesc_offset
                print(f"\n   ✓ Will use offset {vesc_offset:.1f} from vesc.lua for gain calculation")
            elif choice.lower() == 'q':
                print("Skipping speed gain calibration")
                return
            # else continue with current values (choice == '2' or invalid)
        
        print("\nYou will need:")
        print("  - A straight path of at least 5 meters")
        print("  - Markers at known distances (e.g., every 1 meter)")
        print("\nPress ENTER to start (or 'q' to skip)...")
        
        response = input()
        if response.lower() == 'q':
            print("Skipping speed gain calibration")
            return
        
        # Test at multiple speeds
        test_speeds = [0.5, 1.0, 2.0]  # m/s
        measurements = []
        
        for test_speed in test_speeds:
            # Set duration based on speed (shorter for higher speeds to prevent car from going too far)
            duration = 3.0 if test_speed == 2.0 else 5.0
            
            print(f"\n--- Testing at {test_speed} m/s ---")
            print("The car will drive forward at constant speed.")
            print(f"Measure the distance it travels in {duration:.0f} seconds.")
            print("Press ENTER when ready...")
            input()
            
            print("Starting in 3 seconds... Hold R1 to allow the car to move")
            time.sleep(3)
            
            # Record start position
            self.start_odom = self.get_position()
            start_time = time.time()
            
            rpm_samples = []
            
            while time.time() - start_time < duration:
                self.send_drive_command(test_speed, 0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
                
                # Record RPM
                if self.vesc_state is not None:
                    rpm_samples.append(self.vesc_state.state.speed)
            
            self.stop()
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Calculate distance from odometry
            distance_odom = self.get_distance_traveled()
            actual_speed_odom = distance_odom / duration
            
            # Get average RPM
            avg_rpm = np.mean(rpm_samples) if rpm_samples else 0
            
            print(f"\nTest complete!")
            print(f"  Commanded speed: {test_speed:.2f} m/s")
            print(f"  Distance (odometry): {distance_odom:.3f} m")
            print(f"  Actual speed (odometry): {actual_speed_odom:.3f} m/s")
            print(f"  Average ERPM: {avg_rpm:.1f}")
            
            print("\nFor better accuracy, measure the actual distance with a tape measure.")
            print(f"Enter the actual distance traveled in meters (or press ENTER to use {distance_odom:.3f} m):")
            
            user_distance = input("Distance (m): ").strip()
            if user_distance:
                try:
                    distance_odom = float(user_distance)
                    actual_speed_odom = distance_odom / duration
                    print(f"Using measured distance: {distance_odom:.3f} m")
                except ValueError:
                    print("Invalid input, using odometry distance")
            
            measurements.append({
                'commanded_speed': test_speed,
                'actual_speed': actual_speed_odom,
                'erpm': avg_rpm
            })
        
        if len(measurements) == 0:
            print("No measurements recorded")
            return
        
        # Calculate gain using linear regression
        # erpm = gain * speed + offset
        speeds = np.array([m['actual_speed'] for m in measurements])
        erpms = np.array([m['erpm'] for m in measurements])
        
        # Use the offset we calculated earlier, or read from config
        offset = self.results['speed_offset']
        if offset is None:
            # Read the existing offset from config files
            offset, _ = self.read_speed_config()
            self.get_logger().info(f"Using existing speed_to_erpm_offset from config: {offset:.1f}")
        else:
            self.get_logger().info(f"Using calibrated speed_to_erpm_offset: {offset:.1f}")
        
        # Solve for gain: gain = (erpm - offset) / speed
        # Use least squares for better accuracy
        A = speeds.reshape(-1, 1)
        b = erpms - offset
        gain, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        gain = gain[0]
        
        print(f"\n📊 Calibration Results:")
        print(f"   Calculated gain: {gain:.1f}")
        print(f"   Using offset: {offset:.1f}")
        print(f"\nMeasurements:")
        for m in measurements:
            predicted_erpm = gain * m['actual_speed'] + offset
            error = abs(m['erpm'] - predicted_erpm)
            print(f"  Speed: {m['actual_speed']:.2f} m/s, ERPM: {m['erpm']:.1f}, "
                  f"Predicted: {predicted_erpm:.1f}, Error: {error:.1f}")
        
        # Validate the gain is reasonable
        expected_min_gain = 1000  # Typical minimum for RC cars
        expected_max_gain = 10000  # Typical maximum for RC cars
        gain_is_valid = expected_min_gain <= gain <= expected_max_gain
        
        if not gain_is_valid:
            print(f"\n⚠️  WARNING: Calculated gain ({gain:.1f}) is outside expected range!")
            print(f"   Expected range: {expected_min_gain} - {expected_max_gain}")
            print(f"\n🔍 Diagnostic Information:")
            print(f"   ERPM range: {erpms.min():.1f} to {erpms.max():.1f} (variation: {erpms.max() - erpms.min():.1f})")
            print(f"   Speed range: {speeds.min():.2f} to {speeds.max():.2f} m/s")
            
            # Check for common problems
            if erpms.max() - erpms.min() < 100:
                print(f"\n❌ PROBLEM DETECTED: ERPM values barely changed!")
                print(f"   The car was likely NOT responding to speed commands.")
                print(f"   Common causes:")
                print(f"   1. R1 button was not held during the ENTIRE test duration")
                print(f"   2. Car.lua has incorrect values preventing movement")
                print(f"   3. VESC driver is not receiving/processing commands")
                print(f"   4. Motor controller is in an error state")
            elif gain < 0:
                print(f"\n❌ PROBLEM DETECTED: Negative gain (impossible!)")
                print(f"   This means ERPM decreased as speed increased.")
                print(f"   The offset ({offset:.1f}) is likely much too high.")
                print(f"   Try calibrating speed_offset first, or use vesc.lua defaults.")
            elif gain < expected_min_gain:
                print(f"\n❌ PROBLEM DETECTED: Gain is too low")
                print(f"   The car was moving, but ERPM values are lower than expected.")
                print(f"   This usually means the offset ({offset:.1f}) is incorrect.")
            elif gain > expected_max_gain:
                print(f"\n⚠️  Gain is higher than typical, but might be correct for your setup.")
            
            print(f"\n   Options:")
            print(f"   1. Skip saving (recommended - fix the issue and retry)")
            print(f"   2. Save anyway (not recommended)")
            
            choice = input("\n   Enter choice (1/2): ").strip()
            if choice != '2':
                print(f"\n   Skipped saving speed_to_erpm_gain.")
                print(f"   Please fix the issue and re-run speed gain calibration.")
                return
            else:
                print(f"\n   ⚠️  Saving potentially incorrect gain value...")
        
        self.results['speed_gain'] = gain
        print(f"\n✓ Speed gain calibrated: {gain:.1f}")
        
        # Write the gain to car.lua config file
        if self.write_speed_gain(self.results['speed_gain']):
            print(f"\n✅ Updated car.lua with speed gain: {self.results['speed_gain']:.1f}")
        else:
            print(f"\n⚠️  Failed to update car.lua. Please update it manually.")
            print(f"    Add this line: speed_to_erpm_gain = {self.results['speed_gain']:.1f};")
    
    def print_results(self):
        """Print the calibration results summary"""
        print("\n" + "="*60)
        print("CALIBRATION RESULTS SUMMARY")
        print("="*60)
        
        if all(v is None for v in self.results.values()):
            print("\nNo calibration was performed.")
            return
        
        print("\n✅ The following values have been written to car.lua:")
        print("-"*60)
        
        if self.results['steering_offset'] is not None:
            print(f"steering_angle_to_servo_offset = {self.results['steering_offset']:.4f};")
        
        if self.results['steering_gain'] is not None:
            print(f"steering_angle_to_servo_gain = {self.results['steering_gain']:.4f};")
        
        if self.results['speed_offset'] is not None:
            print(f"speed_to_erpm_offset = {self.results['speed_offset']:.1f};")
        
        if self.results['speed_gain'] is not None:
            print(f"speed_to_erpm_gain = {self.results['speed_gain']:.1f};")
        
        if self.results['max_steering_angle'] is not None:
            print(f"max_steering_angle = {self.results['max_steering_angle']:.4f}; -- {math.degrees(self.results['max_steering_angle']):.2f} degrees")
            min_turn_radius = 0.324 / math.tan(self.results['max_steering_angle'])
            print(f"  --> Minimum turning radius: {min_turn_radius:.3f} m")
        
        print("-"*60)
        
        print("\n⚠️  IMPORTANT: You must restart the VESC driver node for the change to take effect!")
        print("   You can do this by:")
        print("   1. Finding which tmux pane contains the vesc_driver.")
        print("   2. Restarting it by pressing CTRL-C and then UP ARROW and ENTER to rerun command.")
        
        print("\n📝 NOTE: These are initial calibration values. Fine-tuning may be needed.")
        print("   Test the car in autonomous mode and adjust as necessary.")
        
def main():
    print("="*60)
    print("       VESC DRIVER CALIBRATION SCRIPT")
    print("="*60)
    print("\nThis script will guide you through calibrating your VESC driver.")
    print("\n️ SAFETY WARNINGS:")
    print("  - Ensure the car has plenty of space to move")
    print("  - Hold R1 trigger to allow the car to move (release to stop)")
    print("  - Have the joystick controller ready at all times")
    print("\nPress ENTER to continue or Ctrl+C to exit...")
    input()
    
    rclpy.init()
    
    calibrator = VESCCalibrator()
    
    try:
        # Wait for data
        calibrator.wait_for_data()
        
        # Run calibration steps
        calibrator.calibrate_steering_offset()
        calibrator.calibrate_steering_gain()
        calibrator.calibrate_speed_offset()
        calibrator.calibrate_speed_gain()
        
        # Print results
        calibrator.print_results()
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
        calibrator.stop()
    except Exception as e:
        print(f"\n\nError during calibration: {e}")
        import traceback
        traceback.print_exc()
        calibrator.stop()
    finally:
        calibrator.stop()
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
