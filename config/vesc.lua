serial_port = "/dev/ttyACM0";
-- erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) +
--   speed_to_erpm_offset
-- for offset=0. speed_to_erpm_gain =
--   num_motor_poles*60/circumference_wheel_in_meters
speed_to_erpm_gain = 5356; -- arrma 3800: 17T pinion gear + P48 T83 spur gear
speed_to_erpm_offset = 180.0; -- should be between 160-200
-- servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle
--    (radians) + steering_angle_to_servo_offset
steering_angle_to_servo_gain = -0.9015; -- -0.9015
steering_angle_to_servo_offset = 0.53; -- should be between 0.4-0.6, default: 0.53
-- erpm_speed_limit = 14000; -- 3250
erpm_speed_limit = 40000; -- 3250 --22000
servo_min = 0.01;
servo_max = 0.99;

-- Taken from the manufacturer: 
-- https://traxxas.com/products/models/electric/ford-fiesta-st-rally?t=specs
wheelbase = 0.324;

max_acceleration = 8.0; -- m/s^2
max_deceleration = 12.0; -- m/s^2

joystick_normal_speed = 1.0; -- m/s
joystick_turbo_speed = 8.0; -- m/s
