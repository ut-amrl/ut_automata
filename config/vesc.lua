serial_port = "/dev/ttyACM0";
-- erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) +
--   speed_to_erpm_offset
-- for offset=0. speed_to_erpm_gain =
--   num_motor_poles*60/circumference_wheel_in_meters
speed_to_erpm_gain = 5356; -- arrma 3800: 17T pinion gear + P48 T83 spur gear
speed_to_erpm_offset = 160.0;
-- servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle
--    (radians) + steering_angle_to_servo_offset
steering_angle_to_servo_gain = -0.9015;
steering_angle_to_servo_offset = 0.426; --0.5054; -- 0.5304
-- erpm_speed_limit = 14000; -- 3250
erpm_speed_limit = 22000; -- 3250
servo_min = 0.05;
servo_max = 0.95;

-- Taken from the manufacturer: 
-- https://traxxas.com/products/models/electric/ford-fiesta-st-rally?t=specs
wheelbase = 0.324;

max_acceleration = 6.0; -- m/s^2
max_deceleration = 6.0; -- m/s^2

joystick_normal_speed = 1.0; -- m/s
joystick_turbo_speed = 2.0; -- m/s
