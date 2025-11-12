serial_port = "/dev/ttyACM0";
-- erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) +
--   speed_to_erpm_offset
-- for offset=0. speed_to_erpm_gain =
--   num_motor_poles*60/circumference_wheel_in_meters
speed_to_erpm_gain = 5356; -- arrma 3800: 17T pinion gear + P48 T83 spur gear
speed_to_erpm_offset = 180.0; -- should be between 160-200
-- servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle
--    (radians) + steering_angle_to_servo_offset
steering_angle_to_servo_gain = -0.9015;
steering_angle_to_servo_offset = 0.5; -- should be between 0.4-0.6
-- erpm_speed_limit = 14000; -- 3250
erpm_speed_limit = 53740; -- gain (5356) * max_speed (10 m/s) + offset (180)
servo_min = 0.05;
servo_max = 0.95;

-- Taken from the manufacturer: 
-- https://traxxas.com/products/models/electric/ford-fiesta-st-rally?t=specs
wheelbase = 0.324;

max_acceleration = 6.0; -- m/s^2
max_deceleration = 6.0; -- m/s^2

joystick_normal_speed = 1.0; -- m/s
joystick_turbo_speed = 2.0; -- m/s

-- Maximum steering angle in radians for joystick control
-- Formula: max_steering_angle = atan(wheelbase / desired_min_turning_radius)
-- Example: 0.286 rad (16.4°) gives 1.1m turning radius with 0.324m wheelbase
max_steering_angle = 0.286; -- radians

-- IMU fusion parameters
fuse_imu = true; -- Set to true to fuse IMU data with odometry using EKF
i2c_bus_number = 7; -- I2C bus number for MPU6050 sensor
calibrate_imu = true; -- Calibrate IMU on startup
imu_gyro_range = 0; -- 0=250, 1=500, 2=1000, 3=2000 deg/s
imu_accel_range = 0; -- 0=2g, 1=4g, 2=8g, 3=16g
imu_dlpf_bandwidth = 0; -- 0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
