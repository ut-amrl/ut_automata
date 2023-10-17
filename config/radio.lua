serial_port = "/dev/ttyACM0";
baud_rate = 9600;

command_rate = 20;  -- how often to send commands (Hz)
command_timeout = 0.5; -- if a command is not received after this amount of time, the car will stop

full_speed = 8.0; -- fastest the car is capable of going (m/s)
max_speed = 4.0; -- fastest we want to allow the car to go (m/s)
max_accel = 10.0; -- most acceleration we want to allow
max_decel = 10.0; -- most decelration we want to allow

speed_to_throttle = 1.0 / full_speed; -- how much throttle to apply per m/s of speed, assumption of linearity

max_curvature = 1.5; -- maximum curvature the car is capable of
curvature_to_servo = 1.0 / max_curvature; -- how much to steer per unit of curvature, assumption of linearity
