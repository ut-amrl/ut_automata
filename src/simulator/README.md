# RoboRacer Simulator

Run the simulator with localization:

```
ros2 run ut_automata simulator --localize
```

- The simulator only subscribes to 2 topics:
   - `/ackermann_curvature_drive` (for drive commands)
   - `/set_pose` (for initial pose setting)

- Available teleop options:
   - keyboard_teleop.py - keyboard control → publishes to `/ackermann_curvature_drive`
   - joystick_teleop.py - joystick control → publishes to `/commands/ackermann`

- To drive with the joystick, press `L1` and then steer with the left joystick and apply throttle with the right joystick.