# AMR Drive Controller — English

This guide covers the current VMX2 Pi architecture for Titan CAN ID 42 and
four independently encoded Maverick 50.9:1 motors.

```text
Keyboard / Waypoint / Nav2
          -> /cmd_vel
          -> inverse kinematics selected in YAML
          -> M0-M3 wheel-speed targets
          -> one encoder PID per motor
          -> Titan duty commands
```

The keyboard does not know motor channels, electrical polarity, duty, PID, or
drive geometry. `drive_controller.py` is the only motor-command publisher.

## Configuration

Edit `src/studica_control/config/drive_controller.yaml`:

```yaml
drive:
  # differential, differential_all_terrain, mecanum, or x_drive
  model: differential_all_terrain
  wheel_diameter: 0.12
  track_width: 0.35
  wheelbase: 0.29
  max_wheel_speed: 0.75

motors:
  order: [front_right, rear_right, front_left, rear_left]
  titan_channels: [0, 1, 2, 3]
  electrical_polarity: [-1.0, -1.0, 1.0, 1.0]

speed_pid:
  enabled: true
  kp: 0.25
  ki: 0.10
  kd: 0.0
  integral_limit: 0.50
  feedback_timeout: 0.30
  duty_limit: 0.70

safety:
  cmd_vel_timeout: 0.35
```

All dimensions use metres and are measured between wheel centres.
Differential models reject lateral `linear.y`; mecanum and X-drive support it.

## Run

Terminal 1 — hardware server:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition \
  --ros-args -r __node:=control_server \
  --params-file /home/vmx/studica_ws/src/studica_control/config/titan_m1_test.yaml
```

Terminal 2 — universal drive controller:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/drive_controller.py \
  --config /home/vmx/studica_ws/src/studica_control/config/drive_controller.yaml
```

Terminal 3 — enable Titan and start the keyboard:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'enable'}"
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.15 --angular-speed 0.8
```

Keys: `W` forward, `S` reverse, `A` rotate left, `D` rotate right, `G` encoder
distance target, `E` stop, and `Q` quit.

Hard stop:

```bash
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'disable'}"
```

The waypoint navigator now publishes `/cmd_vel` only. Start wheel odometry,
the drive controller, and the navigator together with:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh \
  /home/vmx/studica_ws/src/studica_control/config/waypoints_test.yaml
```

Then start the route from another terminal:

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger "{}"
```

Do not run the keyboard while the navigator is active. Both are valid
`/cmd_vel` sources, but only one motion source should be active at a time.
