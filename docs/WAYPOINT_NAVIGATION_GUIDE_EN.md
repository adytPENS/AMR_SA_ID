# Waypoint Navigation Without SLAM — English

> **Current architecture:** the navigator publishes `/cmd_vel` only. Inverse
> kinematics, M0-M3 PID, polarity, and Titan duty are handled by
> `drive_controller.py`. See the
> [AMR drive controller guide](AMR_DRIVE_CONTROLLER_GUIDE_EN.md).

Versi Bahasa Indonesia: [WAYPOINT_NAVIGATION_GUIDE.md](WAYPOINT_NAVIGATION_GUIDE.md)

This program is intended for training and competition tasks where judges
provide point locations and a driving sequence. The robot estimates its local
pose from wheel encoders and the VMX navX IMU, then drives to A, B, C, and D in
the requested order.

## Coordinate system

Place the robot at a known starting pose:

```text
                     +Y (left)
                        ^
                        |
                        |
  start point (0,0) -----+----> +X (robot forward)
```

All coordinates are expressed in metres. Two coordinate modes are supported:

```text
coordinate_mode: map    points and start_pose use the judges' map frame
coordinate_mode: local  points are already relative to the robot start pose
```

For `map` mode, enter the robot pose at the start position:

```yaml
coordinate_mode: map
start_pose: {x: 0.50, y: 0.30, yaw_deg: 90.0}
```

The program automatically translates and rotates map coordinates into the
local odometry frame, which still begins at `(0,0,0)`. Example map point:

```yaml
A: {x: 2.0, y: 1.0}
```

Yaw conventions:

```text
  0 degrees = robot faces map +X
 90 degrees = robot faces map +Y
180 degrees = robot faces map -X
-90 degrees = robot faces map -Y
```

This mode does not provide global localization. Before every run, place the
robot at the configured start position and heading, then reset odometry.
Encoder error accumulates over the route.

## Entering points and the route sequence

Edit:

```text
/home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Example:

```yaml
coordinate_mode: map
start_pose: {x: 0.40, y: 0.30, yaw_deg: 0.0}

waypoints:
  A: {x: 1.20, y: 0.50}
  B: {x: 2.60, y: 0.50}
  C: {x: 2.60, y: 1.70}
  D: {x: 0.80, y: 1.70}

sequence: [A, B, D, C]
```

Coordinates may be changed after the point-location dice roll. Change
`sequence` after the route-order dice roll.

## Prerequisites

Before starting waypoint mode:

1. Place the robot at the configured start pose on a level surface.
2. Start `control_server` for Titan and IMU.
3. Keep the robot still for 15–20 seconds.
4. Zero the IMU yaw.
5. Initialize all four encoders.
6. Start the YDLIDAR driver.
7. Verify that `/imu`, `/scan`, and all four encoder topics are available.

Do not run keyboard teleoperation and waypoint navigation simultaneously.
Both programs publish motor commands.

## Starting waypoint mode

The recommended command is:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh
```

This script starts wheel odometry, resets the pose to `(0,0,0)`, and loads the
waypoint navigator. The motors remain stopped until the start service is
called.

From another terminal, start the route:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 service call /waypoint_navigator/start \
  std_srvs/srv/Trigger "{}"
```

## Physical start button

In competition mode, all ROS nodes should already be running while the robot
remains `ARMED / STOPPED`. The physical button does not boot ROS from scratch;
it triggers this sequence:

```text
press START -> debounce -> validate odometry and LiDAR -> reset odometry
             -> run the configured A/B/C/D sequence
```

Configure the selected VMX DIO channel in `titan_m1_test.yaml`:

```yaml
dio:
  enabled: true
  sensors: ["start_button"]
  start_button:
    pin: 0              # replace with the actual DIO channel
    type: "input"
    interrupt_edge: "rising"
    debounce_ms: 250
```

Enable the button subscriber in `waypoints.yaml`:

```yaml
start_button:
  enabled: true
  topic: "/start_button/state"
  active_high: true     # use false for active-low wiring
  debounce_ms: 250
```

Never connect 12 V to a DIO input. Follow the VMX2 electrical specification
for input wiring and the required pull-up or pull-down resistor. Verify the
button before enabling motor motion:

```bash
ros2 topic echo /start_button/state
```

The value must change when the button is pressed and return when released. A
physical emergency stop that disables actuator power is still recommended;
the start button is not an emergency stop.

Emergency stop from any ROS terminal:

```bash
ros2 service call /waypoint_navigator/stop \
  std_srvs/srv/Trigger "{}"
```

Pressing `Ctrl+C` in the navigator terminal also sends zero to every motor.

## Manual startup

To start odometry without SLAM:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/wheel_odometry.py \
  --ros-args \
  --params-file /home/vmx/studica_ws/src/studica_control/config/wheel_odometry.yaml
```

Reset odometry while the robot is at the start pose:

```bash
ros2 service call /wheel_odometry/reset std_srvs/srv/Empty "{}"
```

Start the navigator:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/waypoint_navigator.py \
  --config /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Override the sequence without editing YAML:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/waypoint_navigator.py \
  --config /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml \
  --sequence A,B,D,A,C,B
```

## Reactive obstacle avoidance

The LiDAR monitors the forward sector. If an obstacle is detected closer than
`0.55 m`, the robot:

1. stops;
2. compares left and right clearance;
3. turns approximately 55 degrees toward the clearer side;
4. follows the wall while controlling front and side clearance;
5. leaves the wall after the waypoint direction stays clear and the robot has
   made measurable progress toward the goal;
6. resumes direct motion toward the active waypoint.

The settings are stored in `waypoints.yaml`:

```yaml
obstacle_avoidance:
  enabled: true
  front_half_angle_deg: 30.0
  usable_half_angle_deg: 80.0
  stop_distance: 0.55
  clear_distance: 0.75
  avoid_angle_deg: 55.0
  turn_timeout: 12.0
  wall_distance: 0.45
  wall_speed: 0.14
  wall_kp: 1.50
  wall_max_turn: 0.55
  wall_search_turn: 0.25
  leave_heading_deg: 25.0
  progress_margin: 0.15
  leave_clear_time: 0.60
  wall_follow_timeout: 35.0
```

Wall following uses the verified `-80..+80` degree LiDAR area and ignores the
rear wheel/body reflections. This remains reactive avoidance rather than a
global planner. A U-shaped obstacle or blocked corridor may end in
`wall_follow_timeout` and a safety stop. An operator-accessible stop button
remains mandatory.

If a fixed wall blocks the direct line between two competition points, add
intermediate corridor points such as `AB1` and `AB2`:

```yaml
waypoints:
  A:   {x: 3.50, y: 1.60}
  AB1: {x: 3.00, y: 0.50}
  AB2: {x: 2.30, y: 0.50}
  B:   {x: 2.00, y: 0.60}

sequence: [A, AB1, AB2, B]
```

Reactive avoidance is primarily intended for the movable block placed by the
judges. It does not replace route planning around fixed walls shown on the
map.

## Safety and initial testing

- Begin with short waypoints, for example A at `(0.50, 0.00)`.
- Keep an operator ready to call the stop service.
- Test without an obstacle first.
- Verify that a physical left turn produces positive odometry yaw.
- Verify LiDAR front, left, and right directions before enabling avoidance.
- Because the current LiDAR is mounted low, confirm that the robot frame and
  wheels are not reported as forward obstacles.
- Never run keyboard teleoperation while the navigator is active.

## Known limitations

- Odometry drifts because there is no global localization correction.
- Duty-cycle control does not guarantee identical wheel speed under unequal
  load.
- Reactive avoidance cannot guarantee a valid route through every map.
- Accurate competition use requires measured start pose, calibrated wheel
  distance, reliable IMU yaw, and validated LiDAR mounting.
