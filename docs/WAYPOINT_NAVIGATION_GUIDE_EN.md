# Waypoint Navigation Without SLAM — English

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
4. drives approximately `0.60 m`;
5. recalculates the direction to the active waypoint.

The settings are stored in `waypoints.yaml`:

```yaml
obstacle_avoidance:
  enabled: true
  stop_distance: 0.55
  clear_distance: 0.75
  avoid_angle_deg: 55.0
  avoid_step_distance: 0.60
  timeout: 12.0
```

This is reactive avoidance, not global route planning. It may fail in narrow
corridors, U-shaped obstacles, or dense environments. An operator-accessible
stop command remains mandatory.

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
