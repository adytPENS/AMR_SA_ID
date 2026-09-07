"""ROS sensor/output adapter. Keyboard remains the sole motor-command owner."""
import json
import math
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String
from rclpy.qos import qos_profile_sensor_data
from mission_runner import MissionRunner


def scan_sectors(msg):
    sectors = {'front': [], 'rear': [], 'left': [], 'right': []}
    centers = {'front': 0, 'rear': math.pi, 'left': math.pi/2, 'right': -math.pi/2}
    for i, value in enumerate(msg.ranges):
        if not math.isfinite(value) or not msg.range_min <= value <= msg.range_max:
            continue
        angle = msg.angle_min + i * msg.angle_increment
        for name, center in centers.items():
            difference = math.atan2(math.sin(angle-center), math.cos(angle-center))
            if abs(difference) <= math.radians(15):
                sectors[name].append(value)
    return {name: min(values) if values else None for name, values in sectors.items()}


class MissionROS:
    def __init__(self, node, config, args):
        self.node = node
        self.args = args
        self.runner = MissionRunner(config, time.monotonic, args.servo_positions)
        self.results_pub = node.create_publisher(String, '/human_interaction/results', 10)
        self.last_results = None
        self.lights = {color: node.create_publisher(Bool, f'/light_{color}/cmd', 10)
                       for color in ('control', 'red', 'green', 'yellow')}
        self.last_lights = None
        self.light_time = 0
        node.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu', self.imu, qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan', lambda msg: self.runner.observe('scan', scan_sectors(msg)), qos_profile_sensor_data)
        node.create_subscription(String, '/color_tracker/result', self.camera, 1)
        for axis, motor in (('lift', 2), ('rotate', 3)):
            node.create_subscription(Float64, f'/{args.oms_sensor}/m_{motor}/encoder',
                                     lambda msg, axis=axis: self.scalar(axis, msg.data), 10)
        for i in range(4):
            node.create_subscription(Float64, f'/{args.sensor}/m_{i}/encoder',
                                     lambda msg, i=i: self.scalar(f'wheel_{i}', msg.data), 10)

    def scalar(self, name, value):
        if math.isfinite(value):
            self.runner.observe(name, value)

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if all(math.isfinite(v) for v in values) and sum(v*v for v in values[2:]) > .5:
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            self.runner.observe('odom', (p.x, p.y, yaw))

    def imu(self, msg):
        q = msg.orientation
        if all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            self.runner.observe('imu', True)

    def camera(self, msg):
        try:
            result = json.loads(msg.data)
            if (isinstance(result, dict) and isinstance(result.get('objects'), list)
                    and result.get('status') != 'no_rgb'):
                self.runner.observe('camera', result)
        except (ValueError, TypeError):
            pass

    def tick(self):
        node = self.node
        for name, sender in node.servo_commands.items():
            if sender.confirmed is not None:
                self.runner.observe('servo_' + name, sender.confirmed)
        output = self.runner.tick()
        lift = 0.0
        try:
            direction = output.get('lift_direction', 0)
            if direction:
                positive = self.runner.calibration['lift_positive_duty_sign']
                rpm = self.args.lift_up_rpm if direction == positive else self.args.lift_down_rpm
                lift = node.lift_pid.calculate(rpm, direction, time.monotonic())
            else:
                node.lift_pid.reset()
        except RuntimeError as error:
            self.runner.fail(str(error))
            output = {}
        node.publish_cmd(output.get('vx', 0), output.get('wz', 0))
        node.publish_oms(lift, output.get('rotate', 0))
        if 'servo' in output:
            name, value = output['servo']
            node.standard_servos[name].target = value
            node.standard_servos[name].last_sent = value
            node.servo_commands[name].set_target(value)
        node.publish_servos(output.get('slide', 0))
        self.update_lights()
        results = json.dumps(self.runner.results)
        if results != self.last_results:
            self.results_pub.publish(String(data=results))
            self.last_results = results

    def update_lights(self):
        color, mode = self.runner.light
        enabled = mode == 'steady' or int(time.monotonic()*2) % 2 == 0
        states = tuple(color == c and enabled for c in ('red', 'green', 'yellow'))
        now = time.monotonic()
        if states != self.last_lights or now - self.light_time > .5:
            self.lights['control'].publish(Bool(data=color != 'off'))
            for c, value in zip(('red', 'green', 'yellow'), states):
                self.lights[c].publish(Bool(data=value))
            self.last_lights, self.light_time = states, now

    def stop(self):
        self.runner.cancel()
        for publisher in self.lights.values():
            publisher.publish(Bool(data=False))
        self.last_lights = None
