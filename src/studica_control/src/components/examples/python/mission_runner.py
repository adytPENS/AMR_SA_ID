"""Non-blocking YAML mission engine. Units are explicit; no ROS dependencies."""
import math


def number(value, name, low=-float('inf'), high=float('inf')):
    if isinstance(value, bool):
        raise ValueError(f'{name}: perlu angka')
    try:
        value = float(value)
    except (ValueError, TypeError):
        raise ValueError(f'{name}: perlu angka') from None
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name}: harus {low}..{high}')
    return value


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class MissionRunner:
    COMMON = {'action', 'timeout_s'}
    FIELDS = {
        'goto': {'x', 'y', 'speed_mps'},
        'forward': {'until', 'speed_mps'},
        'trace': {'side', 'wall_distance_cm', 'until', 'speed_mps'},
        'heading': {'angle_deg'}, 'robot_movetheta': {'angle_deg'},
        'turn': {'direction', 'angle_deg'},
        'robot_move': {'vx_mps', 'duration_s'},
        'oms_rotate': {'angle_deg', 'target', 'duty_percent'},
        'lift': {'delta_cm', 'target'},
        'slide': {'direction', 'duration_s', 'duty_percent'},
        'wrist': {'position', 'settle_s'},
        'gripper': {'position', 'settle_s'},
        'light': {'color', 'mode'}, 'wait': {'seconds'}, 'stop': set(),
        'detect_camera': {'colour', 'shape', 'save_as', 'on_not_found'},
    }
    BASE = {'goto', 'forward', 'trace', 'heading', 'robot_movetheta', 'turn', 'robot_move'}

    def __init__(self, config, clock, servo_positions):
        self.config = config
        self.clock = clock
        self.steps = config.get('mission', [])
        if not isinstance(self.steps, list):
            raise ValueError('mission harus list')
        self.servos = servo_positions
        self.calibration = config.get('calibration', {})
        self.settings = {
            'feedback_timeout_s': .5, 'linear_speed_mps': .15,
            'angular_speed_rps': .5, 'position_tolerance_m': .05,
            'heading_tolerance_deg': 3, 'front_stop_cm': 8,
            'rotate_tolerance_deg': 2, 'lift_tolerance_cm': .5,
            'trace_kp': 2, 'heading_kp': 1.5, 'rotate_duty_percent': 33.3,
            'slide_duty_percent': 15,
        }
        supplied = config.get('settings', {})
        if not isinstance(supplied, dict) or set(supplied) - self.settings.keys():
            raise ValueError('settings: nama parameter tidak dikenal')
        self.settings.update(supplied)
        for name, value in self.settings.items():
            self.settings[name] = number(value, name, .001, 100)
        number(self.settings['linear_speed_mps'], 'linear_speed_mps', .01, .75)
        number(self.settings['angular_speed_rps'], 'angular_speed_rps', .01, 2)
        self.start_pose = config.get('start_pose', {'x': 0, 'y': 0, 'yaw_deg': 0})
        for key in ('x', 'y', 'yaw_deg'):
            self.start_pose[key] = number(self.start_pose.get(key), 'start_pose.' + key)
        self.data = {}
        self.state = 'IDLE'
        self.index = 0
        self.entered = None
        self.context = {}
        self.results = {}
        self.light = ('off', 'steady')
        self.message = 'Belum START'
        self.snapshot = None
        self.origin = None
        self.validate()

    def validate(self):
        for i, s in enumerate(self.steps):
            if not isinstance(s, dict) or s.get('action') not in self.FIELDS:
                raise ValueError(f'mission[{i}]: action tidak dikenal')
            a = s['action']
            for key in ('timeout_s', 'x', 'y', 'speed_mps', 'angle_deg', 'vx_mps',
                        'duration_s', 'delta_cm', 'duty_percent', 'settle_s',
                        'seconds', 'wall_distance_cm'):
                if key in s:
                    s[key] = number(s[key], key)
            unknown = s.keys() - self.COMMON - self.FIELDS[a]
            if unknown:
                raise ValueError(f'mission[{i}]: parameter tidak dikenal {sorted(unknown)}')
            number(s.get('timeout_s', 30), 'timeout_s', .05, 600)
            if a in ('goto',):
                number(s.get('x'), 'x'); number(s.get('y'), 'y')
            if a in ('goto', 'forward', 'trace'):
                number(s.get('speed_mps', self.settings['linear_speed_mps']), 'speed_mps', .01, .75)
            if a in ('forward', 'trace'):
                until = s.get('until')
                if not isinstance(until, dict) or len(until) != 1:
                    raise ValueError('until: pilih travel_cm atau lidar_front_cm')
                key = next(iter(until))
                if key not in ('travel_cm', 'lidar_front_cm'):
                    raise ValueError('until tidak dikenal')
                until[key] = number(until[key], key, .1, 10000)
                if key == 'lidar_front_cm' and until[key] <= self.settings['front_stop_cm']:
                    raise ValueError('lidar_front_cm harus lebih besar dari front_stop_cm')
            if a == 'trace':
                if s.get('side') not in ('left', 'right'):
                    raise ValueError('trace.side harus left/right')
                number(s.get('wall_distance_cm'), 'wall_distance_cm', 1, 500)
            if a in ('heading', 'robot_movetheta', 'turn'):
                number(s.get('angle_deg'), 'angle_deg', -360, 360)
            if a == 'turn':
                if s.get('direction') not in ('left', 'right'):
                    raise ValueError('turn.direction harus left/right')
                number(s['angle_deg'], 'turn.angle_deg', 0, 360)
            if a == 'robot_move':
                number(s.get('vx_mps'), 'vx_mps', -.75, .75)
                number(s.get('duration_s'), 'duration_s', .01, 300)
            if a in ('oms_rotate', 'lift'):
                field = 'angle_deg' if a == 'oms_rotate' else 'delta_cm'
                if (field in s) == ('target' in s):
                    raise ValueError(f'{a}: pilih {field} atau target: default')
                if 'target' in s and s['target'] != 'default':
                    raise ValueError('target harus default')
                if field in s:
                    number(s[field], field, -360 if a == 'oms_rotate' else -200,
                           360 if a == 'oms_rotate' else 200)
                if a == 'oms_rotate':
                    number(s.get('duty_percent', self.settings['rotate_duty_percent']), 'duty_percent', .1, 100)
            if a == 'slide':
                if s.get('direction') not in ('forward', 'backward'):
                    raise ValueError('slide.direction harus forward/backward')
                number(s.get('duration_s'), 'duration_s', .01, 30)
                number(s.get('duty_percent', self.settings['slide_duty_percent']), 'duty_percent', .1, 100)
            if a in ('wrist', 'gripper'):
                if s.get('position') not in ('on', 'off', 'default'):
                    raise ValueError('position harus "on", "off", atau default (kutip on/off di YAML)')
                number(s.get('settle_s', 1), 'settle_s', 0, 30)
            if a == 'light':
                if s.get('color') not in ('red', 'green', 'yellow', 'off') or s.get('mode', 'steady') not in ('steady', 'blink'):
                    raise ValueError('light: color/mode tidak dikenal')
            if a in ('wrist', 'gripper') and s.get('position') != 'default':
                mapping = self.config.get('servo_positions', {}).get(a, {'on': 'left_value', 'off': 'right_value'})
                if not isinstance(mapping, dict) or mapping.get(s['position']) not in ('left_value', 'right_value'):
                    raise ValueError('servo_positions: gunakan "on"/"off" dan left_value/right_value')
            if a in ('slide', 'robot_move') and s['duration_s'] >= s.get('timeout_s', 30):
                raise ValueError('timeout_s harus lebih besar dari duration_s')
            if a == 'wait':
                number(s.get('seconds'), 'seconds', 0, 300)
                if s['seconds'] >= s.get('timeout_s', 30):
                    raise ValueError('timeout_s harus lebih besar dari seconds')
            if a == 'detect_camera':
                if not isinstance(s.get('colour'), str) or not s['colour'].strip():
                    raise ValueError('detect_camera perlu colour')
                if 'shape' in s and s['shape'] not in ('rectangle', 'circle', 'triangle'):
                    raise ValueError('shape: rectangle/circle/triangle')
                if s.get('on_not_found', 'stop') not in ('stop', 'continue'):
                    raise ValueError('on_not_found: stop/continue')
                if not isinstance(s.get('save_as', 'target'), str):
                    raise ValueError('save_as harus string')

    def observe(self, name, value):
        self.data[name] = (value, self.clock())

    def read(self, name):
        sample = self.data.get(name)
        if sample is None or self.clock() - sample[1] > self.settings['feedback_timeout_s']:
            raise ValueError(f'{name}: feedback belum tersedia/kedaluwarsa')
        return sample[0]

    def start(self, snapshot):
        self.validate()
        if not self.steps:
            raise ValueError('mission kosong')
        # Validate all physical scale factors before any mission movement.
        for s in self.steps:
            if s['action'] in ('lift', 'oms_rotate'):
                axis = 'lift' if s['action'] == 'lift' else 'rotate'
                key = axis + ('_cm_per_rev' if axis == 'lift' else '_deg_per_rev')
                scale = number(self.calibration.get(key), 'calibration.' + key)
                self.calibration[key] = scale
                if scale == 0:
                    raise ValueError(key + ' tidak boleh nol')
                sign = number(self.calibration.get(axis + '_positive_duty_sign'), axis + '_positive_duty_sign', -1, 1)
                self.calibration[axis + '_positive_duty_sign'] = sign
                if abs(sign) != 1:
                    raise ValueError('duty_sign harus -1 atau 1')
        self.origin = self.read('odom') if any(s['action'] in self.BASE for s in self.steps) else None
        self.snapshot = snapshot
        self.index = 0
        self.entered = None
        self.context = {}
        self.results = {}
        self.state = 'RUNNING'
        self.message = 'Misi dimulai'

    def cancel(self):
        self.state = 'IDLE'
        self.light = ('off', 'steady')
        self.context = {}

    def fail(self, message):
        self.state = 'ERROR'
        self.message = message
        self.light = ('red', 'steady')

    def pose(self):
        x, y, yaw = self.read('odom')
        self.read('imu')
        for i in range(4):
            self.read(f'wheel_{i}')
        ox, oy, oyaw = self.origin
        rotation = math.radians(self.start_pose['yaw_deg']) - oyaw
        c, s = math.cos(rotation), math.sin(rotation)
        return (self.start_pose['x'] + c * (x-ox) - s * (y-oy),
                self.start_pose['y'] + s * (x-ox) + c * (y-oy), wrap(yaw + rotation))

    def advance(self):
        self.index += 1
        self.entered = None
        self.context = {}
        if self.index >= len(self.steps):
            self.state = 'DONE'
            self.message = 'Misi selesai'
        return {}

    def tick(self):
        if self.state != 'RUNNING':
            return {}
        try:
            now = self.clock()
            step = self.steps[self.index]
            action = step['action']
            if self.entered is None:
                self.entered = now
                self.message = f'{self.index + 1}/{len(self.steps)} {action}'
            elapsed = now - self.entered
            if action == 'detect_camera':
                self.read('camera')  # A missing sensor is not a normal no-match timeout.
            if elapsed > step.get('timeout_s', 30):
                if action == 'detect_camera':
                    self.results[step.get('save_as', 'target')] = None
                    if step.get('on_not_found', 'stop') == 'continue':
                        return self.advance()
                raise ValueError(f'{action}: timeout')
            return getattr(self, 'do_' + action)(step, elapsed)
        except (ValueError, TypeError, KeyError, IndexError) as error:
            self.fail(str(error))
            return {}

    def do_wait(self, s, elapsed):
        return self.advance() if elapsed >= s['seconds'] else {}

    def do_stop(self, s, elapsed):
        self.index = len(self.steps)
        self.state = 'DONE'
        self.message = 'STOP: misi selesai'
        return {}

    def do_light(self, s, elapsed):
        self.light = (s['color'], s.get('mode', 'steady'))
        return self.advance()

    def base_command(self, vx, wz):
        # A front/rear stop also protects time-based movement. Missing LiDAR fails closed.
        scan = self.read('scan')
        direction = 'rear' if vx < 0 else 'front'
        clearance = scan.get(direction)
        if clearance is None:
            raise ValueError(f'LiDAR {direction}: tidak ada data valid')
        if clearance * 100 <= self.settings['front_stop_cm']:
            raise ValueError(f'LiDAR {direction}: penghalang terlalu dekat')
        return {'vx': vx, 'wz': max(-self.settings['angular_speed_rps'],
                                    min(self.settings['angular_speed_rps'], wz))}

    def do_goto(self, s, elapsed):
        x, y, yaw = self.pose()
        dx, dy = s['x'] - x, s['y'] - y
        distance = math.hypot(dx, dy)
        if distance <= self.settings['position_tolerance_m']:
            return self.advance()
        error = wrap(math.atan2(dy, dx) - yaw)
        speed = min(s.get('speed_mps', self.settings['linear_speed_mps']), distance * .6)
        return self.base_command(speed if abs(error) < math.radians(15) else 0,
                                 error * self.settings['heading_kp'])

    def do_robot_move(self, s, elapsed):
        self.pose()
        if elapsed >= s['duration_s']:
            return self.advance()
        return self.base_command(s['vx_mps'], 0)

    def travel(self, pose):
        previous = self.context.get('previous', pose)
        self.context['travel'] = self.context.get('travel', 0) + math.hypot(pose[0]-previous[0], pose[1]-previous[1])
        self.context['previous'] = pose
        return self.context['travel']

    def forward_done(self, s, pose):
        until = s['until']
        distance = self.travel(pose)
        if 'travel_cm' in until:
            return distance * 100 >= until['travel_cm']
        front = self.read('scan').get('front')
        if front is None:
            raise ValueError('LiDAR front tidak valid')
        return front * 100 <= until['lidar_front_cm']

    def do_forward(self, s, elapsed):
        pose = self.pose()
        if self.forward_done(s, pose):
            return self.advance()
        yaw = self.context.setdefault('yaw', pose[2])
        speed = s.get('speed_mps', self.settings['linear_speed_mps'])
        if 'lidar_front_cm' in s['until']:
            speed = min(speed, max(.02, (self.read('scan')['front'] - s['until']['lidar_front_cm']/100) * .6))
        return self.base_command(speed, wrap(yaw - pose[2]) * self.settings['heading_kp'])

    def do_trace(self, s, elapsed):
        pose = self.pose()
        if self.forward_done(s, pose):
            return self.advance()
        side = self.read('scan').get(s['side'])
        if side is None:
            raise ValueError('Dinding ' + s['side'] + ' tidak terdeteksi')
        error = side - s['wall_distance_cm']/100
        turn = error * self.settings['trace_kp'] * (1 if s['side'] == 'left' else -1)
        speed = s.get('speed_mps', self.settings['linear_speed_mps'])
        if 'lidar_front_cm' in s['until']:
            speed = min(speed, max(.02, (self.read('scan')['front'] - s['until']['lidar_front_cm']/100) * .6))
        return self.base_command(speed, turn)

    def do_heading(self, s, elapsed):
        yaw = self.pose()[2]
        error = wrap(math.radians(s['angle_deg']) - yaw)
        if abs(error) <= math.radians(self.settings['heading_tolerance_deg']):
            return self.advance()
        return self.base_command(0, error * self.settings['heading_kp'])

    do_robot_movetheta = do_heading

    def do_turn(self, s, elapsed):
        yaw = self.pose()[2]
        previous = self.context.get('yaw', yaw)
        self.context['turned'] = self.context.get('turned', 0) + wrap(yaw - previous)
        self.context['yaw'] = yaw
        target = math.radians(s['angle_deg']) * (1 if s['direction'] == 'left' else -1)
        error = target - self.context['turned']
        if abs(error) <= math.radians(self.settings['heading_tolerance_deg']):
            return self.advance()
        return self.base_command(0, error * self.settings['heading_kp'])

    def axis(self, axis, s):
        current = self.read(axis)
        scale = self.calibration[axis + ('_cm_per_rev' if axis == 'lift' else '_deg_per_rev')]
        field = 'delta_cm' if axis == 'lift' else 'angle_deg'
        if 'target' not in self.context:
            self.context['target'] = (self.snapshot[axis]['position'] if s.get('target') == 'default'
                                      else current + s[field]/scale)
        error = (self.context['target'] - current) * scale
        tolerance = self.settings['lift_tolerance_cm' if axis == 'lift' else 'rotate_tolerance_deg']
        # Stop on crossing the target rather than repeatedly reversing around it.
        previous = self.context.get('error', error)
        self.context['error'] = error
        if abs(error) <= tolerance or previous * error < 0:
            return self.advance()
        sign = (1 if error > 0 else -1) * self.calibration[axis + '_positive_duty_sign']
        if axis == 'lift':
            return {'lift_direction': sign}  # ROS adapter applies existing RPM PID, no boost.
        return {'rotate': sign * s.get('duty_percent', self.settings['rotate_duty_percent'])/100}

    def do_lift(self, s, elapsed):
        return self.axis('lift', s)

    def do_oms_rotate(self, s, elapsed):
        return self.axis('rotate', s)

    def do_slide(self, s, elapsed):
        if elapsed >= s['duration_s']:
            return self.advance()
        return {'slide': s.get('duty_percent', self.settings['slide_duty_percent']) * (1 if s['direction'] == 'forward' else -1)}

    def servo(self, name, s):
        if s['position'] == 'default':
            value = self.snapshot[name]['position']
        else:
            mapping = self.config.get('servo_positions', {}).get(name, {'on': 'left_value', 'off': 'right_value'})
            side = mapping[s['position']]
            if side not in ('left_value', 'right_value'):
                raise ValueError('servo_positions harus left_value/right_value')
            key = {('wrist', 'left_value'): 'r', ('wrist', 'right_value'): 't',
                   ('gripper', 'left_value'): 'y', ('gripper', 'right_value'): 'u'}[name, side]
            value = self.servos[key][1]
        sample = self.data.get('servo_' + name)
        confirmed = sample and self.clock() - sample[1] <= self.settings['feedback_timeout_s'] and sample[0] == value
        if confirmed:
            start = self.context.setdefault('confirmed_at', self.clock())
            if self.clock() - start >= s.get('settle_s', 1):
                return self.advance()
        else:
            self.context.pop('confirmed_at', None)
        return {'servo': (name, value)}

    def do_wrist(self, s, elapsed):
        return self.servo('wrist', s)

    def do_gripper(self, s, elapsed):
        return self.servo('gripper', s)

    def do_detect_camera(self, s, elapsed):
        # Missing frames fail as sensor errors; fresh empty frames mean 'not found'.
        result = self.read('camera')
        if self.data['camera'][1] < self.entered:
            return {}
        for obj in result.get('objects', []):
            if not isinstance(obj, dict):
                continue
            if str(obj.get('color', obj.get('label', ''))).casefold() != s['colour'].casefold():
                continue
            if 'shape' in s and obj.get('shape') != s['shape']:
                continue
            self.results[s.get('save_as', 'target')] = dict(obj)
            return self.advance()
        return {}
