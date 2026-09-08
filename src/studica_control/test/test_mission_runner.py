"""Deterministic mission simulation: no ROS node or hardware movement."""
import math
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/components/examples/python'))
from mission_runner import MissionRunner
from human_interaction import HumanInteraction

SERVOS = {'r': ('wrist', 30), 't': ('wrist', -20),
          'y': ('gripper', 50), 'u': ('gripper', 10)}
SNAPSHOT = {name: {'position': 0} for name in ('lift', 'rotate', 'wrist', 'gripper')}


def make(steps, **config):
    now = [0.0]
    runner = MissionRunner(dict(mission=steps, **config), lambda: now[0], SERVOS)
    return runner, now


def sensors(runner, pose=(0, 0, 0), front=2, side=.3):
    runner.observe('odom', pose)
    runner.observe('imu', True)
    for i in range(4):
        runner.observe(f'wheel_{i}', 0)
    runner.observe('scan', dict(front=front, rear=2, left=side, right=side))


def test_default_and_examples_are_valid_yaml():
    root = Path(__file__).resolve().parents[3]
    for filename in ('human_interaction.yaml', 'human_interaction_examples.yaml'):
        config = yaml.safe_load((root / 'config' / filename).read_text())
        runner = MissionRunner(config, lambda: 0, SERVOS)
        assert runner.steps
    with pytest.raises(ValueError, match='calibration'):
        runner.start(SNAPSHOT)


def test_timed_light_waits_then_turns_off():
    r, clock = make([dict(action='light', color='yellow', mode='blink',
                          duration_s=5, timeout_s=10), dict(action='stop')])
    r.start(SNAPSHOT)
    assert r.tick() == {}
    assert r.light == ('yellow', 'blink') and r.index == 0
    clock[0] = 4.9
    r.tick()
    assert r.light == ('yellow', 'blink') and r.index == 0
    clock[0] = 5
    r.tick()
    assert r.light == ('off', 'steady') and r.index == 1


@pytest.mark.parametrize('duration', [0, -1, float('nan'), 301, 30])
def test_invalid_light_duration(duration):
    with pytest.raises(ValueError):
        r, _ = make([dict(action='light', color='yellow', duration_s=duration)])
        r.start(SNAPSHOT)


def test_light_wait_done_and_cancel():
    r, clock = make([dict(action='light', color='red'), dict(action='wait', seconds=1), dict(action='stop')])
    r.start(SNAPSHOT)
    assert r.tick() == {} and r.light == ('red', 'steady')
    r.tick()
    clock[0] = 1
    r.tick()
    r.tick()
    assert r.state == 'DONE'
    r.cancel()
    assert r.state == 'IDLE' and r.light[0] == 'off'


@pytest.mark.parametrize('step', [
    dict(action='shell', command='echo bad'),
    dict(action='robot_move', vx_mps=.15),
    dict(action='robot_move', vx_mps=float('nan'), duration_s=1),
    dict(action='forward', until={'lidar_front_cm': 5}),
    dict(action='trace', side='wrong', wall_distance_cm=30, until={'travel_cm': 100}),
    dict(action='wrist', position=True),
    dict(action='light', color='red', typo=1),
    dict(action='slide', direction='forward', duration_s=10, timeout_s=5),
])
def test_invalid_mission_fails_before_start(step):
    with pytest.raises(ValueError):
        make([step])


def test_coordinate_transform_goto_and_absolute_heading():
    r, clock = make([dict(action='goto', x=2, y=2), dict(action='heading', angle_deg=90)],
                    start_pose=dict(x=1, y=2, yaw_deg=0))
    sensors(r, (10, 20, math.pi/2))
    r.start(SNAPSHOT)
    assert r.tick()['vx'] > 0
    clock[0] = .1
    sensors(r, (10, 21, math.pi/2))
    assert r.tick() == {}
    assert r.tick()['wz'] > 0
    sensors(r, (10, 21, math.pi))
    assert r.tick() == {} and r.state == 'DONE'


def test_turn_270_uses_relative_unwrapped_angle():
    r, clock = make([dict(action='turn', direction='left', angle_deg=270)])
    sensors(r)
    r.start(SNAPSHOT)
    assert r.tick()['wz'] > 0
    for angle in (90, 170, -100, -90):
        clock[0] += .1
        sensors(r, (0, 0, math.radians(angle)))
        out = r.tick()
    assert out == {} and r.state == 'DONE'


def test_forward_distance_and_trace_sign_and_lidar_stop():
    r, clock = make([dict(action='forward', until=dict(travel_cm=100)),
                     dict(action='trace', side='right', wall_distance_cm=30,
                          until=dict(lidar_front_cm=10))])
    sensors(r)
    r.start(SNAPSHOT)
    assert r.tick()['vx'] > 0
    sensors(r, (1, 0, 0), side=.5)
    r.tick()
    assert r.tick()['wz'] < 0
    sensors(r, (1, 0, 0), front=.10)
    assert r.tick() == {} and r.state == 'DONE'


def test_stale_and_obstacle_stop_outputs():
    r, clock = make([dict(action='robot_move', vx_mps=.1, duration_s=2)])
    sensors(r)
    r.start(SNAPSHOT)
    assert r.tick()['vx'] == .1
    clock[0] = 1
    assert r.tick() == {} and r.state == 'ERROR'
    sensors(r, front=.04)
    r.start(SNAPSHOT)
    assert r.tick() == {} and r.state == 'ERROR'


def test_oms_signed_scale_default_pid_direction_and_slide_delay():
    cal = dict(lift_cm_per_rev=-2, rotate_deg_per_rev=-180,
               lift_positive_duty_sign=-1, rotate_positive_duty_sign=-1)
    r, clock = make([dict(action='lift', delta_cm=20), dict(action='lift', target='default'),
                     dict(action='oms_rotate', angle_deg=-90),
                     dict(action='slide', direction='backward', duration_s=1)], calibration=cal)
    r.observe('lift', 0)
    r.start(SNAPSHOT)
    assert r.tick() == dict(lift_direction=-1)
    r.observe('lift', -10)
    assert r.tick() == {}
    assert r.tick() == dict(lift_direction=1)
    r.observe('lift', 0)
    r.tick()
    r.observe('rotate', 0)
    assert r.tick()['rotate'] == pytest.approx(.333)
    r.observe('rotate', .5)
    r.tick()
    assert r.tick()['slide'] == -15
    clock[0] += 1
    assert r.tick() == {} and r.state == 'DONE'


def test_servo_waits_ack_and_settle():
    r, clock = make([dict(action='gripper', position='on', settle_s=1)])
    r.start(SNAPSHOT)
    assert r.tick()['servo'] == ('gripper', 50)
    r.observe('servo_gripper', 50)
    r.tick()
    clock[0] += 1
    r.observe('servo_gripper', 50)
    assert r.tick() == {} and r.state == 'DONE'


def test_camera_matches_same_object_and_rejects_old_frame():
    r, clock = make([dict(action='detect_camera', colour='yellow', shape='rectangle')])
    r.observe('camera', {'objects': [{'color': 'yellow', 'shape': 'rectangle'}]})
    clock[0] = .1
    r.start(SNAPSHOT)
    assert r.tick() == {} and r.state == 'RUNNING'
    r.observe('camera', {'objects': [{'color': 'yellow', 'shape': 'circle'},
                                   {'color': 'red', 'shape': 'rectangle'}]})
    r.tick()
    assert r.state == 'RUNNING'
    r.observe('camera', {'objects': [{'color': 'yellow', 'shape': 'rectangle', 'distance_m': .4}]})
    r.tick()
    assert r.state == 'DONE' and r.results['target']['distance_m'] == .4


def test_not_found_timeout_continue_and_cancel():
    r, clock = make([dict(action='detect_camera', colour='yellow', timeout_s=1, on_not_found='continue'),
                     dict(action='wait', seconds=1)])
    r.start(SNAPSHOT)
    r.observe('camera', {'objects': []})
    r.tick()
    clock[0] = 1.1
    r.observe('camera', {'objects': []})
    r.tick()
    assert r.index == 1 and r.results['target'] is None
    r.cancel()
    assert r.tick() == {}


def test_start_button_runs_only_after_capture_and_new_edge(tmp_path):
    r, clock = make([dict(action='wait', seconds=1)])
    h = HumanInteraction(tmp_path/'default.json', lambda: clock[0], runner=r)
    h.button('start', False)
    h.button('start', True)
    assert r.state == 'IDLE'
    h.capture()
    clock[0] = .4
    h.observe('lift', 0); h.observe('rotate', 0)
    h.tick(dict(wrist=0, gripper=0))
    h.button('start', True)
    assert r.state == 'IDLE'
    h.button('start', False); h.button('start', True)
    assert r.state == h.state == 'RUNNING'
    h.button('stop', True)
    assert r.state == 'IDLE' and h.state == 'MANUAL'


def test_camera_timeout_cannot_hide_sensor_loss():
    r, clock = make([dict(action='detect_camera', colour='red', timeout_s=1, on_not_found='continue')])
    r.start(SNAPSHOT)
    r.observe('camera', {'objects': []})
    r.tick()
    clock[0] = 2
    assert r.tick() == {} and r.state == 'ERROR'


def test_scan_invalid_returns_missing_not_infinite_clearance():
    from mission_ros import scan_sectors
    from types import SimpleNamespace
    scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/2,
                           range_min=.02, range_max=10,
                           ranges=[2, .3, float('nan'), float('inf'), 2])
    result = scan_sectors(scan)
    assert result['front'] is None and result['left'] is None
    assert result['right'] == .3 and result['rear'] == 2


def test_adapter_lift_pid_failure_emits_zero_to_all_motors():
    from mission_ros import MissionROS
    from types import SimpleNamespace
    from unittest.mock import Mock
    r, clock = make([dict(action='lift', delta_cm=20)],
                    calibration=dict(lift_cm_per_rev=2, lift_positive_duty_sign=-1))
    r.observe('lift', 0)
    r.start(SNAPSHOT)
    node = SimpleNamespace(servo_commands={}, lift_pid=Mock(),
                           publish_cmd=Mock(), publish_oms=Mock(), publish_servos=Mock())
    node.lift_pid.calculate.side_effect = RuntimeError('RPM stale')
    adapter = SimpleNamespace(node=node, runner=r,
                               args=SimpleNamespace(lift_up_rpm=30, lift_down_rpm=30, slide_polarity=1),
                               update_lights=Mock(), results_pub=Mock(), last_results=None)
    MissionROS.tick(adapter)
    assert r.state == 'ERROR'
    node.publish_cmd.assert_called_once_with(0, 0)
    node.publish_oms.assert_called_once_with(0, 0)
    node.publish_servos.assert_called_once_with(0)


def test_keyboard_does_not_overwrite_running_mission_with_manual_zero():
    import titan_keyboard_teleop as teleop
    from types import SimpleNamespace
    from unittest.mock import Mock, patch
    node = Mock()
    node.human = SimpleNamespace(state='RUNNING', owns_control=True,
                                 stop_pressed=False, tick=Mock(), message='running')
    node.mission.runner.state = 'RUNNING'
    node.mission.runner.message = 'forward'
    node.mission.tick.side_effect = lambda: node.publish_cmd(.12, .1)
    node.standard_servos = {}
    node.human_last_status = None
    clock = iter(i*.02 for i in range(1000))
    with patch.object(sys, 'argv', ['teleop']), \
         patch.object(teleop, 'KeyboardCmdVel', return_value=node), \
         patch.object(teleop.rclpy, 'init'), \
         patch.object(teleop.rclpy, 'spin_once'), \
         patch.object(teleop.rclpy, 'ok', return_value=True), \
         patch.object(teleop.rclpy, 'shutdown'), \
         patch.object(teleop.termios, 'tcgetattr'), \
         patch.object(teleop.termios, 'tcsetattr'), \
         patch.object(teleop.tty, 'setcbreak'), \
         patch.object(teleop.select, 'select', return_value=([1], [], [])), \
         patch.object(teleop.sys, 'stdin') as stdin, \
         patch.object(teleop.time, 'monotonic', side_effect=lambda: next(clock)):
        stdin.read.side_effect = ['w', 'q']
        teleop.main()
    node.mission.tick.assert_called_once()
    assert node.publish_cmd.call_args_list[-1].args == (.12, .1)
    node.mission.stop.assert_called_once()
    node.stop.assert_called_once()
