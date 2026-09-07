"""Software-only tests: no ROS node, hardware server or PWM is started."""

import sys
import unittest
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/components/examples/python'))
from titan_keyboard_teleop import KeyboardCmdVel, StandardServoJog, ServoServiceCommand, parse_args


class StandardServoTest(unittest.TestCase):
    def test_uninitialized_state_and_explicit_initialization(self):
        servo = StandardServoJog(initial=20)
        for invalid in (-151, 151, float('nan'), float('inf')):
            servo.observe(invalid)
        self.assertIsNone(servo.target)
        self.assertIsNone(servo.jog(0, 0.02))
        with self.assertRaises(RuntimeError):
            servo.jog(25, 0.02)
        self.assertEqual(servo.initialize(), 20)
        self.assertIsNone(servo.initialize())

    def test_known_command_restored_not_recentered(self):
        servo = StandardServoJog()
        servo.observe(42)
        self.assertIsNone(servo.initialize())
        servo.observe(0)  # Delayed state must not reset the local ramp.
        self.assertEqual(servo.target, 42)

    def test_fractional_accumulation_and_idle_hold(self):
        servo = StandardServoJog()
        servo.observe(10)
        outputs = [servo.jog(1, 0.02) for _ in range(50)]
        self.assertAlmostEqual(servo.target, 11)
        self.assertIn(11, outputs)
        self.assertIsNone(servo.jog(0, 1))
        self.assertAlmostEqual(servo.target, 11)

    def test_bounds_reversal_and_delay_cap(self):
        servo = StandardServoJog(minimum=-10, maximum=10)
        servo.observe(0)
        self.assertEqual(servo.jog(100, 5), 5)  # No jump after a blocked loop.
        for _ in range(10):
            servo.jog(100, 0.05)
        self.assertEqual(servo.target, 10)
        for _ in range(10):
            servo.jog(-100, 0.05)
        self.assertEqual(servo.target, -10)

    def test_stop_never_publishes_standard_zero(self):
        node = SimpleNamespace(
            slide_publisher=Mock(), servo_commands={
                name: Mock() for name in ('wrist', 'gripper')},
            publish_cmd=Mock(), publish_oms=Mock(),
            standard_servos={'wrist': StandardServoJog(),
                             'gripper': StandardServoJog()}, servo_waiting=set())
        node.standard_servos['wrist'].observe(30)
        node.standard_servos['gripper'].observe(-25)
        node.publish_servos = MethodType(KeyboardCmdVel.publish_servos, node)
        node.publish_servos(40, 25, 0, dt=0.05)
        node.servo_commands['wrist'].set_target.assert_called_once_with(31)
        node.servo_commands['wrist'].reset_mock()
        with patch('titan_keyboard_teleop.rclpy.spin_once'):
            KeyboardCmdVel.stop(node)
        node.servo_commands['wrist'].set_target.assert_not_called()
        node.servo_commands['gripper'].set_target.assert_not_called()
        self.assertEqual(node.slide_publisher.publish.call_args.args[0].data, 0)
        self.assertEqual(node.publish_cmd.call_count, 5)
        self.assertEqual(node.publish_oms.call_count, 5)

    def test_cli_limits_and_slow_oms_keep_base_speed(self):
        with patch.object(sys, 'argv', ['teleop', '--linear-speed', '0.15',
                          '--angular-speed', '0.8', '--rotate-rpm', '10',
                          '--rotate-boost-time', '0']):
            args = parse_args()
        self.assertEqual(args.angular_speed, 0.8)
        self.assertEqual(args.rotate_rpm, 10)
        self.assertEqual(args.rotate_boost_time, 0)
        for flags in (['--wrist-min-angle', '-151'],
                      ['--gripper-min-angle', '30', '--gripper-max-angle', '20'],
                      ['--wrist-start-angle', '151']):
            with patch.object(sys, 'argv', ['teleop'] + flags), \
                    patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_args()


if __name__ == '__main__':
    unittest.main()


class YellowFollowIntegrationTest(unittest.TestCase):
    def test_z_follow_then_stop_or_manual_override(self):
        import titan_keyboard_teleop as teleop
        for override in ('z', 'e', 'x', 'w', 'i', 'g', 'p'):
            with self.subTest(override=override):
                node = Mock()
                node.human.stop_pressed = False
                node.human.owns_control = False
                node.yellow_follower.command.return_value = (0.35, 0.7)
                node.yellow_follower.status = 'mengikuti kuning'
                node.standard_servos = {}
                clock = iter(i * 0.02 for i in range(1000))
                with patch.object(sys, 'argv', ['teleop', '--linear-speed', '0.15']), \
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
                    stdin.read.side_effect = ['z', override, 'q']
                    teleop.main()
                commands = node.publish_cmd.call_args_list
                self.assertEqual(commands[-2].args, (0.15, 0.7))
                self.assertEqual(commands[-1].args,
                                 (0.15, 0.0) if override == 'w' else (0.0, 0.0))
                node.stop.assert_called()


class ServoServiceTest(unittest.TestCase):
    def test_service_uses_angle_and_coalesces_pending_jogs(self):
        client = Mock()
        future = client.call_async.return_value
        future.done.return_value = False
        sender = ServoServiceCommand(client, Mock(), 'wrist')
        sender.set_target(25)
        sender.pump(1)
        self.assertEqual(client.call_async.call_args.args[0].initparams.speed, 25)
        sender.set_target(26)
        sender.set_target(28)
        sender.pump(1.1)
        self.assertEqual(client.call_async.call_count, 1)
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(success=True)
        sender.pump(1.2)
        self.assertEqual(client.call_async.call_args.args[0].initparams.speed, 28)
        sender.pump(1.3)
        self.assertEqual(client.call_async.call_count, 2)

    def test_missing_service_and_rejection_retry_same_target(self):
        client = Mock()
        client.service_is_ready.return_value = False
        sender = ServoServiceCommand(client, Mock(), 'gripper')
        sender.set_target(-35)
        sender.pump(1)
        client.call_async.assert_not_called()
        client.service_is_ready.return_value = True
        sender.pump(2)
        client.call_async.return_value.done.return_value = True
        client.call_async.return_value.result.return_value = SimpleNamespace(
            success=False, message='failed')
        sender.pump(2.1)
        self.assertIsNone(sender.confirmed)
        sender.pump(2.7)
        self.assertEqual(client.call_async.call_count, 2)
        self.assertEqual(client.call_async.call_args.args[0].initparams.speed, -35)

    def test_first_key_initializes_only_selected_standard_servo(self):
        node = SimpleNamespace(
            slide_publisher=Mock(), get_logger=Mock(),
            servo_commands={name: Mock() for name in ('wrist', 'gripper')},
            standard_servos={'wrist': StandardServoJog(initial=40),
                             'gripper': StandardServoJog(initial=-20)})
        KeyboardCmdVel.publish_servos(node, 0, wrist=25, dt=.02)
        node.servo_commands['wrist'].set_target.assert_called_once_with(40)
        node.servo_commands['gripper'].set_target.assert_not_called()
        self.assertIsNone(node.standard_servos['gripper'].target)
        KeyboardCmdVel.publish_servos(node, 0, wrist=-25, dt=.05)
        self.assertLess(node.standard_servos['wrist'].target, 40)
        KeyboardCmdVel.publish_servos(node, 0, gripper=-15, dt=.05)
        node.servo_commands['gripper'].set_target.assert_called_once_with(-20)
        KeyboardCmdVel.publish_servos(node, 0, gripper=15, dt=.05)
        self.assertGreater(node.standard_servos['gripper'].target, -20)


class ServoPositionConfigTest(unittest.TestCase):
    def test_yaml_keys_command_exact_targets(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'servos.yaml'
            config.write_text('wrist: {left_value: 45, right_value: -20}\n'
                              'eof: {left_value: 60, right_value: 5}\n')
            with patch.object(sys, 'argv', ['teleop', '--servo-config', str(config)]):
                args = parse_args()
            node = SimpleNamespace(
                servo_positions=args.servo_positions,
                standard_servos={name: StandardServoJog() for name in ('wrist', 'gripper')},
                servo_commands={name: Mock() for name in ('wrist', 'gripper')},
                get_logger=Mock())
            for key, name, value in (('r', 'wrist', 45), ('t', 'wrist', -20),
                                     ('y', 'gripper', 60), ('u', 'gripper', 5)):
                KeyboardCmdVel.command_servo_position(node, key)
                self.assertEqual(node.standard_servos[name].target, value)
                node.servo_commands[name].set_target.assert_called_with(value)
            for contents in ('wrist: {}',
                             'wrist: {left_value: .nan, right_value: 0}',
                             'wrist: {left_value: 151, right_value: 0}'):
                config.write_text(contents)
                with patch.object(sys, 'argv', ['teleop', '--servo-config', str(config)]), \
                     patch('sys.stderr'), self.assertRaises(SystemExit):
                    parse_args()
