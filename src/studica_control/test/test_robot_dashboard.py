"""Run with ROS sourced: python3 -m unittest discover -s .../test -p test_robot_dashboard.py."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

SCRIPT = Path(__file__).resolve().parents[1] / 'src/components/examples/python/robot_dashboard.py'
spec = importlib.util.spec_from_file_location('robot_dashboard', SCRIPT)
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


class DashboardTests(unittest.TestCase):
    def test_direction_and_limits(self):
        self.assertEqual(gui.duty_value(25, 1), 0.25)
        self.assertEqual(gui.duty_value(25, -1), -0.25)
        self.assertEqual(gui.duty_value(25, 1, True), -0.25)
        for value in (-1, 101, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                gui.duty_value(value, 1)

    def dashboard(self):
        app = gui.Dashboard.__new__(gui.Dashboard)
        app.active = None
        app.titan = '/test_titan'
        app.armed = Mock()
        app.armed.get.return_value = True
        app.pubs = [Mock() for _ in range(4)]
        for pub in app.pubs:
            pub.get_subscription_count.return_value = 1
        app.node = Mock()
        app.node.get_publishers_info_by_topic.return_value = []
        app.duties = [Mock() for _ in range(4)]
        app.inverted = [Mock() for _ in range(4)]
        for duty, invert in zip(app.duties, app.inverted):
            duty.get.return_value = 10
            invert.get.return_value = False
        app.record = Mock()
        app.titan_call = Mock()
        return app

    def test_no_motion_when_disarmed_or_disconnected(self):
        app = self.dashboard()
        app.armed.get.return_value = False
        app.start_motion(0, 1)
        app.pubs[0].publish.assert_not_called()
        app.armed.get.return_value = True
        app.pubs[0].get_subscription_count.return_value = 0
        app.start_motion(0, 1)
        app.pubs[0].publish.assert_not_called()

    def test_competing_publisher_blocks_motion(self):
        app = self.dashboard()
        app.node.get_publishers_info_by_topic.return_value = [Mock(node_name='teleop')]
        app.start_motion(0, 1)
        app.pubs[0].publish.assert_not_called()

    def test_release_stops_correct_motor(self):
        app = self.dashboard()
        app.start_motion(2, -1)
        self.assertEqual(app.pubs[2].publish.call_args.args[0].data, -0.1)
        app.stop_motion()
        self.assertEqual(app.pubs[2].publish.call_args.args[0].data, 0.0)
        app.titan_call.assert_called_with('stop', 2)
        self.assertIsNone(app.active)

    def test_emergency_disables_controller(self):
        app = self.dashboard()
        app.emergency_stop()
        for pub in app.pubs:
            self.assertEqual(pub.publish.call_args.args[0].data, 0.0)
        app.armed.set.assert_called_with(False)
        app.titan_call.assert_called_with('disable')

    def test_buzzer_output_and_disconnect(self):
        app = self.dashboard()
        pub = Mock()
        app.digital_pubs = {'buzzer': (pub, Mock())}
        pub.get_subscription_count.return_value = 0
        app.digital_write('buzzer', True)
        pub.publish.assert_not_called()
        pub.get_subscription_count.return_value = 1
        app.digital_write('buzzer', True)
        self.assertTrue(pub.publish.call_args.args[0].data)
        app.digital_write('buzzer', False)
        self.assertFalse(pub.publish.call_args.args[0].data)

    def test_large_image_summary_is_bounded(self):
        from sensor_msgs.msg import Image, LaserScan
        image = Image(width=640, height=480, encoding='rgb8')
        self.assertEqual(gui.summarize(image), '640 × 480; rgb8')
        scan = LaserScan(range_min=0.1, range_max=10.0, ranges=[float('inf'), 0.01, 2.0])
        self.assertIn('2.000 m', gui.summarize(scan))


if __name__ == '__main__':
    unittest.main()
