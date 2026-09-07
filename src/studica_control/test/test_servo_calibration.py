import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/components/examples/python'))
from servo_calibration import Calibration, save_positions
from titan_keyboard_teleop import StandardServoJog


def test_save_preserves_other_endpoint_and_options(tmp_path):
    path = tmp_path / 'servos.yaml'
    path.write_text('wrist: {left_value: 1, right_value: 2}\n'
                    'eof: {left_value: 3, right_value: 4}\nextra: true\n')
    save_positions(path, {('wrist', 'left_value'): 42})
    config = yaml.safe_load(path.read_text())
    assert config['wrist'] == dict(left_value=42, right_value=2)
    assert config['eof']['right_value'] == 4
    assert config['extra'] is True
    original = path.read_text()
    with pytest.raises(ValueError):
        save_positions(path, {('eof', 'left_value'): float('nan')})
    assert path.read_text() == original


def test_step_limits_mark_only_after_ack(tmp_path):
    node = SimpleNamespace(
        positions={name: StandardServoJog() for name in ('wrist', 'eof')},
        senders={name: Mock(confirmed=None) for name in ('wrist', 'eof')},
        marks={}, step=1, path=tmp_path / 'servos.yaml')
    Calibration.key(node, 'r')
    assert node.positions['wrist'].target == 0
    Calibration.key(node, '1')
    assert node.marks == {}
    Calibration.key(node, 'r')
    node.senders['wrist'].confirmed = 1
    Calibration.key(node, '1')
    assert node.marks['wrist', 'left_value'] == 1
    node.positions['wrist'].target = 150
    Calibration.key(node, 'r')
    assert node.positions['wrist'].target == 150
    Calibration.key(node, 't')
    assert node.positions['wrist'].target == 149
    assert node.positions['eof'].target is None
