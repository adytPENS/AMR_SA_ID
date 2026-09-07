"""Preparation mode tests without ROS or motor commands."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/components/examples/python'))
from human_interaction import HumanInteraction


def make(tmp_path):
    now = [10.0]
    return HumanInteraction(tmp_path / 'default.json', lambda: now[0]), now


def test_capture_requires_new_feedback_and_preserves_units(tmp_path):
    mode, now = make(tmp_path)
    mode.observe('lift', 1)
    mode.observe('rotate', 2)
    mode.capture()
    now[0] += .4
    mode.tick({'wrist': 20, 'gripper': -10})
    assert mode.state == mode.CAPTURE
    mode.observe('lift', 1.2)
    mode.observe('rotate', 2.3)
    mode.tick({'wrist': 20, 'gripper': -10})
    assert mode.state == mode.READY
    data = json.loads(mode.path.read_text())
    assert data['lift']['position'] == 1.2
    assert data['rotate']['unit'] == 'motor_output_revolutions'
    assert data['wrist']['source'] == 'commanded_target_not_measured'
    assert data['slide']['position'] is None
    assert data['restore_gripper_when_holding'] is False
    mode.capture()
    assert mode.state == mode.READY


def test_missing_feedback_does_not_overwrite_default(tmp_path):
    mode, now = make(tmp_path)
    mode.path.write_text('existing')
    mode.capture()
    now[0] += 4
    mode.observe('lift', float('nan'))
    mode.tick({'wrist': None, 'gripper': None})
    assert mode.state == mode.ERROR
    assert mode.path.read_text() == 'existing'


def test_stop_cancel_and_held_start(tmp_path):
    mode, now = make(tmp_path)
    mode.button('stop', True)
    mode.capture()
    assert mode.state == mode.MANUAL
    mode.button('stop', False)
    mode.capture()
    now[0] += .4
    for name in ('lift', 'rotate'):
        mode.observe(name, 0)
    mode.tick({'wrist': 0, 'gripper': 0})
    mode.button('start', True)
    assert 'menunggu START' in mode.message
    mode.button('start', False)
    mode.button('start', True)
    assert 'START ditahan' in mode.message
    assert mode.state == mode.READY
    mode.button('stop', True)
    assert not mode.owns_control
    assert mode.path.exists()


def test_write_error_keeps_mode_stopped(tmp_path):
    mode, now = make(tmp_path)
    mode.path = tmp_path / 'directory'
    mode.path.mkdir()
    mode.capture()
    now[0] += .4
    for name in ('lift', 'rotate'):
        mode.observe(name, 0)
    mode.tick({'wrist': 0, 'gripper': 0})
    assert mode.state == mode.ERROR
    assert mode.owns_control
    assert list(tmp_path.iterdir()) == [mode.path]


def test_slide_delay_configuration_persisted(tmp_path):
    import pytest
    now = [0.0]
    slide = dict(mode='timed', extend_duty_percent=40, extend_seconds=1.2,
                 retract_duty_percent=-35, retract_seconds=1.5)
    mode = HumanInteraction(tmp_path / 'default.json', lambda: now[0], slide=slide)
    mode.capture()
    now[0] = .4
    mode.observe('lift', 0)
    mode.observe('rotate', 0)
    mode.tick({'wrist': 0, 'gripper': 0})
    assert mode.snapshot['slide']['timing'] == slide
    assert mode.snapshot['slide']['source'] == 'manual_reference_at_M'
    with pytest.raises(ValueError):
        HumanInteraction(mode.path, lambda: 0, slide=dict(slide, extend_seconds=-1))
