import importlib.util
import math
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / 'src/components/examples/python/yellow_follow_keyboard.py'
spec = importlib.util.spec_from_file_location('yellow_follow_keyboard', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_compute_auto_cmd_centered_at_target_distance():
    linear_x, angular_z = module.compute_auto_cmd(320.0, 640.0, 0.38)
    assert abs(linear_x) < 1e-6
    assert abs(angular_z) < 1e-6


def test_compute_auto_cmd_turns_toward_left():
    linear_x, angular_z = module.compute_auto_cmd(200.0, 640.0, 0.50)
    assert linear_x > 0.0
    assert angular_z > 0.0


def test_compute_auto_cmd_reverses_when_too_close():
    linear_x, angular_z = module.compute_auto_cmd(320.0, 640.0, 0.20)
    assert linear_x < 0.0
    assert abs(angular_z) < 1e-6


def detection(color='yellow', **values):
    return dict(color=color, pixel_x=160, distance_m=0.7, **values)


def follower_with(objects, width=320):
    import json
    from types import SimpleNamespace
    follower = module.YellowFollower()
    follower.result_callback(SimpleNamespace(data=json.dumps({
        'image_width': width, 'objects': objects})))
    return follower


def test_ignores_other_colors_and_uses_resized_image_width():
    follower = follower_with([detection('red'), detection(' Kuning ')])
    assert follower.command() == module.compute_auto_cmd(160, 320, 0.7)
    assert follower.command()[1] == 0
    assert follower_with([detection('red')]).command() == (0, 0)


def test_stops_after_tracker_timeout_or_target_loss():
    follower = follower_with([detection()])
    assert follower.command(follower.received_at + 0.51) == (0, 0)
    assert follower_with([]).command() == (0, 0)


def test_invalid_payload_clears_previous_target():
    from types import SimpleNamespace
    for payload in ('null', '[]', '123', '{bad', '{"objects": 1}'):
        follower = follower_with([detection()])
        follower.result_callback(SimpleNamespace(data=payload))
        assert follower.command() == (0, 0)


def test_invalid_geometry_or_depth_stops():
    for value in (None, 0, -1, float('nan'), float('inf')):
        obj = detection()
        obj['distance_m'] = value
        assert follower_with([obj]).command() == (0, 0)
    for width in (0, -1, float('nan'), float('inf')):
        assert follower_with([detection()], width).command() == (0, 0)


def test_stop_and_manual_keys_cancel_standalone_auto():
    from types import SimpleNamespace
    for key in ('x', 'e', 'w', 's', 'a', 'd'):
        node = SimpleNamespace(auto_mode=True)
        module.YellowFollowKeyboard._handle_key_press(node, key)
        assert not node.auto_mode
        if key in ('x', 'e'):
            assert node.manual_linear == node.manual_angular == 0



def test_missing_depth_turns_only_and_stops_when_centered():
    for depth in (None, 0, -1, float('nan'), float('inf'), 'bad'):
        for x, sign in ((80, 1), (160, 0), (240, -1)):
            obj = dict(color='yellow', pixel_x=x, distance_m=depth)
            follower = follower_with([obj])
            vx, wz = follower.command()
            assert vx == 0
            assert (wz > 0) - (wz < 0) == sign
            assert 'hanya putar' in follower.status
            assert follower.command(follower.received_at + 0.51) == (0, 0)
    assert follower_with([dict(color='kuning', pixel_x=80)]).command()[1] > 0


def test_depth_loss_removes_translation_and_depth_recovery_restores_it():
    import json
    from types import SimpleNamespace
    follower = follower_with([dict(color='yellow', pixel_x=80, distance_m=0.7)])
    assert follower.command()[0] > 0
    for depth in (None, 0.7):
        follower.result_callback(SimpleNamespace(data=json.dumps({
            'image_width': 320, 'objects': [dict(color='yellow', pixel_x=80, distance_m=depth)]})))
        vx, wz = follower.command()
        assert (vx == 0) if depth is None else (vx > 0)
        assert wz > 0
