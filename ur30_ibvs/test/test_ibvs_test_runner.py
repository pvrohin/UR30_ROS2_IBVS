import importlib.util
import math
import os
import random

_path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'ibvs_test_runner.py')
_spec = importlib.util.spec_from_file_location('ibvs_test_runner', _path)
runner = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(runner)

LIMITS = {
    'center_x': 0.8, 'center_y': 0.0, 'x_range': 0.10, 'y_range': 0.02,
    'yaw_range': math.radians(4.0), 'cam_x': 0.8, 'cam_y': 0.0, 'half_w': 0.6,
    'half_h': 0.3375, 'margin': 0.015, 'min_x': 0.32, 'min_separation': 0.05,
    'min_yaw_separation': math.radians(2.0),
}


def test_nominal_panel_fits_and_far_or_turned_panel_does_not():
    assert runner.footprint_fits(0.8, 0.0, 0.0, 0.8, 0.0, 0.6, 0.3375, 0.015)
    # 0.06 m off in y: the 0.55 m panel edge is 0.335 m from the axis, outside 0.3375-0.015
    assert not runner.footprint_fits(0.8, 0.06, 0.0, 0.8, 0.0, 0.6, 0.3375, 0.015)
    # yaw 10 deg makes the panel 0.72 m tall in the image, more than the 0.675 m view
    assert not runner.footprint_fits(0.8, 0.0, math.radians(10), 0.8, 0.0, 0.6, 0.3375, 0.0)


def test_panel_must_stay_clear_of_the_robot_side():
    # the failing case from the first run: x = 0.737 puts the near edge at 0.287
    assert not runner.clear_of_robot(0.737, math.radians(1.0), 0.32)
    assert runner.clear_of_robot(0.8, 0.0, 0.32)
    # yaw widens the footprint: 0.79 at 3.4 deg reaches 0.323, just clear
    assert runner.clear_of_robot(0.79, math.radians(3.4), 0.32)
    assert not runner.clear_of_robot(0.77, math.radians(4.0), 0.32)


def test_samples_are_inside_the_view_and_in_range():
    rng = random.Random(1)
    previous = None
    for _ in range(200):
        pose = runner.sample_pose(rng, LIMITS, previous)
        assert pose is not None
        x, y, yaw = pose
        assert abs(x - 0.8) <= 0.10 and abs(y) <= 0.02 and abs(yaw) <= math.radians(4.0)
        assert runner.footprint_fits(x, y, yaw, 0.8, 0.0, 0.6, 0.3375, 0.015)
        assert runner.clear_of_robot(x, yaw, 0.32)
        previous = pose


def test_consecutive_samples_differ():
    rng = random.Random(2)
    previous = runner.sample_pose(rng, LIMITS)
    for _ in range(100):
        pose = runner.sample_pose(rng, LIMITS, previous)
        moved = math.hypot(pose[0] - previous[0], pose[1] - previous[1]) >= 0.05
        turned = abs(pose[2] - previous[2]) >= math.radians(2.0)
        assert moved or turned
        previous = pose


def test_same_seed_gives_same_poses():
    a = [runner.sample_pose(random.Random(7), LIMITS) for _ in range(3)]
    b = [runner.sample_pose(random.Random(7), LIMITS) for _ in range(3)]
    assert a == b


def test_impossible_limits_return_none():
    tight = dict(LIMITS, half_h=0.1)
    assert runner.sample_pose(random.Random(0), tight) is None


def test_expected_camera_over_the_plus_y_edge():
    (x, y, z), x_axis = runner.expected_camera(0.8, 0.0, 0.0, 0.805, 0.2)
    assert (round(x, 6), round(y, 6), round(z, 6)) == (0.8, 0.275, 1.005)
    assert x_axis == (1.0, 0.0)
    # panel turned +90 deg: its +y edge is then at -x
    (x, y, z), x_axis = runner.expected_camera(0.8, 0.0, math.pi / 2, 0.805, 0.2)
    assert abs(x - (0.8 - 0.275)) < 1e-9 and abs(y) < 1e-9
    assert abs(x_axis[0]) < 1e-9 and abs(x_axis[1] - 1.0) < 1e-9


def looking_down(yaw):
    # optical z = world -Z, optical x = (cos yaw, sin yaw, 0), y = z cross x
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]]


def test_pose_errors_zero_at_the_expected_pose():
    R = looking_down(math.radians(3))
    e = runner.pose_errors((0.8, 0.275, 1.005), R, (0.8, 0.275, 1.005),
                           (math.cos(math.radians(3)), math.sin(math.radians(3))))
    assert all(abs(v) < 1e-9 for v in e)


def test_pose_errors_report_offsets_and_angles():
    R = looking_down(math.radians(5))
    e = runner.pose_errors((0.803, 0.275, 1.009), R, (0.8, 0.275, 1.005), (1.0, 0.0))
    position, lateral, height, yaw, tilt = e
    assert abs(lateral - 0.003) < 1e-9 and abs(height - 0.004) < 1e-9
    assert abs(position - 0.005) < 1e-9
    assert abs(yaw - 5.0) < 1e-6 and abs(tilt) < 1e-6


def test_quaternion_matrix_and_tilt():
    yaw = math.radians(30)
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    R = runner.quat_to_matrix(q)
    assert abs(R[0][0] - math.cos(yaw)) < 1e-9 and abs(R[1][0] - math.sin(yaw)) < 1e-9
    # a rotation of 180 deg about x looks straight down, 190 deg is tilted 10 deg
    t = math.radians(190)
    R = runner.quat_to_matrix((math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)))
    _, _, _, _, tilt = runner.pose_errors((0, 0, 0), R, (0, 0, 0), (1.0, 0.0))
    assert abs(tilt - 10.0) < 1e-6


def test_gz_request_text():
    text = runner.gz_pose_request('door_panel_test', 0.8, 0.0, 0.79, math.pi)
    assert 'name: "door_panel_test"' in text and 'z: 0.790000' in text
    assert 'w: 0.000000' in text
