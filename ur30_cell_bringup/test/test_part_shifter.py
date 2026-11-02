import importlib.util
import math
import os
import random

_path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'part_shifter.py')
_spec = importlib.util.spec_from_file_location('part_shifter', _path)
shifter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(shifter)

NOMINAL = (0.8, 0.0, 0.0)


def test_smoothstep_has_flat_ends_and_is_clamped():
    assert shifter.smoothstep(-1.0) == 0.0
    assert shifter.smoothstep(2.0) == 1.0
    assert shifter.smoothstep(0.5) == 0.5
    assert shifter.smoothstep(0.1) < 0.1  # slow start
    assert shifter.smoothstep(0.9) > 0.9  # slow finish


def test_interpolation_reaches_the_target_and_zero_ramp_jumps():
    a, b = (0.8, 0.0, 0.0), (0.82, -0.01, 0.03)
    assert shifter.interpolate_pose(a, b, 0.0, 1.0) == a
    assert shifter.interpolate_pose(a, b, 1.0, 1.0) == b
    mid = shifter.interpolate_pose(a, b, 0.5, 1.0)
    assert all(abs(m - (x + y) / 2) < 1e-12 for m, x, y in zip(mid, a, b))
    assert shifter.interpolate_pose(a, b, 0.0, 0.0) == b


def test_random_poses_stay_within_the_limits():
    rng = random.Random(1)
    previous = None
    for _ in range(200):
        pose = shifter.random_pose(rng, NOMINAL, 0.02, math.radians(2.0), previous)
        assert abs(pose[0] - 0.8) <= 0.02 and abs(pose[1]) <= 0.02
        assert abs(pose[2]) <= math.radians(2.0)
        previous = pose


def test_consecutive_random_poses_differ_visibly():
    rng = random.Random(2)
    previous = NOMINAL
    for _ in range(100):
        pose = shifter.random_pose(rng, NOMINAL, 0.02, math.radians(2.0), previous, 0.005)
        moved = math.hypot(pose[0] - previous[0], pose[1] - previous[1]) >= 0.005
        turned = abs(pose[2] - previous[2]) >= math.radians(0.5)
        assert moved or turned
        previous = pose


def test_same_seed_gives_same_poses():
    a = [shifter.random_pose(random.Random(5), NOMINAL, 0.02, 0.03) for _ in range(3)]
    b = [shifter.random_pose(random.Random(5), NOMINAL, 0.02, 0.03) for _ in range(3)]
    assert a == b


def test_impossible_limits_return_none():
    assert shifter.random_pose(random.Random(1), NOMINAL, 0.0, 0.0, NOMINAL) is None


def test_panel_corners_and_boundary_distance():
    corners = shifter.panel_corners(0.8, 0.0, 0.0)
    assert corners[0] == (0.8 - 0.45, -0.275)
    assert corners[2] == (0.8 + 0.45, 0.275)
    # On an edge, at a corner, beside the edge, and in the middle (nearest side 0.275).
    assert shifter.distance_to_polygon_boundary(0.8, 0.275, corners) < 1e-12
    assert shifter.distance_to_polygon_boundary(0.35, 0.275, corners) < 1e-12
    assert abs(shifter.distance_to_polygon_boundary(0.8, 0.3, corners) - 0.025) < 1e-12
    assert abs(shifter.distance_to_polygon_boundary(0.8, 0.0, corners) - 0.275) < 1e-12
    # Beyond a corner the nearest point is the corner itself.
    assert abs(shifter.distance_to_polygon_boundary(1.3, 0.275, corners) - 0.05) < 1e-12


def test_boundary_distance_follows_a_turned_panel():
    corners = shifter.panel_corners(0.8, 0.0, math.radians(90.0))
    # Turned by 90 degrees, the long sides are along y: x = 0.8 +- 0.275.
    assert abs(shifter.distance_to_polygon_boundary(0.8 + 0.275, 0.1, corners)) < 1e-9


def test_ground_point_below_a_downward_camera():
    looking_down = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    point = shifter.ground_point((0.8, 0.1, 1.005), looking_down, 0.805)
    assert abs(point[0] - 0.8) < 1e-12 and abs(point[1] - 0.1) < 1e-12
    sideways = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    assert shifter.ground_point((0, 0, 1.0), sideways, 0.8) is None


def test_gz_request_text():
    text = shifter.gz_pose_request('panel', 0.8, 0.0, 0.79, 0.0)
    assert 'name: "panel"' in text and 'w: 1.000000' in text
