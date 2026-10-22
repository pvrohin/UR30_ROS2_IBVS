#!/usr/bin/env python3
"""Nudges the door panel at random while the dispense lap runs, and scores the lap.

Watches the IBVS node's state. While it is in DISPENSE, every few seconds the panel
(a separate Gazebo model, see door_panel_model.xacro) slides to a new random pose close to
its nominal one, over ramp_sec (0 = it jumps), using the gz set_pose service. Meanwhile the
point on the table under the camera's optical axis is compared with the panel's true
outline, which only this node knows: that distance is the lateral tracking error.

The lap passes when the IBVS node reaches DONE and the error, once the panel has been
still for settle_sec, stayed within tol_position. The error during a nudge is reported
too (it is larger: the arm lags the part) but is not what is judged.

The node needs the cell launched with include_panel:=false and the IBVS node with
dispense_enabled:=true; ur30_ibvs.launch.py does both when demo:=true.
"""
import json
import math
import random
import subprocess
import sys

PANEL_LENGTH = 0.9
PANEL_WIDTH = 0.55


# ---- pure helpers (no ROS), unit tested in test/test_part_shifter.py ----------------


def smoothstep(t):
    """0 -> 1 with zero slope at both ends, t clamped to [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def interpolate_pose(start, end, t, ramp_sec):
    """Pose (x, y, yaw) t seconds into a move from `start` to `end` that takes ramp_sec."""
    k = 1.0 if ramp_sec <= 0.0 else smoothstep(t / ramp_sec)
    return tuple(a + k * (b - a) for a, b in zip(start, end))


def random_pose(rng, nominal, max_xy, max_yaw, previous=None, min_step=0.005,
                min_yaw_step=math.radians(0.5), tries=1000):
    """A pose within max_xy / max_yaw of `nominal`, visibly different from `previous`."""
    for _ in range(tries):
        pose = (nominal[0] + rng.uniform(-max_xy, max_xy),
                nominal[1] + rng.uniform(-max_xy, max_xy),
                nominal[2] + rng.uniform(-max_yaw, max_yaw))
        if previous is None:
            return pose
        moved = math.hypot(pose[0] - previous[0], pose[1] - previous[1]) >= min_step
        turned = abs(pose[2] - previous[2]) >= min_yaw_step
        if moved or turned:
            return pose
    return None


def panel_corners(x, y, yaw, length=PANEL_LENGTH, width=PANEL_WIDTH):
    """Corners of the panel outline in the world, in order round the boundary."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * lx - s * ly, y + s * lx + c * ly)
            for lx, ly in ((-length / 2, -width / 2), (length / 2, -width / 2),
                           (length / 2, width / 2), (-length / 2, width / 2))]


def distance_to_polygon_boundary(px, py, corners):
    """Distance from a point to the nearest side of the closed polygon."""
    best = float('inf')
    for i, (ax, ay) in enumerate(corners):
        bx, by = corners[(i + 1) % len(corners)]
        dx, dy = bx - ax, by - ay
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def ground_point(position, rotation, plane_z):
    """Where the optical axis (the camera's z axis) meets the plane z = plane_z, or None.

    rotation is the camera orientation as a 3x3 matrix (columns = camera axes in world).
    """
    axis = (rotation[0][2], rotation[1][2], rotation[2][2])
    if axis[2] > -0.2:
        return None
    s = (plane_z - position[2]) / axis[2]
    return position[0] + s * axis[0], position[1] + s * axis[1]


def gz_pose_request(model, x, y, z, yaw):
    return ('name: "%s", position: {x: %.6f, y: %.6f, z: %.6f}, '
            'orientation: {x: 0, y: 0, z: %.6f, w: %.6f}' %
            (model, x, y, z, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))


# ---- the ROS node --------------------------------------------------------------------


def main():
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker

    class Shifter(Node):
        def __init__(self):
            super().__init__('part_shifter')
            d = self.declare_parameter
            d('shifts', True)  # false: only score (a clean lap)
            d('seed', -1)
            d('nominal_x', 0.8)
            d('nominal_y', 0.0)
            d('nominal_yaw_deg', 0.0)
            d('max_xy', 0.02)
            d('max_yaw_deg', 2.0)
            d('min_step', 0.005)
            d('ramp_sec', 1.0)
            d('first_delay_sec', 3.0)
            d('period_min_sec', 4.0)
            d('period_max_sec', 8.0)
            d('update_hz', 10.0)
            d('panel_z', 0.79)
            d('panel_top_z', 0.805)
            d('panel_length', PANEL_LENGTH)
            d('panel_width', PANEL_WIDTH)
            d('tol_position', 0.008)
            d('settle_sec', 2.0)
            d('gz_world', 'ur30_cell')
            d('panel_model', 'door_panel_test')
            d('camera_frame', 'camera_color_optical_frame')
            d('world_frame', 'world')
            d('results_file', '')

            self.state = None
            self.create_subscription(
                String, '/ibvs_state_machine/state', self._on_state,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.true_pub = self.create_publisher(Marker, '~/true_outline', latched)
            self.nominal_pub = self.create_publisher(Marker, '~/nominal_outline', latched)
            self.trail_pub = self.create_publisher(Path, '~/tool_trail', 1)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.trail = Path()

        def _on_state(self, msg):
            if msg.data != self.state:
                self.get_logger().info('IBVS state: %s' % msg.data)
            self.state = msg.data

        def p(self, name):
            return self.get_parameter(name).value

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def pump(self):
            rclpy.spin_once(self, timeout_sec=0.02)

        # -- panel
        def set_panel_pose(self, pose):
            request = gz_pose_request(self.p('panel_model'), pose[0], pose[1],
                                      self.p('panel_z'), pose[2])
            command = ['gz', 'service', '-s', '/world/%s/set_pose' % self.p('gz_world'),
                       '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                       '--timeout', '3000', '--req', request]
            try:
                out = subprocess.run(command, capture_output=True, text=True, timeout=10)
            except (OSError, subprocess.TimeoutExpired) as e:
                self.get_logger().error('gz set_pose failed: %s' % e)
                return False
            if out.returncode != 0 or 'true' not in out.stdout:
                self.get_logger().error('gz set_pose failed: %s %s' % (out.stdout, out.stderr))
                return False
            return True

        def outline_marker(self, pose, ns, rgb):
            m = Marker()
            m.header.frame_id = self.p('world_frame')
            m.ns = ns
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.006
            m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
            corners = panel_corners(pose[0], pose[1], pose[2], self.p('panel_length'),
                                    self.p('panel_width'))
            for cx, cy in corners + corners[:1]:
                m.points.append(Point(x=cx, y=cy, z=self.p('panel_top_z') + 0.002))
            return m

        # -- scoring
        def camera_ground_point(self):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.p('world_frame'), self.p('camera_frame'), Time())
            except TransformException:
                return None
            tr, q = t.transform.translation, t.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            return ground_point((tr.x, tr.y, tr.z), rotation, self.p('panel_top_z'))

        def add_trail_point(self, point):
            stamp = self.get_clock().now().to_msg()
            self.trail.header.frame_id = self.p('world_frame')
            self.trail.header.stamp = stamp
            pose = PoseStamped()
            pose.header = self.trail.header
            pose.pose.position.x, pose.pose.position.y = point
            pose.pose.position.z = self.p('panel_top_z') + 0.002
            pose.pose.orientation.w = 1.0
            self.trail.poses.append(pose)
            if len(self.trail.poses) > 4000:
                del self.trail.poses[0]
            self.trail_pub.publish(self.trail)

        # -- the run
        def run(self):
            log = self.get_logger()
            seed = self.p('seed')
            if seed < 0:
                seed = random.SystemRandom().randrange(1, 10 ** 6)
            log.info('seed %d (pass seed:=%d to repeat this run)' % (seed, seed))
            rng = random.Random(seed)

            nominal = (self.p('nominal_x'), self.p('nominal_y'),
                       math.radians(self.p('nominal_yaw_deg')))
            max_yaw = math.radians(self.p('max_yaw_deg'))
            pose = nominal  # where the panel is now (mid-ramp: the interpolated pose)
            ramp = None  # (start_pose, end_pose, start_time)
            self.nominal_pub.publish(self.outline_marker(nominal, 'nominal', (1.0, 0.2, 0.2)))
            self.true_pub.publish(self.outline_marker(pose, 'true', (0.1, 0.9, 0.2)))

            samples, shifts = [], 0
            lap_started = None
            next_shift = None
            last_motion = None  # when the panel last stopped moving
            next_sample = next_marker = next_update = 0.0
            update_period = 1.0 / self.p('update_hz')

            while rclpy.ok():
                self.pump()
                t = self.now()
                if self.state in ('DONE', 'FAILED') and lap_started is not None:
                    break
                if self.state != 'DISPENSE':
                    continue
                if lap_started is None:
                    lap_started = t
                    last_motion = t
                    next_shift = t + self.p('first_delay_sec')
                    log.info('the lap has started: nudging the panel every %.0f-%.0f s' %
                             (self.p('period_min_sec'), self.p('period_max_sec')))

                # start a nudge
                if self.p('shifts') and ramp is None and t >= next_shift:
                    target = random_pose(rng, nominal, self.p('max_xy'), max_yaw, pose,
                                         self.p('min_step'))
                    if target is not None:
                        ramp = (pose, target, t)
                        shifts += 1
                        log.info('nudge %d: dx %+.1f mm, dy %+.1f mm, dyaw %+.1f deg' % (
                            shifts, (target[0] - pose[0]) * 1000, (target[1] - pose[1]) * 1000,
                            math.degrees(target[2] - pose[2])))
                # advance it
                if ramp is not None and t >= next_update:
                    pose = interpolate_pose(ramp[0], ramp[1], t - ramp[2], self.p('ramp_sec'))
                    self.set_panel_pose(pose)
                    next_update = t + update_period
                    if t - ramp[2] >= self.p('ramp_sec'):
                        pose = ramp[1]
                        self.set_panel_pose(pose)
                        ramp = None
                        last_motion = t
                        next_shift = t + rng.uniform(self.p('period_min_sec'),
                                                     self.p('period_max_sec'))
                if ramp is not None:
                    last_motion = t

                if t >= next_sample:
                    next_sample = t + 0.1
                    point = self.camera_ground_point()
                    if point is not None:
                        corners = panel_corners(pose[0], pose[1], pose[2],
                                                self.p('panel_length'), self.p('panel_width'))
                        error = distance_to_polygon_boundary(point[0], point[1], corners)
                        samples.append((t, error, t - last_motion >= self.p('settle_sec')))
                        self.add_trail_point(point)
                if t >= next_marker:
                    next_marker = t + 0.5
                    self.true_pub.publish(self.outline_marker(pose, 'true', (0.1, 0.9, 0.2)))

            finished = self.state == 'DONE'
            errors = [e for _, e, _ in samples]
            settled = [e for _, e, ok in samples if ok]
            result = {'seed': seed, 'finished_lap': finished, 'nudges': shifts,
                      'lap_seconds': round(self.now() - lap_started, 1) if lap_started else None,
                      'samples': len(errors),
                      'max_error_mm': round(max(errors) * 1000, 1) if errors else None,
                      'rms_error_mm': round(math.sqrt(sum(e * e for e in errors) / len(errors))
                                            * 1000, 1) if errors else None,
                      'max_settled_error_mm': round(max(settled) * 1000, 1) if settled else None,
                      'tol_position_mm': self.p('tol_position') * 1000}
            passed = (finished and bool(settled) and
                      max(settled) <= self.p('tol_position'))
            result['passed'] = passed
            log.info('==== lap %s: %s' % ('PASS' if passed else 'FAIL', json.dumps(result)))
            if self.p('results_file'):
                with open(self.p('results_file'), 'w') as f:
                    json.dump(result, f, indent=2)
            return passed

    rclpy.init()
    node = Shifter()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
