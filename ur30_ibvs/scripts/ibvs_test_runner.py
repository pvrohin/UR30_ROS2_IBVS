#!/usr/bin/env python3
"""Runs several IBVS test cases back to back in the simulator.

For each case the door panel is put at a random pose on the table (inside the survey
camera's view), the IBVS node is started, and the camera pose is compared with the pose
it should reach: 0.2 m above the middle of the panel's +y long edge, looking straight
down, image x along the edge. A case passes when the camera gets within tolerance and
stays there for hold_sec while the IBVS node remains in SERVO.

The expected pose comes from the panel pose this runner set and from TF (the simulated
camera pose), not from anything the IBVS node reports about itself.

The panel is a separate Gazebo model (door_panel_model.xacro) moved with the gz
set_pose service, so the cell must be launched with include_panel:=false and the IBVS
node with auto_start:=false; ur30_ibvs.launch.py does both when test_cases > 0.
"""
import json
import math
import random
import subprocess
import sys

PANEL_LENGTH = 0.9
PANEL_WIDTH = 0.55


# ---- pure helpers (no ROS), unit tested in test/test_ibvs_test_runner.py -------------


def footprint_fits(cx, cy, yaw, cam_x, cam_y, half_w, half_h, margin,
                   length=PANEL_LENGTH, width=PANEL_WIDTH):
    """True if the yawed panel lies inside the camera view, with `margin` to spare.

    The camera looks straight down with image x along world x, so the visible
    ground rectangle is |x - cam_x| <= half_w, |y - cam_y| <= half_h.
    """
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    ext_x = (length * c + width * s) / 2.0
    ext_y = (length * s + width * c) / 2.0
    return (abs(cx - cam_x) + ext_x + margin <= half_w and
            abs(cy - cam_y) + ext_y + margin <= half_h)


def clear_of_robot(cx, yaw, min_x, length=PANEL_LENGTH, width=PANEL_WIDTH):
    """True if the panel's near edge stays at x >= min_x.

    The table starts at x = 0.3 and the arm shows at the left of the survey image up to
    about x = 0.29; a panel that reaches it merges with the arm in the detector's
    threshold blob (found in the first run: a panel at x = 0.737 overhung the table and
    the search never succeeded).
    """
    return cx - (length * abs(math.cos(yaw)) + width * abs(math.sin(yaw))) / 2.0 >= min_x


def sample_pose(rng, p, previous=None, tries=2000):
    """A random (x, y, yaw) inside the view; differs visibly from `previous`.

    p needs: center_x, center_y, x_range, y_range, yaw_range (rad), cam_x, cam_y,
    half_w, half_h, margin, min_x, min_separation, min_yaw_separation (rad).
    Returns None if no pose satisfies the constraints.
    """
    for _ in range(tries):
        cx = p['center_x'] + rng.uniform(-p['x_range'], p['x_range'])
        cy = p['center_y'] + rng.uniform(-p['y_range'], p['y_range'])
        yaw = rng.uniform(-p['yaw_range'], p['yaw_range'])
        if not footprint_fits(cx, cy, yaw, p['cam_x'], p['cam_y'], p['half_w'],
                              p['half_h'], p['margin']):
            continue
        if not clear_of_robot(cx, yaw, p['min_x']):
            continue
        if previous is not None:
            moved = math.hypot(cx - previous[0], cy - previous[1]) >= p['min_separation']
            turned = abs(yaw - previous[2]) >= p['min_yaw_separation']
            if not (moved or turned):
                continue
        return cx, cy, yaw
    return None


def expected_camera(cx, cy, yaw, top_z, standoff, edge_sign=1.0, width=PANEL_WIDTH):
    """Camera position and image-x direction that following the panel's long edge gives."""
    e = edge_sign * width / 2.0
    position = (cx - math.sin(yaw) * e, cy + math.cos(yaw) * e, top_z + standoff)
    x_axis = (math.cos(yaw), math.sin(yaw))
    return position, x_axis


def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def pose_errors(cam_position, cam_rotation, exp_position, exp_x_axis):
    """(position error, lateral, height, yaw, tilt) of the camera against the expectation.

    Lengths in metres, angles in degrees, all absolute values. Yaw is the angle between the
    camera x axis (projected on the ground) and the expected direction; tilt is the angle
    between the optical axis and straight down.
    """
    dx = cam_position[0] - exp_position[0]
    dy = cam_position[1] - exp_position[1]
    dz = cam_position[2] - exp_position[2]
    lateral = math.hypot(dx, dy)
    ax, ay = cam_rotation[0][0], cam_rotation[1][0]
    yaw = math.atan2(ax * exp_x_axis[1] - ay * exp_x_axis[0],
                     ax * exp_x_axis[0] + ay * exp_x_axis[1])
    down = max(-1.0, min(1.0, -cam_rotation[2][2]))
    return (math.sqrt(dx * dx + dy * dy + dz * dz), lateral, abs(dz),
            abs(math.degrees(yaw)), math.degrees(math.acos(down)))


def gz_pose_request(model, x, y, z, yaw):
    return ('name: "%s", position: {x: %.6f, y: %.6f, z: %.6f}, '
            'orientation: {x: 0, y: 0, z: %.6f, w: %.6f}' %
            (model, x, y, z, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))


# ---- the ROS node --------------------------------------------------------------------


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker

    class Runner(Node):
        def __init__(self):
            super().__init__('ibvs_test_runner')
            d = self.declare_parameter
            d('num_cases', 3)
            d('seed', -1)  # < 0: pick one and log it
            d('center_x', 0.8)
            d('center_y', 0.0)
            d('x_range', 0.10)
            d('y_range', 0.02)
            d('yaw_range_deg', 4.0)
            # Survey camera: straight down at (0.8, 0), 0.6 m above the panel top face,
            # 640 x 360 px at fx = 640 -> half view 0.6 x 0.3375 m.
            d('cam_x', 0.8)
            d('cam_y', 0.0)
            d('view_half_width', 0.6)
            d('view_half_height', 0.3375)
            d('view_margin', 0.015)
            d('min_panel_x', 0.32)  # nearest x the panel may reach (table edge, arm in view)
            d('min_separation', 0.05)
            d('min_yaw_separation_deg', 2.0)
            d('panel_z', 0.79)  # model origin: the panel centre (top face is 0.015 higher)
            d('panel_top_z', 0.805)
            d('standoff', 0.20)
            d('edge_sign', 1.0)
            d('tol_position', 0.006)
            d('tol_yaw_deg', 1.5)
            d('tol_tilt_deg', 1.5)
            d('hold_sec', 3.0)
            d('converge_timeout_sec', 60.0)
            d('reach_servo_timeout_sec', 120.0)
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
            self.start_client = self.create_client(Trigger, '/ibvs_state_machine/start')
            self.marker_pub = self.create_publisher(
                Marker, '~/panel_marker',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.panel = None  # (x, y, yaw) currently set
            self._marker_time = -1e9

        # -- plumbing
        def _on_state(self, msg):
            if msg.data != self.state:
                self.get_logger().info('IBVS state: %s' % msg.data)
            self.state = msg.data

        def p(self, name):
            return self.get_parameter(name).value

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def pump(self):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.panel is not None and self.now() - self._marker_time > 1.0:
                self.publish_marker()

        def wait_until(self, predicate, timeout):
            start = self.now()
            while rclpy.ok():
                self.pump()
                if predicate():
                    return True
                if self.now() - start > timeout:
                    return False
            return False

        def sleep(self, seconds):
            self.wait_until(lambda: False, seconds)

        # -- panel
        def publish_marker(self):
            x, y, yaw = self.panel
            m = Marker()
            m.header.frame_id = self.p('world_frame')
            m.ns = 'test_panel'
            m.id = 0
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, self.p('panel_z')
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale.x, m.scale.y, m.scale.z = PANEL_LENGTH, PANEL_WIDTH, 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.15, 0.17, 1.0
            self.marker_pub.publish(m)
            self._marker_time = self.now()

        def set_panel_pose(self, x, y, yaw):
            request = gz_pose_request(self.p('panel_model'), x, y, self.p('panel_z'), yaw)
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
            self.panel = (x, y, yaw)
            self.publish_marker()
            return True

        # -- IBVS node
        def request_start(self, timeout=180.0):
            """Call ~/start until the IBVS node accepts it (it refuses while not ready)."""
            start = self.now()
            while rclpy.ok() and self.now() - start < timeout:
                if not self.start_client.wait_for_service(timeout_sec=0.5):
                    self.pump()
                    continue
                future = self.start_client.call_async(Trigger.Request())
                while rclpy.ok() and not future.done():
                    self.pump()
                if future.result() is not None and future.result().success:
                    return True
                self.sleep(1.0)
            return False

        def camera_errors(self):
            """Errors of the camera pose against the expectation for the current panel."""
            try:
                t = self.tf_buffer.lookup_transform(
                    self.p('world_frame'), self.p("camera_frame"), Time())
            except TransformException:
                return None
            tr, q = t.transform.translation, t.transform.rotation
            position, x_axis = expected_camera(
                self.panel[0], self.panel[1], self.panel[2], self.p('panel_top_z'),
                self.p('standoff'), self.p('edge_sign'))
            return pose_errors((tr.x, tr.y, tr.z), quat_to_matrix((q.x, q.y, q.z, q.w)),
                               position, x_axis)

        def within_tolerance(self, e):
            return (e is not None and e[0] <= self.p('tol_position') and
                    e[3] <= self.p('tol_yaw_deg') and e[4] <= self.p('tol_tilt_deg'))

        # -- one case
        def run_case(self, index, pose):
            log = self.get_logger()
            result = {'case': index + 1, 'panel_x': pose[0], 'panel_y': pose[1],
                      'panel_yaw_deg': math.degrees(pose[2]), 'passed': False, 'reason': ''}
            log.info('---- case %d: panel at x=%.3f y=%.3f yaw=%.1f deg' %
                     (index + 1, pose[0], pose[1], math.degrees(pose[2])))
            if index == 0:
                if not self.set_panel_pose(*pose):
                    result['reason'] = 'could not move the panel'
                    return result
                if not self.request_start():
                    result['reason'] = 'the IBVS node never accepted ~/start'
                    return result
            else:
                # Send the arm home first and put the panel in place once it is away.
                if not self.request_start():
                    result['reason'] = 'the IBVS node never accepted ~/start'
                    return result
                self.wait_until(lambda: self.state in ('RESETTING', 'GO_TO_SURVEY'), 30.0)
                self.sleep(2.0)
                if not self.set_panel_pose(*pose):
                    result['reason'] = 'could not move the panel'
                    return result

            t0 = self.now()
            if not self.wait_until(lambda: self.state in ('SERVO', 'FAILED'),
                                   self.p('reach_servo_timeout_sec')) or self.state != 'SERVO':
                result['reason'] = 'IBVS node ended in state %s before servoing' % self.state
                return result
            result['seconds_to_servo'] = round(self.now() - t0, 1)

            errors = [None]

            def converged():
                errors[0] = self.camera_errors()
                return self.state == 'FAILED' or self.within_tolerance(errors[0])

            t1 = self.now()
            if not self.wait_until(converged, self.p('converge_timeout_sec')) \
                    or self.state != 'SERVO':
                result['reason'] = ('state %s, last errors %s' % (self.state, errors[0])
                                    if self.state != 'SERVO' else
                                    'did not reach the tolerance in %.0f s (last errors: %s)' %
                                    (self.p('converge_timeout_sec'), errors[0]))
                return result
            result['seconds_to_converge'] = round(self.now() - t1, 1)

            worst = [0.0] * 5  # over the hold only, not the moment the tolerance was met
            last = errors[0]
            hold_start = self.now()
            while self.now() - hold_start < self.p('hold_sec'):
                self.pump()
                e = self.camera_errors()
                if self.state != 'SERVO':
                    result['reason'] = 'left SERVO (state %s) during the hold' % self.state
                    return result
                if e is None:
                    continue
                worst = [max(a, b) for a, b in zip(worst, e)]
                last = e
                if not self.within_tolerance(e):
                    result['reason'] = 'left the tolerance during the hold: %s' % (e,)
                    return result
            result['passed'] = True
            result['worst_position_mm'] = round(worst[0] * 1000, 1)
            result['worst_yaw_deg'] = round(worst[3], 2)
            result['worst_tilt_deg'] = round(worst[4], 2)
            # where the camera ended up, at the end of the hold (lateral, height in mm)
            result['final_lateral_mm'] = round(last[1] * 1000, 1)
            result['final_height_mm'] = round(last[2] * 1000, 1)
            return result

        def run(self):
            log = self.get_logger()
            seed = self.p('seed')
            if seed < 0:
                seed = random.SystemRandom().randrange(1, 10 ** 6)
            log.info('seed %d (pass seed:=%d to repeat this run)' % (seed, seed))
            rng = random.Random(seed)
            limits = {
                'center_x': self.p('center_x'), 'center_y': self.p('center_y'),
                'x_range': self.p('x_range'), 'y_range': self.p('y_range'),
                'yaw_range': math.radians(self.p('yaw_range_deg')),
                'cam_x': self.p('cam_x'), 'cam_y': self.p('cam_y'),
                'half_w': self.p('view_half_width'), 'half_h': self.p('view_half_height'),
                'margin': self.p('view_margin'), 'min_x': self.p('min_panel_x'),
                'min_separation': self.p('min_separation'),
                'min_yaw_separation': math.radians(self.p('min_yaw_separation_deg')),
            }
            results, previous = [], None
            for i in range(self.p('num_cases')):
                pose = sample_pose(rng, limits, previous)
                if pose is None:
                    log.error('no panel pose fits the view with these limits')
                    break
                previous = pose
                result = self.run_case(i, pose)
                log.info('case %d: %s%s' % (
                    i + 1, 'PASS' if result['passed'] else 'FAIL',
                    '' if result['passed'] else ' - ' + result['reason']))
                results.append(result)
                if not result['passed'] and self.state not in ('SERVO', 'IDLE', 'FAILED'):
                    log.error('the IBVS node is not in a state to continue; stopping')
                    break

            passed = sum(1 for r in results if r['passed'])
            log.info('==== %d of %d cases passed (seed %d)' % (passed, self.p('num_cases'), seed))
            for r in results:
                log.info('  case %d: %s  panel (%.3f, %.3f, %.1f deg)  %s' % (
                    r['case'], 'PASS' if r['passed'] else 'FAIL', r['panel_x'], r['panel_y'],
                    r['panel_yaw_deg'],
                    ('final %.1f mm lateral, %.1f mm height; worst %.1f mm, %.2f deg yaw, '
                     '%.2f deg tilt; servo after %.1f s, converged after %.1f s' % (
                         r['final_lateral_mm'], r['final_height_mm'], r['worst_position_mm'],
                         r['worst_yaw_deg'], r['worst_tilt_deg'], r['seconds_to_servo'],
                         r['seconds_to_converge']))
                    if r['passed'] else r['reason']))
            if self.p('results_file'):
                with open(self.p('results_file'), 'w') as f:
                    json.dump({'seed': seed, 'results': results}, f, indent=2)
            return passed == self.p('num_cases') and len(results) == self.p('num_cases')

    rclpy.init()
    node = Runner()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
