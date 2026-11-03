#!/usr/bin/env python3
"""Automatically transfer one 40 mm dark cube to the opposite tray.

Default is detection-only. Add --execute to move the real robot.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import random
import threading
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from detect_black_cube_rgbd import depth_metres, image_bgr, tf_matrix
from select_black_cube_rgbd import dark_candidates, estimate_square_from_depth
from click_pick_place_sequence import (
    ARM_CARTESIAN_SCALING, ARM_JOINT_SCALING, CUBE_SIZE, GRASP_FORCE, GRASP_TCP_Z,
    GRASP_WIDTH, LIFT_HEIGHT, PLACE_TCP_Z, SURFACE_Z,
    current_downward, execute_cartesian, pose, square_grasp_quaternion,
    validate_workspace, verify_gripper_open, xy,
)
from fr3_skills.motion import JOINTS, MAX_SCALING, _quat_rotate, load_pose
from fr3_skills.motion import Fr3Motion
from episode_recorder import EpisodeRecorder, instrument_motion


TRAY_CENTER_X = 0.415
TRAY_CENTER_ABS_Y = 0.130
PLACE_RADIUS_MIN = 0.035
PLACE_RADIUS_MAX = 0.045
PLACE_YAW_OFFSET_LIMIT = math.radians(30.0)
AUTO_DARK_THRESHOLD = 75
TRAY_CENTERS_PATH = Path('calibration_data/tray_centers.json')
HOME_CAMERA_SETTLE_SECONDS = 1.0
# Extra forward wrist pitch at HOME so the wrist camera sees cubes placed
# far out on the trays. +joint6 tilts the flange toward +X (away from base).
HOME_VIEW_PITCH_RAD = math.radians(8.0)
# Detected top centre vs. commanded place point; includes the grasp offset.
PLACE_SUCCESS_TOLERANCE_M = 0.025
# Extra fixed grasp shift along base X. Episodes 28-31 used +0.004 (and no
# far-edge correction); from episode 32 the corrections below replace it.
GRASP_OFFSET_X_M = 0.0
# The depth patch of the cube top usually comes out 30-35 mm instead of 40 mm,
# cut short on the side away from the camera, so its centre sits toward the
# camera (robot side). Grow each short side back to 40 mm away from the camera.
TOP_FAR_EDGE_MAX_SHIFT_M = 0.006
# Grasps still landed on the camera/robot side, so push a bit further away
# from the camera on top of the far-edge correction.
TOP_EXTRA_AWAY_SHIFT_M = 0.003
# Second look from the hover pose straight above the cube (camera ~34 cm above
# the top, nearly top-down), used for the final grasp x/y/yaw. From episode 32.
HOVER_REDETECT = True
HOVER_REDETECT_TIMEOUT = 8.0
HOVER_REDETECT_MAX_JUMP_M = 0.030   # must be the same cube as seen from HOME

# Demonstration noise (imitation learning). Only the path *to* the contact
# points varies; the grasp and place points themselves stay exact.
NOISE_WAYPOINT_XY_M = 0.010      # hover / lift / retreat waypoints, +-
NOISE_WAYPOINT_Z_M = 0.015
NOISE_HOME_JOINT_RAD = math.radians(2.5)   # every joint of the HOME target, +-
NOISE_CARTESIAN_SPEED = (0.8, 1.2)         # x ARM_CARTESIAN_SCALING
# ARM_JOINT_SCALING already equals MoveIt's MAX_SCALING, so only slower.
NOISE_JOINT_SPEED = (0.7, 1.0)             # x ARM_JOINT_SCALING
# ~1 MB images lose UDP fragments under best effort; reliable lets the
# missing fragments be resent. depth=1 because only the latest frame is used.
CAMERA_IMAGE_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST)


class CameraReceiver:
    """Continuously receive camera data on a node independent of arm motion."""

    def __init__(self):
        self.node = rclpy.create_node('auto_cube_camera_receiver')
        self.lock = threading.Lock()
        self.colors = deque(maxlen=60)
        self.depths = deque(maxlen=60)
        self.infos = deque(maxlen=5)
        self.subscriptions = [
            self.node.create_subscription(
                Image, '/camera/camera/color/image_raw',
                lambda message: self._append(self.colors, message),
                CAMERA_IMAGE_QOS),
            self.node.create_subscription(
                Image, '/camera/camera/aligned_depth_to_color/image_raw',
                lambda message: self._append(self.depths, message),
                CAMERA_IMAGE_QOS),
            self.node.create_subscription(
                CameraInfo, '/camera/camera/color/camera_info',
                lambda message: self._append(self.infos, message),
                qos_profile_sensor_data),
        ]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _append(self, queue, message):
        with self.lock:
            queue.append((time.monotonic(), message))

    def snapshot(self):
        with self.lock:
            return list(self.colors), list(self.depths), list(self.infos)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


def find_valid_cube_squares(frame, depth, camera_matrix, base_to_camera):
    """Find horizontal 40 mm tops, trying several depth seeds per dark blob.

    The RGB contour often contains both the cube top and its visible side.  Its
    centroid can therefore land on the vertical face, so a single centroid is
    not a reliable depth seed.
    """
    candidates = dark_candidates(frame, AUTO_DARK_THRESHOLD, 100, 80000)
    valid = []
    failures = []

    def try_seed(u, v, contour):
        try:
            square = estimate_square_from_depth(
                depth, u, v, camera_matrix, base_to_camera,
                expected_size_m=CUBE_SIZE)
        except ValueError as exc:
            failures.append(str(exc))
            return
        centre_x, centre_y = square['centre_xy_m']
        centre_z_raw = square['centre_z_base_raw_m']
        if not (0.36 <= centre_x <= 0.47
                and 0.08 <= abs(centre_y) <= 0.19):
            return
        if not (0.11 <= centre_z_raw <= 0.19):
            return
        if any(np.hypot(centre_x-old[3]['centre_xy_m'][0],
                        centre_y-old[3]['centre_xy_m'][1]) < 0.015
               for old in valid):
            return
        sides = square['side_lengths_m']
        score = abs(sides[0]-CUBE_SIZE) + abs(sides[1]-CUBE_SIZE)
        valid.append((score, u, v, square, contour))

    for item in candidates:
        contour = item['contour']
        x0, y0, width, height = item['box']
        moments = cv2.moments(contour)
        seeds = []
        if moments['m00']:
            seeds.append((moments['m10']/moments['m00'],
                          moments['m01']/moments['m00']))
        # Cover the interior, including the narrow top face near the upper edge.
        for fy in (0.12, 0.25, 0.40, 0.58, 0.76):
            for fx in (0.18, 0.34, 0.50, 0.66, 0.82):
                u = x0 + fx*width
                v = y0 + fy*height
                if cv2.pointPolygonTest(contour, (u, v), False) >= 0:
                    seeds.append((u, v))
        for u, v in seeds:
            try_seed(u, v, contour)

    # At about 45 degrees the black top, side and shadow can merge into one
    # irregular RGB blob.  If contour seeds failed, scan dark depth pixels in
    # the known trays without requiring a closed or square-looking RGB contour.
    if not valid:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = depth.shape
        vv, uu = np.mgrid[5:height:10, 5:width:10]
        zz = depth[vv, uu]
        mask = (np.isfinite(zz) & (zz > 0.10) & (zz < 2.0)
                & (gray[vv, uu] <= 110))
        u_flat = uu[mask].astype(np.float64)
        v_flat = vv[mask].astype(np.float64)
        z_flat = zz[mask].astype(np.float64)
        if z_flat.size:
            fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
            ppx, ppy = camera_matrix[0, 2], camera_matrix[1, 2]
            camera_points = np.column_stack((
                (u_flat-ppx)*z_flat/fx, (v_flat-ppy)*z_flat/fy,
                z_flat, np.ones_like(z_flat)))
            base_points = (base_to_camera @ camera_points.T).T
            tray = ((base_points[:, 0] >= 0.36)
                    & (base_points[:, 0] <= 0.47)
                    & (np.abs(base_points[:, 1]) >= 0.08)
                    & (np.abs(base_points[:, 1]) <= 0.19)
                    & (base_points[:, 2] >= 0.11)
                    & (base_points[:, 2] <= 0.19))
            indexes = np.flatnonzero(tray)
            # Higher points are more likely to be the cube top than tray floor.
            indexes = indexes[np.argsort(base_points[indexes, 2])[::-1]][:120]
            for index in indexes:
                try_seed(float(u_flat[index]), float(v_flat[index]), None)
                if valid:
                    break
    valid.sort(key=lambda value: value[0])
    return candidates, valid, failures


def far_edge_correction(square, camera_xy, centre_x, centre_y):
    """Base-XY shift that grows each short side of the fitted top to CUBE_SIZE,
    on the side away from the camera.

    edge_yaw points along the long side; side_lengths_m is (short, long).
    """
    yaw = square['edge_yaw_base_rad']
    short_side, long_side = square['side_lengths_m']
    away = np.array([centre_x-camera_xy[0], centre_y-camera_xy[1]], dtype=float)
    shift = np.zeros(2)
    for axis, length in (((math.cos(yaw), math.sin(yaw)), long_side),
                         ((-math.sin(yaw), math.cos(yaw)), short_side)):
        axis = np.array(axis)
        deficit = max(0.0, CUBE_SIZE-length)
        amount = min(deficit/2.0, TOP_FAR_EDGE_MAX_SHIFT_M)
        shift += amount * (1.0 if axis @ away >= 0.0 else -1.0) * axis
    distance = np.linalg.norm(away)
    if distance > 1e-6:
        shift += TOP_EXTRA_AWAY_SHIFT_M * away / distance
    return float(shift[0]), float(shift[1])


def auto_detect_cube(motion, camera, calibration_path, timeout=30.0,
                     near_xy=None, label='HOME', debug_name='auto_cube_detection'):
    """Detect the cube top from fresh RGB-D.

    near_xy: re-detection mode. Only cubes within HOVER_REDETECT_MAX_JUMP_M of
    this base-XY point are accepted, the nearest one wins, and the HOME-view
    far-edge corrections are skipped (the hover view is nearly top-down).
    """
    calibration = json.loads(calibration_path.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    # A one-shot run naturally has a pause before detection. In a continuous
    # run the wrist has just stopped, so let vibration, auto exposure and
    # aligned depth settle before accepting a frame.
    print(f'waiting {HOME_CAMERA_SETTLE_SECONDS:.1f} s for {label} camera/TF settling...')
    time.sleep(HOME_CAMERA_SETTLE_SECONDS)
    print(f'{label} settled; detecting cube from fresh RGB-D...')

    # Only accept frames that arrive after HOME settling. The receiver
    # itself stays alive and keeps spinning while arm actions block this thread.
    request_time = time.monotonic()
    deadline = time.monotonic() + timeout
    last_error = 'waiting for fresh RGB-D from background camera receiver'
    warned_stamp_gap = False
    while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(motion, timeout_sec=0.05)
            all_colors, all_depths, infos = camera.snapshot()
            colors = [item for item in all_colors if item[0] >= request_time]
            depths = [item for item in all_depths if item[0] >= request_time]
            if not colors or not depths or not infos:
                last_error = (f'background camera receiver counts: RGB={len(colors)}, '
                              f'depth={len(depths)}, camera_info={len(infos)}')
                continue
            color_msg = colors[-1][1]
            color_ns = Time.from_msg(color_msg.header.stamp).nanoseconds
            depth_msg = min(
                depths,
                key=lambda item: abs(
                    color_ns-Time.from_msg(item[1].header.stamp).nanoseconds))[1]
            info = infos[-1][1]
            color_stamp = Time.from_msg(color_msg.header.stamp)
            depth_stamp = Time.from_msg(depth_msg.header.stamp)
            stamp_gap_ms = abs(
                color_stamp.nanoseconds-depth_stamp.nanoseconds)/1_000_000.0
            if stamp_gap_ms > 100.0 and not warned_stamp_gap:
                print(f'WARNING: fresh RGB/depth stamp gap is {stamp_gap_ms:.1f} ms; '
                      'continuing because detection runs only while the robot is stopped')
                warned_stamp_gap = True
            try:
                wrist = motion._tf_buffer.lookup_transform(
                    'fr3_link0', 'fr3_link8', color_stamp)
            except TransformException as exc:
                last_error = 'wrist TF unavailable: ' + str(exc)
                continue
            frame = image_bgr(color_msg)
            depth = depth_metres(depth_msg)
            k = np.asarray(info.k, dtype=float).reshape(3, 3)
            base_to_camera = tf_matrix(wrist) @ link8_to_camera
            candidates, valid, failures = find_valid_cube_squares(
                frame, depth, k, base_to_camera)
            if not valid:
                last_error = ('no valid horizontal 40 mm square; ' + failures[0]
                              if failures else 'no dark candidates')
                continue
            if near_xy is not None:
                def jump(item):
                    cx, cy = item[3]['centre_xy_m']
                    return math.hypot(cx-near_xy[0], cy-near_xy[1])
                valid = [item for item in valid if jump(item) <= HOVER_REDETECT_MAX_JUMP_M]
                if not valid:
                    last_error = (f'no cube within {HOVER_REDETECT_MAX_JUMP_M*1000:.0f} mm '
                                  'of the HOME detection')
                    continue
                valid.sort(key=jump)
            else:
                valid.sort(key=lambda value: value[0])
                if len(valid) > 1 and valid[1][0] < valid[0][0] + 0.006:
                    raise RuntimeError(
                        f'{len(valid)} similarly plausible cubes detected; '
                        'refusing automatic choice')
            _, u, v, square, contour = valid[0]
            raw_x, raw_y = square['centre_xy_m']
            if near_xy is None:
                shift_x, shift_y = far_edge_correction(
                    square, base_to_camera[:2, 3], raw_x, raw_y)
            else:
                shift_x = shift_y = 0.0
            x, y = raw_x+shift_x, raw_y+shift_y
            selection = {
                'visible_top_center_fr3_link0_m': [x, y, SURFACE_Z+CUBE_SIZE],
                'raw_depth_square_center_xy_m': [raw_x, raw_y],
                'far_edge_shift_xy_m': [shift_x, shift_y],
                'depth_square_valid': True,
                'depth_square_error': None,
                'depth_square': {
                    key: value for key, value in square.items() if key != 'box_pixels'
                },
                'detected_centroid_uv': [u, v],
                'image_stamp_ns': color_stamp.nanoseconds,
            }
            debug = frame.copy()
            if contour is not None:
                cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
            cv2.polylines(debug, [np.rint(square['box_pixels']).astype(np.int32)],
                          True, (255, 0, 255), 2)
            cv2.circle(debug, (round(u), round(v)), 6, (0, 255, 255), -1)
            Path('calibration_data').mkdir(exist_ok=True)
            cv2.imwrite(f'calibration_data/{debug_name}.jpg', debug)
            Path(f'calibration_data/{debug_name}.json').write_text(
                json.dumps(selection, indent=2)+'\n')
            return selection
    raise RuntimeError('automatic cube detection timed out: ' + last_error)


def sample_opposite_tray_target(source_y, rng):
    target_sign = 1.0 if source_y < 0 else -1.0
    centre_x = TRAY_CENTER_X
    centre_y = target_sign * TRAY_CENTER_ABS_Y
    if TRAY_CENTERS_PATH.exists():
        centres = json.loads(TRAY_CENTERS_PATH.read_text())
        key = 'positive_y_xy_m' if target_sign > 0 else 'negative_y_xy_m'
        centre_x, centre_y = map(float, centres[key])
    for _ in range(100):
        angle = rng.uniform(0.0, 2.0*math.pi)
        # Uniformly sample area in the 35--45 mm annulus around the measured
        # tray centre, instead of repeatedly using one exact radius.
        radius = math.sqrt(rng.uniform(
            PLACE_RADIUS_MIN**2, PLACE_RADIUS_MAX**2))
        x = centre_x + radius*math.cos(angle)
        y = centre_y + radius*math.sin(angle)
        if 0.36 <= x <= 0.47 and 0.08 <= abs(y) <= 0.19:
            yaw = rng.uniform(-PLACE_YAW_OFFSET_LIMIT, PLACE_YAW_OFFSET_LIMIT)
            return x, y, yaw, angle
    raise RuntimeError('failed to sample a safe point on the opposite tray')


class MotionNoise:
    """Sample per-segment noise and log every value as a recorder event."""

    def __init__(self, rng, enabled, recorder=None):
        self.rng = rng
        self.enabled = enabled
        self.recorder = recorder
        self.log = []

    def _record(self, entry):
        self.log.append(entry)
        if self.recorder:
            self.recorder.event('noise', **entry)

    def cartesian(self, label, exact=False):
        """(dx, dy, dz, scaling) for one straight-line move."""
        if not self.enabled:
            return 0.0, 0.0, 0.0, ARM_CARTESIAN_SCALING
        if exact:  # contact moves: exact point, nominal speed
            dx = dy = dz = 0.0
            scaling = ARM_CARTESIAN_SCALING
        else:
            dx = self.rng.uniform(-NOISE_WAYPOINT_XY_M, NOISE_WAYPOINT_XY_M)
            dy = self.rng.uniform(-NOISE_WAYPOINT_XY_M, NOISE_WAYPOINT_XY_M)
            dz = self.rng.uniform(-NOISE_WAYPOINT_Z_M, NOISE_WAYPOINT_Z_M)
            scaling = ARM_CARTESIAN_SCALING*self.rng.uniform(*NOISE_CARTESIAN_SPEED)
        self._record({'segment': label, 'offset_m': [dx, dy, dz], 'scaling': scaling})
        return dx, dy, dz, scaling

    def home(self, label):
        """(joint target dict, scaling) for the move back to HOME."""
        target = load_pose('home')
        target['fr3_joint6'] += HOME_VIEW_PITCH_RAD
        if not self.enabled:
            return target, ARM_JOINT_SCALING
        offsets = {joint: self.rng.uniform(-NOISE_HOME_JOINT_RAD, NOISE_HOME_JOINT_RAD)
                   for joint in JOINTS}
        target = {joint: target[joint]+offsets[joint] for joint in JOINTS}
        scaling = min(MAX_SCALING, ARM_JOINT_SCALING*self.rng.uniform(*NOISE_JOINT_SPEED))
        self._record({'segment': label,
                      'joint_offsets_rad': [offsets[joint] for joint in JOINTS],
                      'scaling': scaling})
        return target, scaling


def move_cartesian(motion, noise, x, y, z, quaternion, label, exact=False):
    dx, dy, dz, scaling = noise.cartesian(label, exact)
    if dx or dy or dz:
        print(f'[{label}] noise offset ({dx*1000:+.1f}, {dy*1000:+.1f}, '
              f'{dz*1000:+.1f}) mm, speed x{scaling/ARM_CARTESIAN_SCALING:.2f}')
    execute_cartesian(motion, pose(x+dx, y+dy, z+dz, quaternion), label, scaling=scaling)


def move_home(motion, noise, label):
    target, scaling = noise.home(label)
    start = motion.current_joints()
    trajectory = motion.plan_to_joints(target, scaling=scaling)
    print(f'[{label}] speed x{scaling/ARM_JOINT_SCALING:.2f}')
    print(motion.describe(trajectory, start))
    motion.activate_arm()
    try:
        motion.execute(trajectory)
    finally:
        motion.deactivate_arm()


def hover_redetect(motion, camera, calibration_path, home_xy, noise):
    """Look again from the hover pose. Returns the new selection or None."""
    try:
        refined = auto_detect_cube(
            motion, camera, calibration_path, timeout=HOVER_REDETECT_TIMEOUT,
            near_xy=home_xy, label='hover', debug_name='auto_cube_detection_hover')
    except RuntimeError as exc:
        print(f'hover re-detection failed ({exc}); using the HOME detection')
        if noise.recorder:
            noise.recorder.event('hover_redetect', ok=False, error=str(exc))
            noise.recorder.update(hover_detection=None)
        return None
    rx, ry = xy(refined)
    dx, dy = rx-home_xy[0], ry-home_xy[1]
    print(f'hover re-detection: ({rx:+.4f}, {ry:+.4f}), change from HOME '
          f'({dx*1000:+.1f}, {dy*1000:+.1f}) mm, size '
          f"{refined['depth_square']['side_lengths_m'][0]*1000:.1f} x "
          f"{refined['depth_square']['side_lengths_m'][1]*1000:.1f} mm")
    if noise.recorder:
        noise.recorder.event('hover_redetect', ok=True, xy_m=[rx, ry],
                             change_from_home_m=[dx, dy])
        noise.recorder.update(hover_detection=refined)
    return refined


def pick_with_noise(motion, selection, noise, camera=None, calibration_path=None,
                    home_gripper=True):
    """Same sequence as click_pick_place_sequence.pick_and_lift, plus noise
    and an optional second look from the hover pose before descending."""
    x, y = xy(selection)
    validate_workspace(x+GRASP_OFFSET_X_M, y)
    quaternion = square_grasp_quaternion(motion, selection)
    hover_z = GRASP_TCP_Z + LIFT_HEIGHT
    if home_gripper:
        motion.home_gripper()
    motion.open_gripper(width=0.08, speed=0.05)
    verify_gripper_open(motion)
    move_cartesian(motion, noise, x, y, hover_z, quaternion, 'move above pick')
    if HOVER_REDETECT and camera is not None:
        refined = hover_redetect(motion, camera, calibration_path, (x, y), noise)
        if refined is not None:
            x, y = xy(refined)
            quaternion = square_grasp_quaternion(motion, refined)
    x += GRASP_OFFSET_X_M
    validate_workspace(x, y)
    move_cartesian(motion, noise, x, y, GRASP_TCP_Z, quaternion, 'descend to grasp',
                   exact=True)
    motion.grasp(GRASP_WIDTH, speed=0.03, force=GRASP_FORCE,
                 epsilon_inner=0.005, epsilon_outer=0.005)
    move_cartesian(motion, noise, x, y, hover_z, quaternion, 'lift')
    move_home(motion, noise, 'carry to near-home pose')


def place_with_random_yaw(motion, x, y, yaw_offset, noise):
    current, _ = current_downward(motion)
    current_x = _quat_rotate(current[3:7], (1.0, 0.0, 0.0))
    home_yaw = math.atan2(current_x[1], current_x[0])
    yaw = math.atan2(math.sin(home_yaw+yaw_offset),
                     math.cos(home_yaw+yaw_offset))
    print(f'place yaw: HOME {math.degrees(home_yaw):+.1f} deg '
          f'+ offset {math.degrees(yaw_offset):+.1f} deg '
          f'= {math.degrees(yaw):+.1f} deg')
    quaternion = (math.cos(yaw/2.0), math.sin(yaw/2.0), 0.0, 0.0)
    hover_z = PLACE_TCP_Z + LIFT_HEIGHT
    move_cartesian(motion, noise, x, y, hover_z, quaternion, 'move above random place')
    move_cartesian(motion, noise, x, y, PLACE_TCP_Z, quaternion, 'descend to random place',
                   exact=True)
    motion.open_gripper(width=0.08, speed=0.05)
    move_cartesian(motion, noise, x, y, hover_z, quaternion, 'retreat after place')
    move_home(motion, noise, 'return to HOME')


def verify_placement(motion, camera, calibration_path, place_x, place_y):
    """Look again from HOME: is the cube on the target tray, near the target?"""
    try:
        selection = auto_detect_cube(motion, camera, calibration_path, timeout=10.0)
    except RuntimeError as exc:
        return False, {'error': str(exc)}
    x, y, _ = selection['visible_top_center_fr3_link0_m']
    error = math.hypot(x-place_x, y-place_y)
    on_target_tray = (y > 0) == (place_y > 0)
    success = on_target_tray and error <= PLACE_SUCCESS_TOLERANCE_M
    print(f'placement check: detected ({x:+.3f}, {y:+.3f}), '
          f'target ({place_x:+.3f}, {place_y:+.3f}), error {error*1000:.1f} mm '
          f'-> {"SUCCESS" if success else "FAIL"}')
    return success, {
        'detected_xy_m': [x, y], 'target_xy_m': [place_x, place_y],
        'error_m': error, 'on_target_tray': on_target_tray,
        'tolerance_m': PLACE_SUCCESS_TOLERANCE_M,
        'image_stamp_ns': selection['image_stamp_ns'],
    }


def run_episode(args, motion, camera, recorder, rng, episode, expected_source_sign):
    """Detect, pick, place, check. Returns (success, verification) or
    (None, None) in detection-only mode."""
    def event(name, **data):
        if recorder:
            recorder.event(name, **data)

    event('phase_detect')
    selection = auto_detect_cube(motion, camera, args.calibration)
    source_x, source_y, _ = selection['visible_top_center_fr3_link0_m']
    source_sign = 1 if source_y > 0 else -1
    if expected_source_sign is not None and source_sign != expected_source_sign:
        raise RuntimeError(
            'detected cube is on the wrong tray for the previous placement: '
            f'expected sign {expected_source_sign:+d}, detected y={source_y:+.3f}')
    place_x, place_y, place_yaw, circle_angle = sample_opposite_tray_target(
        source_y, rng)
    square = selection['depth_square']
    plan = {
        'episode': episode,
        'detected_cube_xy_m': [source_x, source_y],
        'source_tray': 'left (+Y)' if source_y > 0 else 'right (-Y)',
        'detected_size_mm': [v*1000 for v in square['side_lengths_m']],
        'detected_edge_yaw_deg': square['edge_yaw_base_deg'],
        'target_tray': 'left (+Y)' if place_y > 0 else 'right (-Y)',
        'place_xy_m': [place_x, place_y],
        'place_circle_angle_deg': math.degrees(circle_angle),
        'place_yaw_offset_from_home_deg': math.degrees(place_yaw),
        'grasp_offset_x_m': GRASP_OFFSET_X_M,
        'far_edge_shift_mm': [v*1000 for v in selection['far_edge_shift_xy_m']],
        'hover_redetect': HOVER_REDETECT,
        'execute': args.execute,
    }
    print(json.dumps(plan, indent=2))
    if not args.execute:
        print('DETECTION ONLY: robot did not move. Re-run with --execute after checking.')
        return None, None
    noise = MotionNoise(rng, enabled=not args.no_noise, recorder=recorder)
    if recorder:
        recorder.update(plan=plan, detection=selection, noise_enabled=noise.enabled,
                        noise_config={
                            'waypoint_xy_m': NOISE_WAYPOINT_XY_M,
                            'waypoint_z_m': NOISE_WAYPOINT_Z_M,
                            'home_joint_rad': NOISE_HOME_JOINT_RAD,
                            'cartesian_speed': NOISE_CARTESIAN_SPEED,
                            'joint_speed': NOISE_JOINT_SPEED,
                        })
    motion.ensure_table_collision()
    event('phase_pick')
    pick_with_noise(motion, selection, noise, camera=camera,
                    calibration_path=args.calibration, home_gripper=(episode == 1))
    event('phase_place')
    place_with_random_yaw(motion, place_x, place_y, place_yaw, noise)
    if recorder:
        recorder.update(noise=noise.log)
    event('phase_verify')
    success, verification = verify_placement(
        motion, camera, args.calibration, place_x, place_y)
    verification['target_xy_m'] = [place_x, place_y]
    return success, verification


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='move the real robot (default: detect and print only)')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--episodes', type=int, default=1,
                        help='number of automatic transfers (default: 1)')
    parser.add_argument('--continuous', action='store_true',
                        help='repeat transfers until Ctrl+C')
    parser.add_argument('--calibration', type=Path,
                        default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--data-dir', type=Path, default=Path('data/demos'),
                        help='where --execute episodes are recorded')
    parser.add_argument('--no-record', action='store_true',
                        help='do not record episodes with --execute')
    parser.add_argument('--no-noise', action='store_true',
                        help='disable waypoint / HOME / speed demonstration noise')
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be at least 1')
    rng = random.Random(args.seed)

    rclpy.init()
    motion = Fr3Motion()
    camera = CameraReceiver()
    if args.execute:
        # Start the first detection from the same tilted HOME view as later episodes.
        move_home(motion, MotionNoise(rng, enabled=False), 'move to HOME view')
    recorder = None
    if args.execute and not args.no_record:
        recorder = EpisodeRecorder(motion, args.data_dir, run_info={
            'argv': sys.argv, 'seed': args.seed,
            'calibration': json.loads(args.calibration.read_text()),
            'tray_centers': (json.loads(TRAY_CENTERS_PATH.read_text())
                             if TRAY_CENTERS_PATH.exists() else None),
        })
        instrument_motion(motion, recorder)
    try:
        episode = 0
        expected_source_sign = None
        while rclpy.ok() and (args.continuous or episode < args.episodes):
            episode += 1
            print(f'=== AUTO TRANSFER EPISODE {episode} ===')
            if recorder:
                recorder.start()
            try:
                success, verification = run_episode(
                    args, motion, camera, recorder, rng, episode, expected_source_sign)
            except BaseException as exc:
                if recorder:
                    recorder.stop(False, error=repr(exc))
                raise
            if verification is None:  # detection only
                return
            if recorder:
                recorder.stop(success, verification=verification,
                              place_error_m=verification.get('error_m'))
            expected_source_sign = 1 if verification['target_xy_m'][1] > 0 else -1
            print(f'AUTO TRANSFER EPISODE {episode} COMPLETE (success={success})')
    finally:
        if recorder:
            recorder.close()
        camera.close()
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
