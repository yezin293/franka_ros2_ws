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
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from detect_black_cube_rgbd import depth_metres, image_bgr, tf_matrix
from select_black_cube_rgbd import dark_candidates, estimate_square_from_depth
from click_pick_place_sequence import (
    CUBE_SIZE, LIFT_HEIGHT, PLACE_TCP_Z, SURFACE_Z,
    execute_cartesian, execute_near_home, pick_and_lift, pose,
)
from fr3_skills.motion import Fr3Motion


TRAY_CENTER_X = 0.415
TRAY_CENTER_ABS_Y = 0.130
PLACE_RADIUS_MIN = 0.040
PLACE_RADIUS_MAX = 0.050
AUTO_DARK_THRESHOLD = 75
TRAY_CENTERS_PATH = Path('calibration_data/tray_centers.json')


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


def auto_detect_cube(motion, calibration_path, timeout=30.0):
    calibration = json.loads(calibration_path.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    # Reset perception after every return to HOME. This deliberately discards
    # every frame from the previous transfer and starts with empty buffers.
    perception = getattr(motion, '_auto_cube_perception', None)
    if perception is not None:
        for subscription in perception['subscriptions']:
            motion.destroy_subscription(subscription)
    perception = {
        'colors': deque(maxlen=30),
        'depths': deque(maxlen=30),
        'infos': deque(maxlen=5),
    }
    perception['subscriptions'] = [
        motion.create_subscription(
            Image, '/camera/camera/color/image_raw',
            perception['colors'].append, qos_profile_sensor_data),
        motion.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            perception['depths'].append, qos_profile_sensor_data),
        motion.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            perception['infos'].append, qos_profile_sensor_data),
    ]
    motion._auto_cube_perception = perception
    colors = perception['colors']
    depths = perception['depths']
    infos = perception['infos']
    deadline = time.monotonic() + timeout
    last_error = 'waiting for RGB-D after perception reset'
    warned_stamp_gap = False
    while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(motion, timeout_sec=0.05)
            if not colors or not depths or not infos:
                continue
            # Both queues were created at HOME, so every message is new.
            color_msg = colors[-1]
            color_ns = Time.from_msg(color_msg.header.stamp).nanoseconds
            depth_msg = min(
                depths,
                key=lambda depth: abs(
                    color_ns-Time.from_msg(depth.header.stamp).nanoseconds))
            info = infos[-1]
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
            valid.sort(key=lambda value: value[0])
            if len(valid) > 1 and valid[1][0] < valid[0][0] + 0.006:
                raise RuntimeError(
                    f'{len(valid)} similarly plausible cubes detected; refusing automatic choice')
            _, u, v, square, contour = valid[0]
            x, y = square['centre_xy_m']
            selection = {
                'visible_top_center_fr3_link0_m': [x, y, SURFACE_Z+CUBE_SIZE],
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
            cv2.imwrite('calibration_data/auto_cube_detection.jpg', debug)
            Path('calibration_data/auto_cube_detection.json').write_text(
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
        # Uniformly sample area in the 40--50 mm annulus around the measured
        # tray centre, instead of repeatedly using one exact radius.
        radius = math.sqrt(rng.uniform(
            PLACE_RADIUS_MIN**2, PLACE_RADIUS_MAX**2))
        x = centre_x + radius*math.cos(angle)
        y = centre_y + radius*math.sin(angle)
        if 0.36 <= x <= 0.47 and 0.08 <= abs(y) <= 0.19:
            yaw = rng.uniform(0.0, math.pi/2.0)
            return x, y, yaw, angle
    raise RuntimeError('failed to sample a safe point on the opposite tray')


def place_with_random_yaw(motion, x, y, yaw):
    quaternion = (math.cos(yaw/2.0), math.sin(yaw/2.0), 0.0, 0.0)
    hover_z = PLACE_TCP_Z + LIFT_HEIGHT
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'move above random place')
    execute_cartesian(motion, pose(x, y, PLACE_TCP_Z, quaternion), 'descend to random place')
    motion.open_gripper(width=0.08, speed=0.05)
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'retreat after place')
    execute_near_home(motion)


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
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be at least 1')
    rng = random.Random(args.seed)

    rclpy.init()
    motion = Fr3Motion()
    try:
        episode = 0
        expected_source_sign = None
        while rclpy.ok() and (args.continuous or episode < args.episodes):
            episode += 1
            print(f'=== AUTO TRANSFER EPISODE {episode} ===')
            selection = auto_detect_cube(motion, args.calibration)
            source_x, source_y, _ = selection['visible_top_center_fr3_link0_m']
            source_sign = 1 if source_y > 0 else -1
            if expected_source_sign is not None and source_sign != expected_source_sign:
                raise RuntimeError(
                    'detected cube is on the wrong tray for the previous placement: '
                    f'expected sign {expected_source_sign:+d}, detected y={source_y:+.3f}')
            place_x, place_y, place_yaw, circle_angle = sample_opposite_tray_target(
                source_y, rng)
            square = selection['depth_square']
            print(json.dumps({
                'episode': episode,
                'detected_cube_xy_m': [source_x, source_y],
                'source_tray': 'left (+Y)' if source_y > 0 else 'right (-Y)',
                'detected_size_mm': [v*1000 for v in square['side_lengths_m']],
                'detected_edge_yaw_deg': square['edge_yaw_base_deg'],
                'target_tray': 'left (+Y)' if place_y > 0 else 'right (-Y)',
                'place_xy_m': [place_x, place_y],
                'place_circle_angle_deg': math.degrees(circle_angle),
                'place_yaw_deg': math.degrees(place_yaw),
                'execute': args.execute,
            }, indent=2))
            if not args.execute:
                print('DETECTION ONLY: robot did not move. Re-run with --execute after checking.')
                return
            motion.ensure_table_collision()
            pick_and_lift(motion, selection, home_gripper=(episode == 1))
            place_with_random_yaw(motion, place_x, place_y, place_yaw)
            expected_source_sign = 1 if place_y > 0 else -1
            print(f'AUTO TRANSFER EPISODE {episode} COMPLETE')
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
