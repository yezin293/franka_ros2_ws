#!/usr/bin/env python3
"""Click a dark object in a frozen RGB-D frame and save its FR3-base pose.

This is perception-only: it never sends a command to the robot.
Left click a highlighted candidate to select it; press R to refresh or Q to quit.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from detect_black_cube_rgbd import (
    depth_metres, image_bgr, quaternion_to_matrix, tf_matrix)


def dark_candidates(frame, threshold, min_area, max_area):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, threshold)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = gray.shape
    result = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area <= area <= max_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
            continue
        result.append({'contour': contour, 'box': (x, y, w, h), 'area': area})
    return sorted(result, key=lambda item: item['area'], reverse=True)


def draw_candidates(frame, candidates):
    view = frame.copy()
    for index, item in enumerate(candidates, 1):
        contour = item['contour']
        x, y, w, h = item['box']
        cv2.drawContours(view, [contour], -1, (0, 255, 255), 2)
        cv2.putText(view, str(index), (x, max(18, y-4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(view, 'Click object | R: refresh | Q: quit', (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return view


def estimate_square_from_depth(depth, u, v, camera_matrix, base_to_camera,
                               expected_size_m=0.040):
    """Fit a square in base XY to the connected, equal-depth patch at (u,v)."""
    cx, cy = int(round(u)), int(round(v))
    height, width = depth.shape
    if not (0 <= cx < width and 0 <= cy < height):
        raise ValueError('clicked pixel is outside the depth image')
    y0, y1 = max(0, cy-100), min(height, cy+101)
    x0, x1 = max(0, cx-100), min(width, cx+101)
    roi = depth[y0:y1, x0:x1]
    local_x, local_y = cx-x0, cy-y0
    patch = roi[max(0, local_y-5):local_y+6, max(0, local_x-5):local_x+6]
    seed_values = patch[np.isfinite(patch) & (patch > 0.10) & (patch < 2.0)]
    if seed_values.size < 20:
        raise ValueError('not enough valid depth around click')
    seed_depth = float(np.median(seed_values))
    # Keep this tight: at an oblique view the cube side can remain within
    # several millimetres of the top near an edge and merge into a long patch.
    equal_depth = (np.isfinite(roi) & (roi > 0.10)
                   & (np.abs(roi-seed_depth) <= 0.003)).astype(np.uint8)
    equal_depth = cv2.morphologyEx(equal_depth, cv2.MORPH_CLOSE,
                                   np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(equal_depth, 8)
    label = int(labels[local_y, local_x])
    if label == 0 or stats[label, cv2.CC_STAT_AREA] < 120:
        raise ValueError('clicked depth patch is not a connected planar region')
    ys, xs = np.where(labels == label)
    full_u = xs.astype(np.float64) + x0
    full_v = ys.astype(np.float64) + y0
    values = roi[ys, xs].astype(np.float64)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    ppx, ppy = camera_matrix[0, 2], camera_matrix[1, 2]
    camera_points = np.column_stack(((full_u-ppx)*values/fx,
                                     (full_v-ppy)*values/fy,
                                     values, np.ones_like(values)))
    base_points = (base_to_camera @ camera_points.T).T
    centred = base_points[:, :3] - np.mean(base_points[:, :3], axis=0)
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    plane_normal = vh[-1]
    plane_normal /= np.linalg.norm(plane_normal)
    if plane_normal[2] < 0:
        plane_normal = -plane_normal
    tilt_deg = math.degrees(math.acos(np.clip(plane_normal[2], -1.0, 1.0)))
    # A valid cube top must be horizontal in the robot-base frame. A 40 mm
    # front/side face is also square, so size alone cannot distinguish it.
    if tilt_deg > 20.0:
        raise ValueError(
            f'clicked square is a side/front face: plane tilt {tilt_deg:.1f} deg '
            'from horizontal (maximum 20 deg)')
    rectangle = cv2.minAreaRect(base_points[:, :2].astype(np.float32))
    (centre_x, centre_y), (side_a, side_b), angle_deg = rectangle
    short_side, long_side = sorted((float(side_a), float(side_b)))
    if not (0.028 <= short_side <= 0.052 and 0.030 <= long_side <= 0.060):
        raise ValueError(
            f'depth plane size {short_side*1000:.1f} x {long_side*1000:.1f} mm '
            'does not match a 40 mm cube top')
    if long_side / short_side > 1.35:
        raise ValueError('depth plane is not square enough')
    box_pixels = cv2.boxPoints(cv2.minAreaRect(
        np.column_stack((full_u, full_v)).astype(np.float32)))
    # OpenCV's angle describes the side associated with rectangle width.
    yaw = math.radians(float(angle_deg))
    if side_a < side_b:
        yaw += math.pi / 2.0
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return {
        'centre_xy_m': [float(centre_x), float(centre_y)],
        'centre_z_base_raw_m': float(np.mean(base_points[:, 2])),
        'side_lengths_m': [short_side, long_side],
        'edge_yaw_base_rad': yaw,
        'edge_yaw_base_deg': math.degrees(yaw),
        'depth_seed_m': seed_depth,
        'component_pixels': int(len(xs)),
        'plane_normal_base': plane_normal.tolist(),
        'plane_tilt_from_horizontal_deg': tilt_deg,
        'box_pixels': box_pixels,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--out', type=Path,
                        default=Path('calibration_data/selected_black_cube.json'))
    parser.add_argument('--save', type=Path,
                        default=Path('calibration_data/selected_black_cube.jpg'))
    parser.add_argument('--threshold', type=int, default=75)
    parser.add_argument('--min-area', type=float, default=100.0)
    parser.add_argument('--max-area', type=float, default=80000.0)
    parser.add_argument('--cube-size-mm', type=float, default=40.0)
    parser.add_argument(
        '--surface-z', type=float, default=None,
        help='Known support-surface z in fr3_link0; anchors cube top z to surface + cube size')
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    rclpy.init()
    node = rclpy.create_node('select_black_cube_rgbd')
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    colors, depths, infos = [], [], []
    subscriptions = [
        node.create_subscription(Image, '/camera/camera/color/image_raw',
                                 lambda m: colors.__setitem__(slice(None), [m]),
                                 qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                                 lambda m: depths.__setitem__(slice(None), [m]),
                                 qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 lambda m: infos.__setitem__(slice(None), [m]),
                                 qos_profile_sensor_data),
    ]
    window = 'Select black cube (no robot motion)'
    clicked = []

    def on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked[:] = [(x, y)]

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    refresh = True
    snapshot = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if refresh and colors and depths and infos:
                color_msg, depth_msg, info = colors[-1], depths[-1], infos[-1]
                color_ns = Time.from_msg(color_msg.header.stamp).nanoseconds
                depth_ns = Time.from_msg(depth_msg.header.stamp).nanoseconds
                if abs(color_ns-depth_ns) <= 50_000_000:
                    try:
                        wrist = tf_buffer.lookup_transform(
                            'fr3_link0', 'fr3_link8', Time.from_msg(color_msg.header.stamp))
                    except TransformException:
                        continue
                    frame = image_bgr(color_msg)
                    depth = depth_metres(depth_msg)
                    candidates = dark_candidates(
                        frame, args.threshold, args.min_area, args.max_area)
                    snapshot = (frame, depth, info, wrist, color_ns, candidates)
                    refresh = False
                    clicked.clear()
            if snapshot is None:
                continue
            frame, depth, info, wrist, stamp_ns, candidates = snapshot
            cv2.imshow(window, draw_candidates(frame, candidates))
            key = cv2.waitKey(20) & 0xff
            if key in (ord('q'), 27):
                print('Selection cancelled; no robot command was sent.')
                return
            if key == ord('r'):
                refresh = True
                continue
            if not clicked:
                continue
            click_x, click_y = clicked.pop()
            containing = [item for item in candidates
                          if cv2.pointPolygonTest(
                              item['contour'], (float(click_x), float(click_y)), False) >= 0]
            selection_mode = 'detected_dark_contour'
            if containing:
                selected = min(containing, key=lambda item: item['area'])
            else:
                # Manual selection is the safety authority. A dark cube can be
                # connected to its shadow or the image edge, so segmentation
                # may intentionally reject it even though the user sees it.
                # Use a small region centred exactly where the user clicked.
                radius = 12
                points = cv2.ellipse2Poly(
                    (click_x, click_y), (radius, radius), 0, 0, 360, 10)
                contour = points.reshape(-1, 1, 2).astype(np.int32)
                bx = max(0, click_x-radius)
                by = max(0, click_y-radius)
                selected = {
                    'contour': contour,
                    'box': (bx, by, radius*2+1, radius*2+1),
                    'area': cv2.contourArea(contour),
                }
                selection_mode = 'manual_click_depth_patch'
            contour = selected['contour']
            object_mask = np.zeros(depth.shape, np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, -1)
            object_mask = cv2.erode(object_mask, np.ones((5, 5), np.uint8))
            valid = depth[(object_mask > 0) & np.isfinite(depth)
                          & (depth > 0.10) & (depth < 2.0)]
            if valid.size < 20:
                print(f'Only {valid.size} valid depth pixels; press R and try again.')
                continue
            moments = cv2.moments(contour)
            u = moments['m10']/moments['m00']
            v = moments['m01']/moments['m00']
            z = float(np.median(valid))
            k = np.asarray(info.k, dtype=float).reshape(3, 3)
            point_camera = np.array([(u-k[0, 2])*z/k[0, 0],
                                     (v-k[1, 2])*z/k[1, 1], z, 1.0])
            base_to_camera = tf_matrix(wrist) @ link8_to_camera
            point_base = base_to_camera @ point_camera
            square = None
            square_error = None
            try:
                square = estimate_square_from_depth(
                    depth, u, v, k, base_to_camera,
                    expected_size_m=args.cube_size_mm/1000.0)
                point_base[0], point_base[1] = square['centre_xy_m']
            except ValueError as exc:
                square_error = str(exc)
            raw_point_base = point_base[:3].copy()
            if args.surface_z is not None:
                point_base[2] = args.surface_z + args.cube_size_mm / 1000.0
            x, y, w, h = selected['box']
            chosen = frame.copy()
            cv2.drawContours(chosen, [contour], -1, (0, 255, 0), 3)
            cv2.circle(chosen, (round(u), round(v)), 6, (0, 255, 255), -1)
            if square is not None:
                cv2.polylines(chosen, [np.rint(square['box_pixels']).astype(np.int32)],
                              True, (255, 0, 255), 2)
            label = f'selected base [{point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}]'
            cv2.putText(chosen, label, (max(5, x), max(25, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.save.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.save), chosen)
            result = {
                'schema_version': 1,
                'image_stamp_ns': stamp_ns,
                'clicked_pixel_uv': [click_x, click_y],
                'selection_mode': selection_mode,
                'detected_centroid_uv': [float(u), float(v)],
                'box_xywh': [x, y, w, h],
                'cube_size_m': args.cube_size_mm / 1000.0,
                'depth_to_visible_top_m': z,
                'depth_mad_mm': float(np.median(np.abs(valid-z))*1000.0),
                'point_camera_color_optical_m': point_camera[:3].tolist(),
                'raw_visible_top_center_fr3_link0_m': raw_point_base.tolist(),
                'visible_top_center_fr3_link0_m': point_base[:3].tolist(),
                'support_surface_z_m': args.surface_z,
                'z_source': ('known surface + cube size' if args.surface_z is not None
                             else 'hand-eye RGB-D transform'),
                'depth_square_valid': square is not None,
                'depth_square_error': square_error,
                'depth_square': (None if square is None else {
                    key: value for key, value in square.items() if key != 'box_pixels'
                }),
                'debug_image': str(args.save),
                'safe_to_execute': False,
                'note': 'User-selected perception result only; no robot command was sent.',
            }
            args.out.write_text(json.dumps(result, indent=2)+'\n')
            cv2.imshow(window, chosen)
            cv2.waitKey(500)
            print(json.dumps(result, indent=2))
            print('SAVED:', args.out)
            return
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
