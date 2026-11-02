#!/usr/bin/env python3
"""Live RGB-D visualization of what auto_cube_transfer considers a cube.

This node is perception-only and never commands the robot. Press Q to quit.
"""
import argparse
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from detect_black_cube_rgbd import depth_metres, image_bgr, tf_matrix
from auto_cube_transfer import find_valid_cube_squares, sample_opposite_tray_target
from click_pick_place_sequence import CUBE_SIZE, SURFACE_Z


def project_base_point(point_base, base_to_camera, camera_matrix):
    camera_point = np.linalg.inv(base_to_camera) @ np.asarray(
        [point_base[0], point_base[1], point_base[2], 1.0])
    if camera_point[2] <= 0.0:
        return None
    u = camera_matrix[0, 0]*camera_point[0]/camera_point[2] + camera_matrix[0, 2]
    v = camera_matrix[1, 1]*camera_point[1]/camera_point[2] + camera_matrix[1, 2]
    return int(round(u)), int(round(v))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path,
                        default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--save', type=Path,
                        default=Path('/tmp/auto_cube_live.jpg'))
    parser.add_argument('--seed', type=int, default=None,
                        help='optional fixed placement seed; omitted means a new target each launch')
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    rclpy.init()
    node = rclpy.create_node('auto_cube_visualizer')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
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
    publisher = node.create_publisher(Image, '/auto_cube/debug_image', 1)
    output_message = Image()
    output_message.encoding = 'bgr8'
    output_message.is_bigendian = 0
    window = 'Automatic cube perception (Q: quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    last_process = 0.0
    last_stamp = None
    period = 1.0 / args.rate
    rng = random.Random(args.seed)
    targets = {}
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not colors or not depths or not infos:
                continue
            now = time.monotonic()
            if now-last_process < period:
                if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                    break
                continue
            color_msg, depth_msg, info = colors[-1], depths[-1], infos[-1]
            stamp = Time.from_msg(color_msg.header.stamp)
            if stamp.nanoseconds == last_stamp:
                continue
            depth_stamp = Time.from_msg(depth_msg.header.stamp)
            if abs(stamp.nanoseconds-depth_stamp.nanoseconds) > 50_000_000:
                continue
            try:
                wrist = tf_buffer.lookup_transform('fr3_link0', 'fr3_link8', stamp)
            except TransformException:
                continue
            last_process, last_stamp = now, stamp.nanoseconds
            frame = image_bgr(color_msg)
            depth = depth_metres(depth_msg)
            camera_matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
            base_to_camera = tf_matrix(wrist) @ link8_to_camera
            candidates, valid, _ = find_valid_cube_squares(
                frame, depth, camera_matrix, base_to_camera)
            view = frame.copy()
            for item in candidates:
                contour = item['contour']
                cv2.drawContours(view, [contour], -1, (0, 255, 255), 1)
            for _, _, _, square, _ in valid:
                polygon = np.rint(square['box_pixels']).astype(np.int32)
                cv2.polylines(view, [polygon], True, (255, 0, 255), 2)
            status = f'dark candidates: {len(candidates)} | valid cubes: {len(valid)}'
            color = (0, 255, 0) if valid else (0, 0, 255)
            if valid:
                selected = valid[0][3]
                polygon = np.rint(selected['box_pixels']).astype(np.int32)
                cv2.polylines(view, [polygon], True, (0, 255, 0), 4)
                x, y = selected['centre_xy_m']
                a, b = selected['side_lengths_m']
                status += (f' | SELECTED x={x:.3f} y={y:.3f} '
                           f'{a*1000:.1f}x{b*1000:.1f}mm '
                           f"yaw={selected['edge_yaw_base_deg']:+.1f}deg")
                source_sign = 1 if y > 0.0 else -1
                if source_sign not in targets:
                    targets[source_sign] = sample_opposite_tray_target(y, rng)
                target_x, target_y, target_yaw, _ = targets[source_sign]
                # Hand-eye XY is accurate, but its absolute base Z has a known
                # offset. Infer that offset from the detected cube top so the
                # target floor point projects onto the live image correctly.
                raw_z_offset = (selected['centre_z_base_raw_m']
                                - (SURFACE_Z+CUBE_SIZE))
                pixel = project_base_point(
                    (target_x, target_y, SURFACE_Z+raw_z_offset),
                    base_to_camera, camera_matrix)
                if pixel is not None:
                    target_u, target_v = pixel
                    if 0 <= target_u < view.shape[1] and 0 <= target_v < view.shape[0]:
                        cv2.drawMarker(view, (target_u, target_v), (0, 0, 255),
                                       cv2.MARKER_CROSS, 28, 4)
                        cv2.circle(view, (target_u, target_v), 10, (0, 0, 255), 3)
                        cv2.putText(
                            view, (f'PLACE x={target_x:.3f} y={target_y:.3f} '
                                   f'yaw={np.degrees(target_yaw):.0f}'),
                            (max(4, target_u-105), max(50, target_v-18)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 2,
                            cv2.LINE_AA)
            cv2.rectangle(view, (0, 0), (view.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(view, status, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 1, cv2.LINE_AA)
            args.save.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.save), view)
            output_message.header = color_msg.header
            output_message.height, output_message.width = view.shape[:2]
            output_message.step = output_message.width*3
            output_message.data = view.tobytes()
            publisher.publish(output_message)
            cv2.imshow(window, view)
            if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
