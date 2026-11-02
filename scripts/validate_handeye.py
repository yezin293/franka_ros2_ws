#!/usr/bin/env python3
"""Validate eye-in-hand calibration at a new robot pose without refitting it.

The checkerboard must remain fixed. The script rejects a pose that is too
similar to a calibration sample, then reconstructs the board pose in
fr3_link0 and compares it with the fixed-board pose saved by calibration.
It never sends robot commands and never adds a calibration sample.
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


def quaternion_to_matrix(q):
    x, y, z, w = map(float, q)
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ])


def transform(rotation, translation):
    value = np.eye(4)
    value[:3, :3] = np.asarray(rotation).reshape(3, 3)
    value[:3, 3] = np.asarray(translation).reshape(3)
    return value


def tf_matrix(message):
    p, q = message.transform.translation, message.transform.rotation
    return transform(quaternion_to_matrix([q.x, q.y, q.z, q.w]), [p.x, p.y, p.z])


def rotation_error_deg(a, b):
    relative = a.T @ b
    return math.degrees(math.acos(np.clip((np.trace(relative)-1)/2, -1, 1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--samples', type=Path, default=Path('calibration_data/handeye_d435i'))
    parser.add_argument('--out', type=Path, default=Path('calibration_data/handeye_validation'))
    parser.add_argument(
        '--reference-validation', type=Path,
        help='Compare with a previous validation result instead of the calibration-time board pose')
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--min-new-translation-mm', type=float, default=30)
    parser.add_argument('--min-new-rotation-deg', type=float, default=10)
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    reference_source = 'calibration estimated fixed board pose'
    reference_target = np.asarray(
        calibration['estimated_fixed_board_pose_in_base']['matrix'], dtype=float)
    if args.reference_validation:
        reference_data = json.loads(args.reference_validation.read_text())
        reference_target = np.asarray(
            reference_data['predicted_board_pose_in_base_matrix'], dtype=float)
        reference_source = str(args.reference_validation)
    training_poses = []
    for path in sorted(args.samples.glob('sample_*/sample.json')):
        data = json.loads(path.read_text())
        pose = data['gripper_to_base']
        training_poses.append(transform(
            quaternion_to_matrix(pose['quaternion_xyzw']), pose['translation_m']))

    rclpy.init()
    node = rclpy.create_node('validate_handeye')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    images, infos = [], []
    subscriptions = [
        node.create_subscription(Image, '/camera/camera/color/image_raw',
                                 lambda msg: images.__setitem__(slice(None), [msg]),
                                 qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 lambda msg: infos.__setitem__(slice(None), [msg]),
                                 qos_profile_sensor_data),
    ]
    start = time.monotonic()
    last_stamp = None
    last_status = 'waiting for synchronized RGB, CameraInfo and wrist TF'
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not images or not infos:
                continue
            msg, info = images[-1], infos[-1]
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == last_stamp:
                continue
            try:
                wrist_message = buffer.lookup_transform('fr3_link0', 'fr3_link8', stamp)
            except TransformException as exc:
                last_status = 'waiting for wrist TF at image timestamp: ' + str(exc)
                continue
            last_stamp = stamp.nanoseconds
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise RuntimeError('unsupported encoding: ' + msg.encoding)
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            image = image[:, :msg.width*3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            found, corners = cv2.findChessboardCornersSB(
                cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (10, 7),
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not found:
                last_status = '10 x 7 internal corners not found'
                continue
            corners = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)
            camera_matrix = np.asarray(info.k, dtype=float).reshape(3, 3)
            distortion = np.asarray(info.d, dtype=float)
            points = np.zeros((70, 3), dtype=np.float64)
            points[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2) * 0.016
            ok, rvecs, tvecs, pnp_errors = cv2.solvePnPGeneric(
                points, corners, camera_matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE)
            if not ok or not rvecs:
                last_status = 'IPPE solvePnP failed'
                continue
            pnp_errors = np.asarray(pnp_errors, dtype=float).reshape(-1)
            selected = int(np.argmin(pnp_errors))
            rvec, tvec = rvecs[selected], tvecs[selected]
            rotation, _ = cv2.Rodrigues(rvec)
            projected, _ = cv2.projectPoints(points, rvec, tvec, camera_matrix, distortion)
            reprojection = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
            base_to_link8 = tf_matrix(wrist_message)
            nearest_translation = min(
                np.linalg.norm(base_to_link8[:3, 3]-pose[:3, 3])*1000
                for pose in training_poses)
            nearest_rotation = min(
                rotation_error_deg(base_to_link8[:3, :3], pose[:3, :3])
                for pose in training_poses)
            if (nearest_translation < args.min_new_translation_mm
                    and nearest_rotation < args.min_new_rotation_deg):
                raise RuntimeError(
                    'pose is too similar to calibration data: nearest translation '
                    f'{nearest_translation:.1f} mm, nearest rotation {nearest_rotation:.1f} deg; '
                    'move at least 30 mm or rotate at least 10 deg')
            camera_to_target = transform(rotation, tvec)
            prediction = base_to_link8 @ link8_to_camera @ camera_to_target
            candidates = [('direct', prediction)]
            if args.reference_validation:
                # A rectangular checkerboard is unchanged by a 180-degree turn.
                # Account for reversed corner numbering while retaining the same
                # physical board coordinate frame used by the reference capture.
                checkerboard_flip = transform(
                    np.diag([-1.0, -1.0, 1.0]), [9 * 0.016, 6 * 0.016, 0.0])
                candidates.append(('checkerboard_180deg', prediction @ checkerboard_flip))
            comparisons = []
            for name, candidate in candidates:
                comparisons.append({
                    'candidate': name,
                    'translation_error_mm': float(np.linalg.norm(
                        candidate[:3, 3]-reference_target[:3, 3])*1000),
                    'rotation_error_deg': rotation_error_deg(
                        reference_target[:3, :3], candidate[:3, :3]),
                    'matrix': candidate,
                })
            selected_comparison = min(
                comparisons,
                key=lambda item: item['translation_error_mm'] + item['rotation_error_deg'])
            prediction = selected_comparison['matrix']
            translation_error = selected_comparison['translation_error_mm']
            rotation_error = selected_comparison['rotation_error_deg']
            result = {
                'schema_version': 1,
                'independent_from_fit': True,
                'image_stamp_ns': stamp.nanoseconds,
                'nearest_training_pose_translation_mm': nearest_translation,
                'nearest_training_pose_rotation_deg': nearest_rotation,
                'board_reprojection_rms_px': reprojection,
                'pnp_method': 'SOLVEPNP_IPPE',
                'pnp_candidate_reprojection_errors_px': pnp_errors.tolist(),
                'pnp_selected_candidate': selected,
                'fixed_board_translation_error_mm': translation_error,
                'fixed_board_rotation_error_deg': rotation_error,
                'validation_reference_source': reference_source,
                'checkerboard_order_candidate': selected_comparison['candidate'],
                'checkerboard_order_comparisons': [
                    {key: value for key, value in item.items() if key != 'matrix'}
                    for item in comparisons
                ],
                'predicted_board_pose_in_base_matrix': prediction.tolist(),
                'reference_board_pose_in_base_matrix': reference_target.tolist(),
                'note': 'Validation sample was not used to refit calibration.',
            }
            args.out.mkdir(parents=True, exist_ok=True)
            folder = args.out / ('validation_' + str(stamp.nanoseconds))
            folder.mkdir(exist_ok=False)
            cv2.drawChessboardCorners(image, (10, 7), corners.astype(np.float32), True)
            cv2.imwrite(str(folder/'corners.png'), image)
            (folder/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
            print('SAVED:', folder)
            return
        raise RuntimeError('validation timed out: ' + last_status)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
