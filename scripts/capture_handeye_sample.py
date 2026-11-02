"""Save one stationary eye-in-hand sample. Never sends robot commands.

Run after sourcing ROS and the workspace:
  /usr/bin/python3 scripts/capture_handeye_sample.py

Board: 11 x 8 squares, 16 mm square size (10 x 7 internal corners).
Output transforms use metres. Capturing is not calibration/validation.
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


def transform_values(t):
    p, q = t.transform.translation, t.transform.rotation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def displacement(a, b):
    pa, qa = transform_values(a)
    pb, qb = transform_values(b)
    cosine = float(np.clip(abs(np.dot(qa, qb) / (np.linalg.norm(qa) * np.linalg.norm(qb))), 0, 1))
    return float(np.linalg.norm(pa - pb)), math.degrees(2 * math.acos(cosine))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[1] / 'calibration_data' / 'handeye_d435i')
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('capture_handeye_sample')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    images, infos = [], []

    def receive_image(msg):
        images[:] = [msg]

    def receive_info(msg):
        infos[:] = [msg]

    subscriptions = [
        node.create_subscription(Image, '/camera/camera/color/image_raw', receive_image, qos_profile_sensor_data),
        node.create_subscription(CameraInfo, '/camera/camera/color/camera_info', receive_info, qos_profile_sensor_data),
    ]
    start = time.monotonic()
    previous_stamp = None
    pending = None
    stationary = []
    last_status = 'waiting for RGB, CameraInfo and wrist TF'
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.03)
            if not images or not infos:
                continue
            if pending is None:
                pending = images[-1]
            msg, info = pending, infos[-1]
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == previous_stamp:
                pending = None
                continue
            # Give the TF stream time to reach this image's timestamp.
            try:
                wrist = buffer.lookup_transform('fr3_link0', 'fr3_link8', stamp)
            except TransformException as exc:
                last_status = 'waiting for wrist TF at image timestamp: ' + str(exc)
                if 'past' in str(exc).lower() or (node.get_clock().now().nanoseconds - stamp.nanoseconds) > 1_000_000_000:
                    pending = None
                continue
            previous_stamp = stamp.nanoseconds
            pending = None
            age = (node.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            if not -0.1 <= age <= 1.0:
                stationary.clear()
                last_status = 'image timestamp is stale or clock is inconsistent'
                continue
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise RuntimeError('unsupported image encoding: ' + msg.encoding)
            if (msg.width, msg.height, msg.header.frame_id) != (info.width, info.height, info.header.frame_id):
                raise RuntimeError('RGB and CameraInfo dimensions/frame mismatch')
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, (10, 7), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not found:
                stationary.clear()
                last_status = '10 x 7 internal corners not found'
                continue
            corners = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)
            # Check both visual and wrist stability for >= 0.5 s.
            if stationary:
                delta_m, delta_deg = displacement(stationary[0][1], wrist)
                visual_delta = float(np.max(np.linalg.norm(corners - stationary[0][2], axis=2)))
                # At steep viewing angles, sub-pixel corner refinement can jitter
                # by a few pixels even while the rigidly mounted camera is still.
                # Keep the wrist limits strict and allow a modest image-space
                # tolerance so valid oblique views are not rejected forever.
                if delta_m > 0.001 or delta_deg > 0.5 or visual_delta > 5.0:
                    last_status = (
                        f'not stationary: wrist {delta_m * 1000:.2f} mm, '
                        f'{delta_deg:.2f} deg; corners {visual_delta:.2f} px'
                    )
                    stationary.clear()
            stationary.append((stamp.nanoseconds, wrist, corners))
            if len(stationary) < 3 or (stamp.nanoseconds - stationary[0][0]) / 1e9 < 0.5:
                last_status = 'waiting for a stationary, visible board for 0.5 s'
                continue
            camera_matrix = np.array(info.k, dtype=float).reshape(3, 3)
            if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
                raise RuntimeError('unsupported distortion model: ' + info.distortion_model)
            object_points = np.zeros((70, 3), dtype=np.float64)
            object_points[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2) * 0.016
            ok, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, np.array(info.d))
            if not ok:
                raise RuntimeError('solvePnP failed')
            rotation, _ = cv2.Rodrigues(rvec)
            camera_points = (rotation @ object_points.T + tvec).T
            if np.any(camera_points[:, 2] <= 0):
                raise RuntimeError('board solution is behind camera')
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, np.array(info.d))
            error = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2))))
            if error > 1.5:
                raise RuntimeError(f'board reprojection RMS too high: {error:.3f} px')
            position, quaternion = transform_values(wrist)
            target_to_camera = np.eye(4)
            target_to_camera[:3, :3] = rotation
            target_to_camera[:3, 3] = tvec.flatten()
            sample = {
                'schema_version': 1,
                'image_stamp_ns': stamp.nanoseconds,
                'base_frame': 'fr3_link0', 'gripper_frame': 'fr3_link8',
                'camera_frame': msg.header.frame_id,
                'board': {'squares': [11, 8], 'inner_corners': [10, 7], 'square_size_m': 0.016},
                'image_size': [msg.width, msg.height],
                'camera_matrix': camera_matrix.tolist(), 'distortion': list(info.d),
                'distortion_model': info.distortion_model,
                'gripper_to_base': {'translation_m': position.tolist(), 'quaternion_xyzw': quaternion.tolist()},
                'target_to_camera_matrix': target_to_camera.tolist(),
                'corners_px': corners.reshape(-1, 2).tolist(),
                'board_reprojection_rms_px': error,
                'stationary_window_s': (stamp.nanoseconds - stationary[0][0]) / 1e9,
                'note': 'Matched wrist TF at RGB timestamp; board corner ordering must stay consistent across views. Not a calibrated hand-eye result.',
            }
            folder = args.out / ('sample_' + str(stamp.nanoseconds))
            folder.mkdir(parents=True, exist_ok=False)
            if not cv2.imwrite(str(folder / 'rgb.png'), image):
                raise RuntimeError('could not save RGB image')
            cv2.drawChessboardCorners(image, (10, 7), corners.astype(np.float32), True)
            if not cv2.imwrite(str(folder / 'corners.png'), image):
                raise RuntimeError('could not save annotated image')
            (folder / 'sample.json').write_text(json.dumps(sample, indent=2) + '\n')
            print(f'SAVED: {folder}\nCorners: 70; board reprojection RMS: {error:.3f} px')
            print('One sample saved. Hand-eye calibration has NOT been solved yet.')
            return
        raise RuntimeError(f'capture timed out: {last_status}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
