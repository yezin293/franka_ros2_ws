#!/usr/bin/env python3
"""Detect a dark cube with aligned RealSense depth; never commands the robot."""
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


def tf_matrix(message):
    t = message.transform.translation
    q = message.transform.rotation
    value = np.eye(4)
    value[:3, :3] = quaternion_to_matrix([q.x, q.y, q.z, q.w])
    value[:3, 3] = [t.x, t.y, t.z]
    return value


def image_bgr(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise RuntimeError('unsupported color encoding: ' + msg.encoding)
    value = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    value = value[:, :msg.width*3].reshape(msg.height, msg.width, 3).copy()
    return cv2.cvtColor(value, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else value


def depth_metres(msg):
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
        return raw[:, :msg.width].astype(np.float32) * 0.001
    if msg.encoding == '32FC1':
        raw = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)
        return raw[:, :msg.width].copy()
    raise RuntimeError('unsupported depth encoding: ' + msg.encoding)


def detect_dark_cube(frame, threshold, min_area, max_area):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, threshold)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    height, width = gray.shape
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area <= area <= max_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
            continue
        aspect = w / float(h)
        rectangularity = area / float(w*h)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull_area if hull_area else 0.0
        if not 0.45 <= aspect <= 2.2 or rectangularity < 0.45 or solidity < 0.75:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            continue
        centre = (moments['m10']/moments['m00'], moments['m01']/moments['m00'])
        squareness = math.exp(-abs(math.log(aspect)))
        score = area * rectangularity * solidity * squareness
        candidates.append((score, contour, centre, (x, y, w, h)))
    return max(candidates, key=lambda item: item[0]) if candidates else None, mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--save', type=Path, default=Path('/tmp/black_cube_detection.jpg'))
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--threshold', type=int, default=75)
    parser.add_argument('--min-area', type=float, default=150.0)
    parser.add_argument('--max-area', type=float, default=50000.0)
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text())
    link8_to_camera = np.asarray(
        calibration['link8_T_camera_color_optical']['matrix'], dtype=float)
    rclpy.init()
    node = rclpy.create_node('detect_black_cube_rgbd')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
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
    start = time.monotonic()
    status = 'waiting for RGB, aligned depth and CameraInfo'
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not colors or not depths or not infos:
                continue
            color_msg, depth_msg, info = colors[-1], depths[-1], infos[-1]
            color_ns = Time.from_msg(color_msg.header.stamp).nanoseconds
            depth_ns = Time.from_msg(depth_msg.header.stamp).nanoseconds
            if abs(color_ns-depth_ns) > 50_000_000:
                status = 'waiting for synchronized RGB and depth'
                continue
            frame, depth = image_bgr(color_msg), depth_metres(depth_msg)
            detection, dark_mask = detect_dark_cube(
                frame, args.threshold, args.min_area, args.max_area)
            if detection is None:
                status = 'no suitable dark square found'
                continue
            _, contour, (u, v), (x, y, w, h) = detection
            object_mask = np.zeros(depth.shape, np.uint8)
            cv2.drawContours(object_mask, [contour], -1, 255, -1)
            object_mask = cv2.erode(object_mask, np.ones((5, 5), np.uint8))
            valid = depth[(object_mask > 0) & np.isfinite(depth) & (depth > 0.10) & (depth < 2.0)]
            if valid.size < 20:
                status = f'not enough valid object depth pixels ({valid.size})'
                continue
            z = float(np.median(valid))
            k = np.asarray(info.k, dtype=float).reshape(3, 3)
            point_camera = np.array([(u-k[0, 2])*z/k[0, 0],
                                     (v-k[1, 2])*z/k[1, 1], z, 1.0])
            try:
                wrist = buffer.lookup_transform(
                    'fr3_link0', 'fr3_link8', Time.from_msg(color_msg.header.stamp))
            except TransformException as exc:
                status = 'waiting for wrist TF: ' + str(exc)
                continue
            point_base = tf_matrix(wrist) @ link8_to_camera @ point_camera
            spread_mm = float(np.median(np.abs(valid-z)) * 1000.0)
            annotated = frame.copy()
            cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(annotated, (round(u), round(v)), 5, (0, 255, 255), -1)
            label = f'base [{point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}] m'
            cv2.putText(annotated, label, (max(5, x), max(25, y-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            args.save.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.save), annotated)
            result = {
                'pixel_uv': [float(u), float(v)],
                'box_xywh': [x, y, w, h],
                'valid_depth_pixels': int(valid.size),
                'depth_median_m': z,
                'depth_mad_mm': spread_mm,
                'point_camera_color_optical_m': point_camera[:3].tolist(),
                'point_fr3_link0_m': point_base[:3].tolist(),
                'debug_image': str(args.save),
                'note': 'Detection only; no robot command was sent.',
            }
            print(json.dumps(result, indent=2))
            return
        raise RuntimeError('detection timed out: ' + status)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
