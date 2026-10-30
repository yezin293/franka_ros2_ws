"""Continuously detect the largest dark blob and publish its robot-frame
pose, live, until Ctrl+C -- a normal always-on perception node, not a
one-shot snapshot.

  ros2 run fr3_skills object_pose_publisher
  ros2 run fr3_skills object_pose_publisher --save /tmp/check.jpg   # overwrite each frame

Also publishes an annotated debug view (detection box/centroid, no cv_bridge
needed) to <topic>/debug_image -- watch it with:
  ros2 run rqt_image_view rqt_image_view /object_pose/debug_image

All tunables (device, resolution, topic, rate, surface_z, calibration file
paths) come from config/vision_pick_place.yaml -- edit that file directly
(with --symlink-install, no rebuild needed) instead of passing flags.

Needs scripts/camera_intrinsics.json (see scripts/calibrate_intrinsics_from_folder.py)
and scripts/camera_extrinsics.json (see scripts/calibrate_extrinsics.py).

Pixel -> robot-frame xy is done by back-projecting the detected pixel into a
3D ray from the camera (using the intrinsics to undistort/normalize, and the
extrinsics -- the camera's pose in robot base frame from the checkerboard/
ArUco + solvePnP calibration -- to express that ray in robot frame) and
intersecting it with the known tray-surface plane (z = surface_z, measured
by touching the tray floor with the gripper -- see
scripts/touch_points.json's 'tray_surface').
"""
import argparse
import json
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from fr3_skills.motion import BASE_FRAME

# Dark-object threshold for the black cylinders/cube on the grey tray
# (temporary -- swap for YOLO or another detector once picking by color
# isn't enough).
DARK_THRESHOLD = 70
MIN_AREA = 1000
# Pixel region containing the pick tray, at the 3840x2160 calibration
# resolution -- excludes the cluttered background (chairs, clothing) above
# the trays, which is otherwise darker/bigger than the object and wins the
# "largest blob" pick. Re-measure if the camera is reframed.
ROI = (1050, 1450, 1750, 2160)  # x0, y0, x1, y1

CONFIG_PATH = os.path.join(get_package_share_directory('fr3_skills'),
                            'config', 'vision_pick_place.yaml')


def load_config():
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def load_calibration(intrinsics_path, extrinsics_path):
    with open(intrinsics_path) as f:
        intr = json.load(f)
    camera_matrix = np.array(intr['camera_matrix'])
    dist_coeffs = np.array(intr['dist_coeffs'])
    with open(extrinsics_path) as f:
        extr = json.load(f)
    rvec = np.array(extr['rvec'])
    tvec = np.array(extr['tvec'])
    return camera_matrix, dist_coeffs, rvec, tvec


def pixel_to_robot_xy(px, py, camera_matrix, dist_coeffs, rvec, tvec, surface_z):
    """Back-project a pixel to a ray and intersect it with the z=surface_z
    plane in robot base frame. rvec/tvec are the robot-frame-to-camera-frame
    extrinsics from cv2.solvePnP (object_points were given in robot frame)."""
    undistorted = cv2.undistortPoints(
        np.array([[[px, py]]], dtype=np.float64), camera_matrix, dist_coeffs)
    ray_cam = np.array([undistorted[0, 0, 0], undistorted[0, 0, 1], 1.0])

    R, _ = cv2.Rodrigues(rvec)
    camera_pos = (-R.T @ tvec).flatten()
    ray_robot = R.T @ ray_cam  # direction only, no translation

    if abs(ray_robot[2]) < 1e-9:
        raise ValueError('ray is parallel to the tray plane -- check extrinsics')
    s = (surface_z - camera_pos[2]) / ray_robot[2]
    if s <= 0:
        raise ValueError('tray plane is behind the camera along this ray -- '
                          'check surface_z/extrinsics')
    point = camera_pos + s * ray_robot
    return point[0], point[1]


def detect(frame):
    """Return ((px, py), (bx, by, bw, bh)) for the largest dark blob in ROI,
    both in full-frame pixel coordinates, or (None, None) if nothing found."""
    x0, y0, x1, y1 = ROI
    roi = frame[y0:y1, x0:x1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, DARK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]
    if not contours:
        return None, None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None, None
    px, py = M['m10'] / M['m00'] + x0, M['m01'] / M['m00'] + y0
    bx, by, bw, bh = cv2.boundingRect(largest)
    return (px, py), (bx + x0, by + y0, bw, bh)


def annotate(frame, centroid, box):
    """Draw the ROI and, if found, the detected object's box/centroid."""
    out = frame.copy()
    cv2.rectangle(out, ROI[:2], ROI[2:], (255, 0, 0), 3)
    if box is not None:
        x, y, w, h = box
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 4)
        cv2.circle(out, (int(centroid[0]), int(centroid[1])), 10, (0, 255, 255), -1)
    else:
        cv2.putText(out, 'no object', (ROI[0], ROI[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
    return out


def open_camera(device, width, height):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise SystemExit(f'could not open camera /dev/video{device}')
    # VideoCapture defaults to the YUYV format, which this camera only
    # supports up to 640x480 -- silently breaking the calibration (done
    # against the camera's native resolution). 3840x2160 only exists under
    # MJPG (v4l2-ctl --list-formats-ext).
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    if (actual_w, actual_h) != (width, height):
        print(f'warning: camera gave {actual_w:.0f}x{actual_h:.0f}, '
              f'not the requested {width}x{height} -- '
              'pixel->robot mapping will be wrong unless intrinsics/extrinsics '
              'were calibrated at this size')
    # The very first frame after opening at this format/resolution comes back
    # black, and auto-exposure/auto-focus can still be settling for a few
    # more frames after that -- discard some before trusting any frame.
    time.sleep(1.0)
    for _ in range(5):
        cap.read()
    return cap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', default=None,
                         help='overwrite this path with an annotated frame each cycle')
    args = parser.parse_args()
    cfg = load_config()

    camera_matrix, dist_coeffs, rvec, tvec = load_calibration(
        cfg['intrinsics'], cfg['extrinsics'])
    cap = open_camera(cfg['device'], cfg['cam_width'], cfg['cam_height'])

    rclpy.init()
    node = Node('object_pose_publisher')
    pub = node.create_publisher(PoseStamped, cfg['topic'], 10)
    debug_topic = cfg['topic'] + '/debug_image'
    image_pub = node.create_publisher(Image, debug_topic, 1)
    period = 1.0 / cfg['publish_rate_hz']
    print(f"publishing live detections on {cfg['topic']} at "
          f"{cfg['publish_rate_hz']} Hz -- Ctrl+C to stop")
    print(f'debug view: ros2 run rqt_image_view rqt_image_view {debug_topic}')
    img_msg = Image()
    img_msg.encoding = 'bgr8'
    img_msg.is_bigendian = 0
    try:
        while rclpy.ok():
            ok, frame = cap.read()
            if ok:
                centroid, box = detect(frame)
                if centroid is None:
                    print('no object detected')
                else:
                    px, py = centroid
                    x, y = pixel_to_robot_xy(px, py, camera_matrix, dist_coeffs,
                                              rvec, tvec, cfg['surface_z'])
                    msg = PoseStamped()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = BASE_FRAME
                    msg.pose.position.x = float(x)
                    msg.pose.position.y = float(y)
                    msg.pose.position.z = cfg['pick_z']
                    msg.pose.orientation.x = 1.0  # pointing straight down
                    msg.pose.orientation.w = 0.0
                    pub.publish(msg)
                    print(f'pixel ({px:.0f}, {py:.0f}) -> robot '
                          f'(x={x:+.3f}, y={y:+.3f})')

                annotated = annotate(frame, centroid, box)
                small = cv2.resize(annotated, (1280, 720))
                if args.save:
                    cv2.imwrite(args.save, small)
                img_msg.header.stamp = node.get_clock().now().to_msg()
                img_msg.height, img_msg.width = small.shape[:2]
                img_msg.step = img_msg.width * 3
                img_msg.data = small.tobytes()
                image_pub.publish(img_msg)
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
