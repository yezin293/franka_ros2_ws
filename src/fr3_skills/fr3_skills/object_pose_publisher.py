"""Continuously detect the cube (YOLOv8, fine-tuned on scripts/yolo_roboflow_dataset)
and publish its robot-frame pose, live, until Ctrl+C -- a normal always-on
perception node, not a one-shot snapshot.

  ros2 run fr3_skills object_pose_publisher
  ros2 run fr3_skills object_pose_publisher --save /tmp/check.jpg   # overwrite each frame

Also publishes an annotated debug view (detection box/centroid, no cv_bridge
needed) to <topic>/debug_image -- watch it with:
  ros2 run rqt_image_view rqt_image_view /object_pose/debug_image

All tunables (image topic, rate, surface_z, calibration file paths) come
from config/vision_pick_place.yaml -- edit that file directly (with
--symlink-install, no rebuild needed) instead of passing flags.

Eye-in-hand camera (D435I on the wrist, see 2026-11-23 handoff section):
unlike the old fixed OBSBOT camera, this one moves with the arm, so its
pose in robot base frame is NOT constant. Needs scripts/eih_camera_intrinsics.json
(the camera's own factory calibration) and scripts/eih_extrinsics.json (the
FIXED flange->camera offset from scripts/calibrate_eye_in_hand.py). Every
frame, the current flange pose is looked up over tf2 (fr3_link0 ->
fr3_hand_tcp) and combined with that fixed offset to get the camera's
current pose in robot base frame.

Pixel -> robot-frame xyz is done with the D435I's own depth reading at the
detected pixel (back-projected via the intrinsics, then rotated into robot
frame via that frame's extrinsics) -- not a planar assumption. The old
"intersect the ray with a known flat surface" approach (z = surface_z) is
kept only as a fallback for frames where the depth pixel is invalid (e.g.
glare, out of range), since depth pretending the object sits exactly on the
tray floor silently mispredicts xy (the detected pixel is on the object's
*top* face, not the floor) in proportion to how oblique the view is --
fine for a near-overhead view, bad for a far/angled one.
"""
import argparse
import json
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from fr3_skills.motion import BASE_FRAME, TCP_LINK

YOLO_CONF_THRESHOLD = 0.8 #confidence 값 조절
OUTLIER_RESET_STREAK = 3       # consecutive gated-out detections before re-anchoring to them
NO_DETECTION_RESET_STREAK = 3  # consecutive empty frames before dropping the smoothed estimate
CONFIRM_SAMPLES = 2            # consecutive agreeing readings required after a (re-)anchor
                                # before publishing -- a single fresh anchor is unvalidated
                                # (nothing to compare it against yet), so a noisy-but-not-wild
                                # first depth sample would otherwise be trusted immediately and
                                # bias the EMA for a while

CONFIG_PATH = os.path.join(get_package_share_directory('fr3_skills'),
                            'config', 'vision_pick_place.yaml')


def load_config():
    with open(CONFIG_PATH) as f:
        return yaml.safe_load(f)


def load_calibration(intrinsics_path, extrinsics_path):
    """Returns (camera_matrix, dist_coeffs, R_flange_cam, t_flange_cam) --
    the last two are the FIXED flange->camera offset (a point in camera
    frame, transformed by R_flange_cam/t_flange_cam, lands in flange frame)."""
    with open(intrinsics_path) as f:
        intr = json.load(f)
    camera_matrix = np.array(intr['camera_matrix'])
    dist_coeffs = np.array(intr['dist_coeffs'])
    with open(extrinsics_path) as f:
        extr = json.load(f)
    t_flange_cam = np.array(extr['flange_to_camera_xyz'])
    R_flange_cam = _quat_to_R(*extr['flange_to_camera_quat_xyzw'])
    return camera_matrix, dist_coeffs, R_flange_cam, t_flange_cam


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def current_camera_extrinsics(flange_xyz, flange_quat_xyzw, R_flange_cam, t_flange_cam):
    """Combine the live flange pose (base->flange) with the fixed
    flange->camera offset to get this instant's base->camera extrinsics
    (rvec, tvec in the same convention cv2.solvePnP would give: a
    robot-frame point transforms into camera frame via P_cam = R@P_base+t)."""
    R_base_flange = _quat_to_R(*flange_quat_xyzw)
    t_base_flange = np.array(flange_xyz)
    # cam -> base (point in cam frame, expressed in base frame)
    R_cam_base = R_base_flange @ R_flange_cam
    t_cam_base = R_base_flange @ t_flange_cam + t_base_flange
    # invert -> base -> cam
    R_base_cam = R_cam_base.T
    t_base_cam = -R_base_cam @ t_cam_base
    rvec, _ = cv2.Rodrigues(R_base_cam)
    return rvec, t_base_cam


def pixel_to_robot_xy(px, py, camera_matrix, dist_coeffs, rvec, tvec, surface_z):
    """Back-project a pixel to a ray and intersect it with the z=surface_z
    plane in robot base frame. rvec/tvec are this instant's robot-frame-to-
    camera-frame extrinsics."""
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


def sample_depth(depth_frame, px, py, scale, patch=5):
    """Median depth (meters) in a small patch around (px, py), ignoring
    invalid (0) pixels. Returns None if nothing valid is in the patch."""
    h, w = depth_frame.shape
    half = patch // 2
    x0, x1 = max(0, int(px) - half), min(w, int(px) + half + 1)
    y0, y1 = max(0, int(py) - half), min(h, int(py) + half + 1)
    region = depth_frame[y0:y1, x0:x1]
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * scale


def pixel_depth_to_robot_xyz(px, py, depth_m, camera_matrix, dist_coeffs, rvec, tvec):
    """Back-project a pixel using its own measured depth (no surface-plane
    assumption) to get the robot-frame 3D point directly."""
    undistorted = cv2.undistortPoints(
        np.array([[[px, py]]], dtype=np.float64), camera_matrix, dist_coeffs)
    ray_cam = np.array([undistorted[0, 0, 0], undistorted[0, 0, 1], 1.0])
    point_cam = ray_cam * depth_m

    R, _ = cv2.Rodrigues(rvec)
    camera_pos = (-R.T @ tvec).flatten()
    return camera_pos + R.T @ point_cam


def detect(frame, model):
    """Return ((px, py), (bx, by, bw, bh)) for the highest-confidence YOLO
    detection, both in full-frame pixel coordinates, or (None, None) if
    nothing passed YOLO_CONF_THRESHOLD."""
    result = model.predict(frame, conf=YOLO_CONF_THRESHOLD, verbose=False)[0]
    if len(result.boxes) == 0:
        return None, None
    best = result.boxes[result.boxes.conf.argmax()]
    x0, y0, x1, y1 = best.xyxy[0].tolist()
    px, py = (x0 + x1) / 2, (y0 + y1) / 2
    return (px, py), (int(x0), int(y0), int(x1 - x0), int(y1 - y0))


def annotate(frame, centroid, box):
    """Draw the detected object's box/centroid, if found."""
    out = frame.copy()
    if box is not None:
        x, y, w, h = box
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 4)
        cv2.circle(out, (int(centroid[0]), int(centroid[1])), 10, (0, 255, 255), -1)
    else:
        cv2.putText(out, 'no object', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', default=None,
                         help='overwrite this path with an annotated frame each cycle')
    args = parser.parse_args()
    cfg = load_config()

    camera_matrix, dist_coeffs, R_flange_cam, t_flange_cam = load_calibration(
        cfg['intrinsics'], cfg['extrinsics'])
    from ultralytics import YOLO
    model = YOLO(cfg['yolo_weights'])

    rclpy.init()
    node = Node('object_pose_publisher')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    pub = node.create_publisher(PoseStamped, cfg['topic'], 10)
    debug_topic = cfg['topic'] + '/debug_image'
    image_pub = node.create_publisher(Image, debug_topic, 1)

    latest = {}

    def image_cb(msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        latest['frame'] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        latest['stamp'] = msg.header.stamp

    def depth_cb(msg):
        latest['depth'] = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    node.create_subscription(Image, cfg['image_topic'], image_cb, 1)
    node.create_subscription(Image, cfg['depth_topic'], depth_cb, 1)

    state = {'x': None, 'y': None, 'z': None, 'miss_streak': 0, 'outlier_streak': 0,
              'confirm_count': 0}

    def reset_cb(_msg):
        # Requested by vision_pick_place.py at the start of each cycle --
        # without this, the smoothed estimate carries over from whatever the
        # camera saw during the PREVIOUS cycle's own motion (gripper, tray
        # walls, background, ...), and a genuinely new object in the pick
        # tray gets rejected as an "outlier" against that stale, unrelated
        # anchor instead of being tracked fresh.
        state['x'] = state['y'] = state['z'] = None
        state['miss_streak'] = 0
        state['outlier_streak'] = 0
        state['confirm_count'] = 0
        print('    tracking reset (requested by vision_pick_place)')

    reset_topic = cfg['topic'] + '/reset'
    node.create_subscription(Empty, reset_topic, reset_cb, 1)

    period = 1.0 / cfg['publish_rate_hz']
    print(f"publishing live detections on {cfg['topic']} at "
          f"{cfg['publish_rate_hz']} Hz -- Ctrl+C to stop")
    print(f'debug view: ros2 run rqt_image_view rqt_image_view {debug_topic}')
    print(f'tracking reset topic: {reset_topic} (std_msgs/Empty)')
    print(f"waiting for images on {cfg['image_topic']} and {cfg['depth_topic']} ...")
    img_msg = Image()
    img_msg.encoding = 'bgr8'
    img_msg.is_bigendian = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            if 'frame' not in latest or 'depth' not in latest:
                continue
            frame = latest['frame']
            depth_frame = latest['depth']

            try:
                # Look up the flange pose AT THE FRAME'S OWN TIMESTAMP, not
                # "latest" -- if the arm is still moving (e.g. right as it
                # arrives home, before fully settling), "latest" would give a
                # newer, different pose than when this photo was actually
                # taken, producing a geometrically wrong extrinsics and wild
                # 3D errors. Matching the frame's own stamp removes that race
                # regardless of how long inference takes or how fast the arm
                # is moving.
                tf = tf_buffer.lookup_transform(BASE_FRAME, TCP_LINK,
                                                 rclpy.time.Time.from_msg(latest['stamp']))
            except TransformException as e:
                print(f'no TF {BASE_FRAME}->{TCP_LINK} at frame time yet: {e}')
                continue
            flange_xyz = (tf.transform.translation.x, tf.transform.translation.y,
                          tf.transform.translation.z)
            flange_quat = (tf.transform.rotation.x, tf.transform.rotation.y,
                           tf.transform.rotation.z, tf.transform.rotation.w)
            rvec, tvec = current_camera_extrinsics(flange_xyz, flange_quat,
                                                     R_flange_cam, t_flange_cam)

            centroid, box = detect(frame, model)
            if centroid is None:
                print('no object detected')
                state['miss_streak'] += 1
                if state['miss_streak'] >= NO_DETECTION_RESET_STREAK and state['x'] is not None:
                    print('    no detections for a while -- resetting smoothed estimate')
                    state['x'] = state['y'] = state['z'] = None
                    state['outlier_streak'] = 0
            else:
                px, py = centroid
                depth_m = sample_depth(depth_frame, px, py, cfg['depth_scale'])
                if depth_m is not None:
                    x, y, z = pixel_depth_to_robot_xyz(px, py, depth_m, camera_matrix,
                                                         dist_coeffs, rvec, tvec)
                    source = 'depth'
                else:
                    x, y = pixel_to_robot_xy(px, py, camera_matrix, dist_coeffs,
                                              rvec, tvec, cfg['surface_z'])
                    z = cfg['surface_z']
                    source = 'fallback(no depth)'

                if not (cfg['z_sane_min'] <= z <= cfg['z_sane_max']):
                    # A bad depth sample (glare, reflection, a mixed edge pixel) can
                    # put z wildly off -- check this BEFORE touching state, since
                    # right after a reset there's no existing anchor for the
                    # outlier gate below to compare against, so a garbage first
                    # reading would otherwise be trusted unconditionally.
                    print(f'    rejecting frame -- z={z:+.3f} outside sane range '
                          f"[{cfg['z_sane_min']:+.3f}, {cfg['z_sane_max']:+.3f}] [{source}]")
                    state['miss_streak'] += 1
                    if state['miss_streak'] >= NO_DETECTION_RESET_STREAK and state['x'] is not None:
                        print('    no good detections for a while -- resetting smoothed estimate')
                        state['x'] = state['y'] = state['z'] = None
                        state['outlier_streak'] = 0
                else:
                    state['miss_streak'] = 0
                    publish_ok = False
                    if state['x'] is None:
                        state['x'], state['y'], state['z'] = x, y, z
                        state['outlier_streak'] = 0
                        state['confirm_count'] = 0
                    else:
                        jump = ((x - state['x']) ** 2 + (y - state['y']) ** 2) ** 0.5
                        if jump > cfg['outlier_gate_m']:
                            state['outlier_streak'] += 1
                            print(f'    ignoring outlier jump={jump:.3f}m [{source}] '
                                  f'(streak {state["outlier_streak"]}/{OUTLIER_RESET_STREAK})')
                            if state['outlier_streak'] >= OUTLIER_RESET_STREAK:
                                print('    outlier persisted -- re-anchoring to it')
                                state['x'], state['y'], state['z'] = x, y, z
                                state['outlier_streak'] = 0
                                state['confirm_count'] = 0
                        else:
                            state['outlier_streak'] = 0
                            a = cfg['ema_alpha']
                            state['x'] = a * x + (1 - a) * state['x']
                            state['y'] = a * y + (1 - a) * state['y']
                            state['z'] = a * z + (1 - a) * state['z']
                            # A fresh (or re-)anchor is unvalidated -- nothing existed to
                            # compare it against -- so require a few consecutive agreeing
                            # readings before trusting it, instead of publishing immediately
                            # on whatever the first, possibly noisy, sample happened to be.
                            if state['confirm_count'] < CONFIRM_SAMPLES:
                                state['confirm_count'] += 1
                                print(f'    confirming anchor '
                                      f'({state["confirm_count"]}/{CONFIRM_SAMPLES}) ...')
                            publish_ok = state['confirm_count'] >= CONFIRM_SAMPLES

                    if publish_ok:
                        grasp_z = state['z'] + cfg['grasp_clearance_z']
                        msg = PoseStamped()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = BASE_FRAME
                        msg.pose.position.x = state['x']
                        msg.pose.position.y = state['y']
                        msg.pose.position.z = grasp_z
                        msg.pose.orientation.x = 1.0  # pointing straight down
                        msg.pose.orientation.w = 0.0
                        pub.publish(msg)
                        print(f'pixel ({px:.0f}, {py:.0f}) -> robot smoothed '
                              f'(x={state["x"]:+.3f}, y={state["y"]:+.3f}, grasp_z={grasp_z:+.3f}) '
                              f'[{source}]')

            annotated = annotate(frame, centroid, box)
            if args.save:
                cv2.imwrite(args.save, annotated)
            img_msg.header.stamp = node.get_clock().now().to_msg()
            img_msg.height, img_msg.width = annotated.shape[:2]
            img_msg.step = img_msg.width * 3
            img_msg.data = annotated.tobytes()
            image_pub.publish(img_msg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
