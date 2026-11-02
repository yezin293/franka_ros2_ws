#!/usr/bin/env python3
"""Click geometric centres of the left and right trays (no robot motion)."""
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from detect_black_cube_rgbd import depth_metres, image_bgr, tf_matrix


CALIBRATION = Path('scripts/handeye_d435i.json')
OUTPUT = Path('calibration_data/tray_centers.json')


def main():
    handeye = json.loads(CALIBRATION.read_text())
    link8_to_camera = np.asarray(
        handeye['link8_T_camera_color_optical']['matrix'], dtype=float)
    rclpy.init()
    node = rclpy.create_node('calibrate_tray_centers')
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
    window = 'Click LEFT tray centre, then RIGHT tray centre (Q: quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    clicks = []

    def mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 2:
            clicks.append((x, y))

    cv2.setMouseCallback(window, mouse)
    snapshot = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if snapshot is None and colors and depths and infos:
                color_msg, depth_msg, info = colors[-1], depths[-1], infos[-1]
                stamp = Time.from_msg(color_msg.header.stamp)
                if abs(stamp.nanoseconds-Time.from_msg(depth_msg.header.stamp).nanoseconds) > 50_000_000:
                    continue
                try:
                    wrist = tf_buffer.lookup_transform('fr3_link0', 'fr3_link8', stamp)
                except TransformException:
                    continue
                snapshot = (image_bgr(color_msg), depth_metres(depth_msg),
                            np.asarray(info.k).reshape(3, 3),
                            tf_matrix(wrist) @ link8_to_camera)
            if snapshot is None:
                continue
            frame, depth, camera_matrix, base_to_camera = snapshot
            view = frame.copy()
            instruction = ('CLICK LEFT TRAY CENTRE' if not clicks else
                           'CLICK RIGHT TRAY CENTRE' if len(clicks) == 1 else
                           'SAVING...')
            cv2.rectangle(view, (0, 0), (view.shape[1], 35), (0, 0, 0), -1)
            cv2.putText(view, instruction, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 2, cv2.LINE_AA)
            for index, point in enumerate(clicks):
                cv2.drawMarker(view, point, (0, 0, 255), cv2.MARKER_CROSS, 28, 3)
                cv2.putText(view, ('LEFT', 'RIGHT')[index],
                            (point[0]+12, point[1]-12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 255), 2)
            cv2.imshow(window, view)
            if cv2.waitKey(20) & 0xff in (ord('q'), 27):
                return
            if len(clicks) < 2:
                continue
            points = []
            for u, v in clicks:
                patch = depth[max(0, v-5):v+6, max(0, u-5):u+6]
                values = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 2.0)]
                if values.size < 20:
                    raise RuntimeError(f'not enough valid depth near click ({u}, {v})')
                z = float(np.median(values))
                camera_point = np.array([
                    (u-camera_matrix[0, 2])*z/camera_matrix[0, 0],
                    (v-camera_matrix[1, 2])*z/camera_matrix[1, 1], z, 1.0])
                base_point = base_to_camera @ camera_point
                points.append([float(base_point[0]), float(base_point[1])])
            data = {'left_image_tray_xy_m': points[0],
                    'right_image_tray_xy_m': points[1]}
            # Runtime selection uses base-Y sign, independent of image layout.
            for point in points:
                data['positive_y_xy_m' if point[1] >= 0 else 'negative_y_xy_m'] = point
            if 'positive_y_xy_m' not in data or 'negative_y_xy_m' not in data:
                raise RuntimeError('both clicks transformed to the same base-Y side')
            OUTPUT.parent.mkdir(parents=True, exist_ok=True)
            OUTPUT.write_text(json.dumps(data, indent=2)+'\n')
            print('SAVED:', OUTPUT)
            print(json.dumps(data, indent=2))
            return
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
