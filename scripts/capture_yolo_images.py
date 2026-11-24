"""Live preview + SPACE-to-capture for collecting a YOLO training dataset
(e.g. to upload to Roboflow). Move the cube around the tray (position,
orientation, lighting) between captures for variety.

  source install/setup.bash && /usr/bin/python3 scripts/capture_yolo_images.py

Needs the realsense2_camera node running (/camera/camera/color/image_raw).
Saves scripts/yolo_dataset/NNN.jpg. SPACE = capture, ESC/q = quit.
"""
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT_DIR = 'scripts/yolo_dataset'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    existing = [f for f in os.listdir(OUT_DIR) if f.endswith('.jpg')]
    count = len(existing)

    rclpy.init()
    node = Node('capture_yolo_images')
    latest = {}

    def cb(msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        latest['frame'] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    node.create_subscription(Image, '/camera/camera/color/image_raw', cb, 1)
    print('waiting for camera stream ...')
    while 'frame' not in latest and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'{count} image(s) already saved. SPACE = capture, ESC/q = quit.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if 'frame' not in latest:
                continue
            disp = latest['frame'].copy()
            cv2.putText(disp, f'images: {count}  SPACE=capture  ESC=quit',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow('yolo dataset capture', disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if key == ord(' '):
                path = os.path.join(OUT_DIR, f'{count:03d}.jpg')
                cv2.imwrite(path, latest['frame'])
                print(f'[{count:03d}] saved {path}')
                count += 1
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
