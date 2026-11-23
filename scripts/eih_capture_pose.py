"""Capture one (photo, flange pose) sample for eye-in-hand hand-eye
calibration. Run once per arm pose (checkerboard must stay fully visible,
vary orientation each time -- see calibrate_eye_in_hand.py for the next step).

  source install/setup.bash && /usr/bin/python3 scripts/eih_capture_pose.py

Needs the realsense2_camera node running (/camera/camera/color/image_raw)
and the robot bringup/TF running. Saves scripts/eih_samples/NN.jpg and
appends the flange pose to scripts/eih_samples/poses.json.
"""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from fr3_skills.motion import Fr3Motion

OUT_DIR = 'scripts/eih_samples'
POSES_PATH = os.path.join(OUT_DIR, 'poses.json')


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    poses = {}
    if os.path.exists(POSES_PATH):
        with open(POSES_PATH) as f:
            poses = json.load(f)
    idx = len(poses)

    rclpy.init()
    motion = Fr3Motion()

    frame_holder = {}

    def cb(msg):
        if 'frame' in frame_holder:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame_holder['frame'] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    motion.create_subscription(Image, '/camera/camera/color/image_raw', cb, 1)
    deadline = time.time() + 5.0
    while 'frame' not in frame_holder and time.time() < deadline:
        rclpy.spin_once(motion, timeout_sec=0.1)
    if 'frame' not in frame_holder:
        raise SystemExit('no image received on /camera/camera/color/image_raw -- '
                          'is the realsense2_camera node running?')

    pose = motion.current_tcp_pose()

    photo_path = os.path.join(OUT_DIR, f'{idx:02d}.jpg')
    cv2.imwrite(photo_path, frame_holder['frame'])
    poses[f'{idx:02d}'] = {'xyz': list(pose[:3]), 'quat_xyzw': list(pose[3:])}
    with open(POSES_PATH, 'w') as f:
        json.dump(poses, f, indent=2)

    print(f'saved {photo_path}')
    print(f"flange pose: xyz={pose[:3]} quat_xyzw={pose[3:]}")
    print(f'{len(poses)} sample(s) total so far')

    motion.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
