"""Live preview + SPACE-to-capture for eye-in-hand hand-eye calibration
samples. Move the arm to a new pose (checkerboard fully visible, vary
orientation each time), press SPACE to save a (photo, flange pose) sample,
repeat ~10-15 times, ESC/q to quit.

  source install/setup.bash && /usr/bin/python3 scripts/eih_capture_live.py

Needs the realsense2_camera node running (/camera/camera/color/image_raw)
and the robot bringup/TF running. Saves scripts/eih_samples/NN.jpg and
appends the flange pose to scripts/eih_samples/poses.json.
"""
import json
import os

import cv2
import numpy as np
import rclpy
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

    rclpy.init()
    motion = Fr3Motion()
    latest = {}

    def cb(msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        latest['frame'] = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    motion.create_subscription(Image, '/camera/camera/color/image_raw', cb, 1)
    print('waiting for camera stream ...')
    while 'frame' not in latest and rclpy.ok():
        rclpy.spin_once(motion, timeout_sec=0.1)

    print(f'{len(poses)} sample(s) already saved. SPACE = capture, ESC/q = quit.')
    try:
        while rclpy.ok():
            rclpy.spin_once(motion, timeout_sec=0.01)
            if 'frame' not in latest:
                continue
            disp = latest['frame'].copy()
            cv2.putText(disp, f'samples: {len(poses)}  SPACE=capture  ESC=quit',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.imshow('eih calibration capture', disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if key == ord(' '):
                idx = len(poses)
                pose = motion.current_tcp_pose()
                photo_path = os.path.join(OUT_DIR, f'{idx:02d}.jpg')
                cv2.imwrite(photo_path, latest['frame'])
                poses[f'{idx:02d}'] = {'xyz': list(pose[:3]), 'quat_xyzw': list(pose[3:])}
                with open(POSES_PATH, 'w') as f:
                    json.dump(poses, f, indent=2)
                print(f'[{idx:02d}] saved {photo_path}  xyz={[f"{v:+.3f}" for v in pose[:3]]}')
    finally:
        cv2.destroyAllWindows()
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
