"""Record the robot's current TCP position as a named calibration point.

Jog the robot (RViz drag + Plan/Execute, or pose_listener) so the gripper
fingertip touches a specific physical point -- e.g. one corner of the ArUco
marker -- then run this to save that point's robot-frame (x, y, z) under a
name. Run once per point; points accumulate in scripts/touch_points.json.

  /usr/bin/python3 scripts/record_touch_point.py --name corner_top_left
  /usr/bin/python3 scripts/record_touch_point.py --name corner_top_right
  /usr/bin/python3 scripts/record_touch_point.py --name corner_bottom_right
  /usr/bin/python3 scripts/record_touch_point.py --name corner_bottom_left

Naming convention for the marker corners matters -- it must match the pixel
corner order from cv2.aruco.detectMarkers, which returns each marker's
corners as (top-left, top-right, bottom-right, bottom-left) in the marker's
own orientation as printed (reading the marker image top-to-bottom).
"""
import argparse
import json
import os

import rclpy

from fr3_skills.motion import Fr3Motion

OUT_PATH = 'scripts/touch_points.json'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    args = parser.parse_args()

    rclpy.init()
    motion = Fr3Motion()
    pose = motion.current_tcp_pose()
    motion.destroy_node()
    rclpy.try_shutdown()

    points = {}
    if os.path.exists(OUT_PATH):
        with open(OUT_PATH) as f:
            points = json.load(f)
    points[args.name] = {'xyz': list(pose[:3]), 'quat_xyzw': list(pose[3:])}
    with open(OUT_PATH, 'w') as f:
        json.dump(points, f, indent=2)

    x, y, z = pose[:3]
    print(f"recorded '{args.name}': x={x:+.4f} y={y:+.4f} z={z:+.4f}")
    print(f'{len(points)} point(s) saved so far in {OUT_PATH}: {list(points)}')


if __name__ == '__main__':
    main()
