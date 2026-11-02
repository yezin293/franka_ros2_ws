#!/usr/bin/env python3
"""Move the current TCP straight upward without changing XY or orientation."""
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped

from fr3_skills.motion import BASE_FRAME, Fr3Motion, TCP_LINK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=0.15)
    parser.add_argument('--scaling', type=float, default=0.05)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not 0.01 <= args.distance <= 0.25:
        raise SystemExit('distance must be between 0.01 and 0.25 m')

    rclpy.init()
    motion = Fr3Motion()
    try:
        motion.ensure_table_collision()
        current = motion.current_tcp_pose()
        target = PoseStamped()
        target.header.frame_id = BASE_FRAME
        target.pose.position.x = current[0]
        target.pose.position.y = current[1]
        target.pose.position.z = current[2] + args.distance
        target.pose.orientation.x = current[3]
        target.pose.orientation.y = current[4]
        target.pose.orientation.z = current[5]
        target.pose.orientation.w = current[6]
        print(f'current TCP: ({current[0]:+.3f}, {current[1]:+.3f}, {current[2]:+.3f}) m')
        print(f'target TCP : ({current[0]:+.3f}, {current[1]:+.3f}, '
              f'{current[2] + args.distance:+.3f}) m')
        start = motion.current_joints()
        trajectory = motion.plan_cartesian_path(
            [target], scaling=args.scaling, max_step=0.005, link_name=TCP_LINK)
        print(motion.describe(trajectory, start))
        if not args.execute:
            print('PLAN ONLY: robot did not move.')
            return
        motion.activate_arm()
        try:
            motion.execute(trajectory)
        finally:
            motion.deactivate_arm()
        final = motion.current_tcp_pose()
        print(f'EXECUTED: current TCP = ({final[0]:+.3f}, {final[1]:+.3f}, {final[2]:+.3f}) m')
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
