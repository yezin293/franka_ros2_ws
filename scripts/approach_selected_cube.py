#!/usr/bin/env python3
"""Move only to a safe hover pose above a user-selected cube."""
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped

from fr3_skills.motion import BASE_FRAME, Fr3Motion, TCP_LINK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path,
                        default=Path('calibration_data/selected_black_cube.json'))
    parser.add_argument('--hover-height', type=float, default=0.10)
    parser.add_argument('--target-z', type=float,
                        help='Explicit TCP z target; overrides top z + hover height')
    parser.add_argument('--scaling', type=float, default=0.05)
    parser.add_argument('--cartesian-current-orientation', action='store_true',
                        help='Move straight to hover while preserving current TCP orientation')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()

    selected = json.loads(args.selection.read_text())
    if selected.get('safe_to_execute') is not False:
        raise SystemExit('unexpected selection format; refusing to plan')
    x, y, top_z = map(float, selected['visible_top_center_fr3_link0_m'])
    target_z = args.target_z if args.target_z is not None else top_z + args.hover_height
    if not (0.15 <= x <= 0.70 and -0.40 <= y <= 0.40 and 0.03 <= top_z <= 0.50):
        raise SystemExit(
            f'selected point ({x:.3f}, {y:.3f}, {top_z:.3f}) is outside safety bounds')

    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = target_z
    print(f'selected cube top : ({x:+.3f}, {y:+.3f}, {top_z:+.3f}) m')
    print(f'hover TCP target  : ({x:+.3f}, {y:+.3f}, {target_z:+.3f}) m')
    print('gripper action    : none')
    rclpy.init()
    motion = Fr3Motion()
    try:
        motion.ensure_table_collision()
        start = motion.current_joints()
        if args.cartesian_current_orientation:
            current_tcp = motion.current_tcp_pose()
            target.pose.orientation.x = current_tcp[3]
            target.pose.orientation.y = current_tcp[4]
            target.pose.orientation.z = current_tcp[5]
            target.pose.orientation.w = current_tcp[6]
            trajectory = motion.plan_cartesian_path(
                [target], scaling=args.scaling, max_step=0.005, link_name=TCP_LINK)
            final = dict(zip(trajectory.joint_trajectory.joint_names,
                             trajectory.joint_trajectory.points[-1].positions))
            changes = [abs(final[name]-start[name]) for name in start]
            max_change = max(changes)
            print('selected motion: straight Cartesian, current TCP orientation preserved')
        else:
            options = []
            for yaw_deg in range(-180, 180, 45):
                yaw = math.radians(yaw_deg)
                # Rz(yaw) * Rx(pi): gripper remains vertical; only its opening
                # direction changes. Search this free degree of freedom so MoveIt
                # does not choose an unnecessarily distant IK branch.
                target.pose.orientation.x = math.cos(yaw / 2.0)
                target.pose.orientation.y = math.sin(yaw / 2.0)
                target.pose.orientation.z = 0.0
                target.pose.orientation.w = 0.0
                try:
                    candidate = motion.plan_to_pose(
                        target, scaling=args.scaling, tolerance_m=0.005,
                        tolerance_rad=0.02, link_name=TCP_LINK)
                except RuntimeError:
                    continue
                final = dict(zip(candidate.joint_trajectory.joint_names,
                                 candidate.joint_trajectory.points[-1].positions))
                changes = [abs(final[name]-start[name]) for name in start]
                options.append((max(changes), sum(changes), yaw_deg, candidate))
            if not options:
                raise RuntimeError('no collision-free vertical hover plan for any gripper yaw')
            max_change, total_change, yaw_deg, trajectory = min(options)
            print(f'selected gripper yaw: {yaw_deg:+d} deg '
                  f'(max joint change {max_change:.3f} rad, total {total_change:.3f} rad)')
        if max_change > 1.50:
            raise RuntimeError(
                f'safest plan still changes one joint by {max_change:.3f} rad; refusing execution')
        print(motion.describe(trajectory, start))
        if not args.execute:
            print('PLAN ONLY: robot did not move.')
            return
        motion.activate_arm()
        try:
            motion.execute(trajectory)
        finally:
            motion.deactivate_arm()
        pose = motion.current_tcp_pose()
        print('EXECUTED: current TCP = '
              f'({pose[0]:+.3f}, {pose[1]:+.3f}, {pose[2]:+.3f}) m')
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
