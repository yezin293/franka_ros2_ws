"""Vision-driven pick-and-place: subscribe to the live pose from
object_pose_publisher (run that in another terminal first), then
approach -> pick -> move to place -> place -> home. Prints a numbered
state at each stage.

  ros2 run fr3_skills object_pose_publisher          # terminal 1, leave running
  ros2 run fr3_skills vision_pick_place               # terminal 2, plan-only
  ros2 run fr3_skills vision_pick_place --execute      # actually move

All tunables (place position, object width, grasp force, etc.) come from
config/vision_pick_place.yaml -- edit that file directly (with
--symlink-install, no rebuild needed) instead of passing flags.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped

from fr3_skills.motion import Fr3Motion
from fr3_skills.move_home import move_home
from fr3_skills.object_pose_publisher import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true',
                         help='actually move (default: plan-only, check RViz)')
    parser.add_argument('--timeout', type=float, default=10.0,
                         help='seconds to wait for a live detection')
    args = parser.parse_args()
    dry_run = not args.execute
    cfg = load_config()

    rclpy.init()
    motion = Fr3Motion()
    try:
        print(f"[1] waiting for a live detection on {cfg['topic']} "
              '(is object_pose_publisher running?) ...')
        pending = []
        motion.create_subscription(PoseStamped, cfg['topic'], pending.append, 10)
        deadline = time.time() + args.timeout
        while not pending and time.time() < deadline:
            rclpy.spin_once(motion, timeout_sec=0.1)
        if not pending:
            raise SystemExit(f"no pose received on {cfg['topic']} within "
                              f'{args.timeout}s -- is object_pose_publisher running '
                              'and seeing the object?')
        pick_pose = pending[-1]  # most recent
        pick_pose.pose.position.z = cfg['pick_z']
        x, y = pick_pose.pose.position.x, pick_pose.pose.position.y
        print(f'    got robot (x={x:+.3f}, y={y:+.3f})')

        place_pose = PoseStamped()
        place_pose.header.frame_id = pick_pose.header.frame_id
        place_pose.pose.position.x, place_pose.pose.position.y, \
            place_pose.pose.position.z = cfg['place']
        place_pose.pose.orientation.x = 1.0
        place_pose.pose.orientation.w = 0.0

        motion.ensure_table_collision()

        print('[2] approach + pick ...')
        motion.pick(pick_pose, approach_height=cfg['approach_height'], width=cfg['width'],
                    force=cfg['force'], scaling=cfg['scaling'], dry_run=dry_run,
                    epsilon=cfg['epsilon'])
        print('    picked.' if not dry_run else '    pick: plan only (not executed)')

        print('[3] move to place + place ...')
        motion.place(place_pose, approach_height=cfg['approach_height'],
                     scaling=cfg['scaling'], dry_run=dry_run)
        print('    placed.' if not dry_run else '    place: plan only (not executed)')

        print('[4] returning home ...')
        move_home(motion, cfg['scaling'], execute=args.execute)
        print('    done.' if not dry_run else '    home: plan only (not executed)')
    except (RuntimeError, ValueError) as e:
        print('stopped:', e)
        sys.exit(1)
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
