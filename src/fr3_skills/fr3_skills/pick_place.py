"""Pick at one pose and place at another.

Positions are x y z [m] in the fr3_link0 frame, target is the real fingertip
contact point (fr3_hand_tcp) -- needs bringup/MoveIt with load_gripper:=true.

  ros2 run fr3_skills pick_place --pick 0.4 0.0 0.02 --place 0.4 0.2 0.02
  ros2 run fr3_skills pick_place --pick ... --place ... --execute   # actually move
"""
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped

from fr3_skills.motion import BASE_FRAME, Fr3Motion


def make_pose(xyz):
    p = PoseStamped()
    p.header.frame_id = BASE_FRAME
    p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
    # Gripper pointing straight down (roll 180 deg from identity) -- identity
    # itself points the TCP's z-axis up, which is essentially never reachable
    # for a real pick/place target close to the table.
    p.pose.orientation.x = 1.0
    p.pose.orientation.w = 0.0
    return p


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pick', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--place', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--width', type=float, default=0.0, help='grasp target width [m]')
    parser.add_argument('--force', type=float, default=20.0, help='grasp force [N]')
    parser.add_argument('--epsilon', type=float, default=0.005,
                         help='grasp width tolerance [m] -- widen if --width is a rough estimate')
    parser.add_argument('--approach-height', type=float, default=0.1, help='retreat height [m]')
    parser.add_argument('--scaling', type=float, default=0.1)
    parser.add_argument('--execute', action='store_true',
                         help='actually move (default: plan-only, check RViz)')
    args = parser.parse_args()
    dry_run = not args.execute

    rclpy.init()
    motion = Fr3Motion()
    try:
        motion.ensure_table_collision()
        motion.pick(make_pose(args.pick), approach_height=args.approach_height,
                    width=args.width, force=args.force, scaling=args.scaling,
                    dry_run=dry_run, epsilon=args.epsilon)
        print('picked.' if not dry_run else 'pick: plan only (not executed)')
        motion.place(make_pose(args.place), approach_height=args.approach_height,
                     scaling=args.scaling, dry_run=dry_run)
        print('placed.' if not dry_run else 'place: plan only (not executed)')
    except (RuntimeError, ValueError) as e:
        print('stopped:', e)
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
