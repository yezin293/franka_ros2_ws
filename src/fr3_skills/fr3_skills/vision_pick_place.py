"""Vision-driven pick-and-place: subscribe to the live pose from
object_pose_publisher (run that in another terminal first), then
approach -> pick -> move to place -> place -> home, repeating forever
(Ctrl+C to stop) -- whenever a new object shows up in the pick tray, it
gets moved to the place tray.

  ros2 run fr3_skills object_pose_publisher          # terminal 1, leave running
  ros2 run fr3_skills vision_pick_place               # terminal 2, plan-only
  ros2 run fr3_skills vision_pick_place --execute      # actually move

Detections in the place tray itself (e.g. the object just placed there,
if briefly visible) are ignored via pick_region_y_min in
config/vision_pick_place.yaml -- only detections with y >= that value count
as a new pick target. Adjust it if the pick/place trays are on the other
side of the robot's y-axis.

All tunables (place position, object width, grasp force, etc.) come from
config/vision_pick_place.yaml -- edit that file directly (with
--symlink-install, no rebuild needed) instead of passing flags.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from fr3_skills.motion import Fr3Motion
from fr3_skills.move_home import move_home
from fr3_skills.object_pose_publisher import load_config


def wait_for_pick_target(motion, topic, pick_region_y_min, poll_timeout, heartbeat_every=10.0):
    """Block (spinning motion) until a PoseStamped arrives on topic whose y
    is within the pick tray (>= pick_region_y_min), ignoring anything else
    (e.g. the object sitting in the place tray). Returns that pose -- already
    smoothed/outlier-gated upstream by object_pose_publisher, so the first
    one that clears the y filter is used as-is."""
    pending = []
    sub = motion.create_subscription(PoseStamped, topic, pending.append, 10)
    last_heartbeat = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(motion, timeout_sec=0.2)
            while pending:
                msg = pending.pop(0)
                if msg.pose.position.y >= pick_region_y_min:
                    return msg
            if time.time() - last_heartbeat > heartbeat_every:
                print(f'    still waiting for an object in the pick tray '
                      f'(y >= {pick_region_y_min:+.3f}) ...')
                last_heartbeat = time.time()
    finally:
        motion.destroy_subscription(sub)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true',
                         help='actually move (default: plan-only, check RViz)')
    parser.add_argument('--once', action='store_true',
                         help='run a single pick-place cycle instead of looping forever')
    args = parser.parse_args()
    dry_run = not args.execute
    cfg = load_config()
    pick_region_y_min = cfg.get('pick_region_y_min', 0.0)

    rclpy.init()
    motion = Fr3Motion()
    reset_pub = motion.create_publisher(Empty, cfg['topic'] + '/reset', 1)
    cycle = 0
    try:
        while True:
            cycle += 1
            print(f'=== cycle {cycle} ===')
            # object_pose_publisher smooths/outlier-gates across frames, which
            # would otherwise carry over the previous cycle's own motion (the
            # gripper, tray walls, ...) into this cycle's first real detection
            # and reject it as an "outlier". Tell it to start tracking fresh.
            reset_pub.publish(Empty())
            print(f"[1] waiting for a pick-tray detection on {cfg['topic']} "
                  '(is object_pose_publisher running?) ...')
            pick_pose = wait_for_pick_target(motion, cfg['topic'], pick_region_y_min,
                                              poll_timeout=0.2)
            x, y, z = (pick_pose.pose.position.x, pick_pose.pose.position.y,
                       pick_pose.pose.position.z)
            print(f'    got robot (x={x:+.3f}, y={y:+.3f}, z={z:+.3f})')

            place_pose = PoseStamped()
            place_pose.header.frame_id = pick_pose.header.frame_id
            place_pose.pose.position.x, place_pose.pose.position.y, \
                place_pose.pose.position.z = cfg['place']
            place_pose.pose.orientation.x = 1.0
            place_pose.pose.orientation.w = 0.0

            motion.ensure_table_collision()

            print('[2] pick ...')
            motion.pick(pick_pose, approach_height=cfg['approach_height'],
                        width=cfg['width'], force=cfg['force'], scaling=cfg['scaling'],
                        dry_run=dry_run, epsilon=cfg['epsilon'])
            print('    picked.' if not dry_run else '    pick: plan only (not executed)')

            print('[3] move to place + place ...')
            motion.place(place_pose, approach_height=cfg['approach_height'],
                         dry_run=dry_run)
            print('    placed.' if not dry_run else '    place: plan only (not executed)')

            print('[4] returning home ...')
            move_home(motion, cfg['scaling'], execute=args.execute)
            print('    done.' if not dry_run else '    home: plan only (not executed)')

            if args.once or dry_run:
                if dry_run:
                    print('plan-only mode -- stopping after one cycle '
                          '(nothing actually moved, looping would replan the same object)')
                break
    except KeyboardInterrupt:
        print('\nstopped by Ctrl+C')
    except (RuntimeError, ValueError) as e:
        print('stopped:', e)
        sys.exit(1)
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
