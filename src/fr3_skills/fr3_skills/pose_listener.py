"""Subscribe to a PoseStamped topic and move fr3_arm there.

Publish target poses in the fr3_link0 frame (planning frame in moveit.rviz).
The target is the gripper fingertip (fr3_hand_tcp), not fr3_link8 -- motion.py
converts internally.

  ros2 run fr3_skills pose_listener                  # plan only each time (watch RViz)
  ros2 run fr3_skills pose_listener --execute         # also move -- activates/deactivates
                                                       # fr3_arm_controller itself, right
                                                       # around each move (not left active while idle)
"""
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped

from fr3_skills.motion import TCP_LINK, Fr3Motion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/object_pose')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--scaling', type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    motion = Fr3Motion()
    motion.ensure_table_collision()
    # Callback only stores the message; current_joints()/plan/execute below all do
    # their own blocking rclpy.spin_once() calls, which must not run nested inside
    # a subscription callback (breaks the executor's wait set).
    pending = []
    motion.create_subscription(PoseStamped, args.topic, pending.append, 10)

    mode = 'execute' if args.execute else 'plan only'
    print(f'listening on {args.topic} ({mode}) ...')
    try:
        while rclpy.ok():
            rclpy.spin_once(motion, timeout_sec=0.1)
            if not pending:
                continue
            msg = pending.pop(0)
            try:
                start = motion.current_joints()
                trajectory = motion.plan_to_pose(msg, args.scaling, link_name=TCP_LINK)
                print(motion.describe(trajectory, start))
                if not args.execute:
                    print('plan only (not executed) -- check RViz, rerun with --execute to move for real')
                    continue
                motion.activate_arm()
                try:
                    motion.execute(trajectory)
                finally:
                    motion.deactivate_arm()
                print('done. now:', ' '.join(f'{v:+.3f}' for v in motion.current_joints().values()))
            except (RuntimeError, ValueError) as e:
                print('stopped:', e)
    except KeyboardInterrupt:
        pass
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
