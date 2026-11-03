#!/usr/bin/env python3
"""Run a trained BC / ACT policy on the real FR3 and score each trial.

  # no motion: run the policy on the live camera for a few seconds and print
  python3 il/run_policy.py --ckpt runs/act_v1/best.pt

  # real robot, 10 trials, results appended to runs/act_v1/eval.jsonl
  python3 il/run_policy.py --ckpt runs/act_v1/best.pt --execute --trials 10

One trial:
  1. tilted HOME (same as the demos, no noise), open gripper
  2. detect the cube from HOME (only to score the trial; the policy never sees it)
  3. stream policy joint targets at the training rate to
     /fr3_arm_controller/joint_trajectory. Gripper: when the predicted command
     flips, the arm holds still and the same grasp / open call as the demos runs.
  4. stop after the release + --after-release seconds, or at --max-seconds
  5. back to HOME, detect again: success = cube found on the other tray

Safety limits: per-step joint change capped at the largest step seen in the
demos (and --max-step), joint limits, and the trial aborts if the fingertip
leaves the tray workspace or goes below --min-tcp-z. Keep a hand on the
e-stop anyway.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import torch
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'scripts'))

from common import preprocess, stats_to_torch, STATE_DIM  # noqa: E402
from models import build_policy  # noqa: E402
from auto_cube_transfer import (  # noqa: E402
    CameraReceiver, MotionNoise, auto_detect_cube, move_home)
from click_pick_place_sequence import GRASP_FORCE, GRASP_WIDTH  # noqa: E402
from fr3_skills.motion import JOINTS, Fr3Motion  # noqa: E402

TOPIC = '/fr3_arm_controller/joint_trajectory'
# FR3 joint limits [rad] with a 0.05 rad margin.
JOINT_LOW = np.array([-2.74, -1.78, -2.90, -3.04, -2.81, 0.54, -3.01]) + 0.05
JOINT_HIGH = np.array([2.74, 1.78, 2.90, -0.15, 2.81, 4.52, 3.01]) - 0.05
WORKSPACE_X = (0.20, 0.60)
WORKSPACE_Y = (-0.28, 0.28)


class RobotState:
    """Joint, gripper and TCP state on its own spinning node."""

    def __init__(self):
        self.node = rclpy.create_node('il_policy_state')
        self.lock = threading.Lock()
        self.q = None
        self.width = None
        self.order = None
        self.node.create_subscription(JointState, '/franka/joint_states', self._joints, 10)
        self.node.create_subscription(JointState, '/franka_gripper/joint_states', self._gripper, 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.publisher = self.node.create_publisher(JointTrajectory, TOPIC, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _joints(self, msg):
        if self.order is None:
            self.order = [list(msg.name).index(j) for j in JOINTS]
        with self.lock:
            self.q = np.array([msg.position[i] for i in self.order])

    def _gripper(self, msg):
        with self.lock:
            self.width = float(sum(msg.position))

    def get(self):
        with self.lock:
            return (None if self.q is None else self.q.copy()), self.width

    def tcp_xyz(self):
        try:
            t = self.tf.lookup_transform('fr3_link0', 'fr3_hand_tcp', Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return np.array([tr.x, tr.y, tr.z])

    def command(self, q, seconds):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points = [point]
        self.publisher.publish(msg)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


class Policy:
    """Checkpoint -> (joint target, gripper probability) from raw inputs.
    ACT uses temporal ensembling over overlapping chunks (weights exp(-m*i),
    i = 0 for the oldest prediction, as in the ACT paper)."""

    def __init__(self, path, device, ensemble_m):
        ckpt = torch.load(path, map_location=device, weights_only=False)
        self.config = ckpt['config']
        self.config['pretrained'] = False   # weights come from the checkpoint
        self.model = build_policy(self.config).to(device).eval()
        self.model.load_state_dict(ckpt['model'])
        self.stats_np = ckpt['stats']
        self.stats = stats_to_torch(ckpt['stats'], device)
        self.device = device
        self.h, self.w = self.config['image_hw']
        self.chunk = self.config['chunk']
        self.ensemble_m = ensemble_m
        self.reset()

    def reset(self):
        self.history = []   # (step the chunk was predicted at, chunk K x 8 tensor)
        self.step = 0

    @torch.no_grad()
    def __call__(self, rgb, q, width):
        image = cv2.resize(rgb, (self.w, self.h), interpolation=cv2.INTER_AREA)
        image = torch.from_numpy(image).permute(2, 0, 1)[None].to(self.device)
        state = torch.tensor(np.append(q, width), dtype=torch.float32, device=self.device)[None]
        state = (state - self.stats['state_mean']) / self.stats['state_std']
        with torch.autocast('cuda', dtype=torch.bfloat16):
            chunk = self.model.predict(preprocess(image, False), state)[0].float()
        # Joint outputs are deltas from the joints at prediction time; make them
        # absolute here so chunks predicted at different steps can be averaged.
        q_now = torch.tensor(q, dtype=torch.float32, device=self.device)
        chunk[:, :7] = chunk[:, :7] * self.stats['delta_std'] + self.stats['delta_mean'] + q_now
        chunk[:, 7] = torch.sigmoid(chunk[:, 7])
        if self.chunk == 1:
            action = chunk[0]
        else:
            self.history.append((self.step, chunk))
            self.history = [(s, c) for s, c in self.history if self.step - s < self.chunk]
            preds = torch.stack([c[self.step - s] for s, c in self.history])   # oldest first
            weights = torch.exp(-self.ensemble_m * torch.arange(len(preds), device=self.device))
            action = (preds * (weights / weights.sum())[:, None]).sum(0)
        self.step += 1
        action = action.cpu().numpy()
        return action[:7], float(action[7])


def latest_rgb(camera, max_age):
    colors, _, _ = camera.snapshot()
    if not colors or time.monotonic() - colors[-1][0] > max_age:
        return None
    msg = colors[-1][1]
    frame = np.frombuffer(msg.data, np.uint8).reshape(
        msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
    return frame[:, :, ::-1].copy() if msg.encoding == 'bgr8' else frame


def run_policy_loop(args, policy, robot, camera, motion, execute):
    """Returns (reason, stats dict)."""
    period = 1.0 / policy.config['hz']
    limit = np.minimum(np.asarray(policy.stats_np['max_step']), args.max_step)
    policy.reset()
    gripper_closed = False
    released = False
    release_time = None
    vote = 0
    steps = 0
    start = time.monotonic()
    next_tick = start
    while rclpy.ok():
        now = time.monotonic()
        if now - start > args.max_seconds:
            return 'timeout', {'steps': steps}
        if released and now - release_time > args.after_release:
            return 'finished', {'steps': steps}
        q, width = robot.get()
        rgb = latest_rgb(camera, max_age=0.5)
        if q is None or width is None or rgb is None:
            time.sleep(0.01)
            continue
        if execute:
            tcp = robot.tcp_xyz()
            if tcp is not None and (tcp[2] < args.min_tcp_z
                                    or not WORKSPACE_X[0] - 0.05 <= tcp[0] <= WORKSPACE_X[1] + 0.05
                                    or not WORKSPACE_Y[0] - 0.05 <= tcp[1] <= WORKSPACE_Y[1] + 0.05):
                robot.command(q, 0.2)
                return f'safety stop: tcp at {np.round(tcp, 3).tolist()}', {'steps': steps}

        target, grip_prob = policy(rgb, q, width)
        step = np.clip(target - q, -limit, limit)
        target = np.clip(q + step, JOINT_LOW, JOINT_HIGH)

        # Gripper: two consecutive votes past the hysteresis band.
        want_close = grip_prob > 0.6 if not gripper_closed else not grip_prob < 0.4
        vote = vote + 1 if want_close != gripper_closed else 0
        if not execute:
            if steps % 5 == 0:
                print(f'step {steps:4d}  |dq| max {np.abs(target-q).max():.4f} rad  '
                      f'gripper p(close) {grip_prob:.2f}')
            if now - start > args.dry_seconds:
                return 'dry run done', {'steps': steps}
        elif vote >= 2:
            vote = 0
            robot.command(q, 0.2)   # hold still, as in the demos
            if want_close:
                print('policy: close gripper')
                try:
                    motion.grasp(GRASP_WIDTH, speed=0.03, force=GRASP_FORCE,
                                 epsilon_inner=0.005, epsilon_outer=0.005)
                except RuntimeError as exc:
                    return f'grasp failed ({exc})', {'steps': steps}
                gripper_closed = True
            else:
                print('policy: open gripper')
                motion.open_gripper(width=0.08, speed=0.05)
                gripper_closed = False
                released = True
                release_time = time.monotonic()
            policy.reset()
            next_tick = time.monotonic()
            continue
        else:
            robot.command(target, period * args.lookahead)
        steps += 1
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))
    return 'interrupted', {'steps': steps}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ckpt', type=Path, required=True)
    parser.add_argument('--execute', action='store_true', help='move the real robot')
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--results', type=Path, default=None,
                        help='default: <ckpt dir>/eval.jsonl')
    parser.add_argument('--calibration', type=Path, default=Path('scripts/handeye_d435i.json'))
    parser.add_argument('--max-seconds', type=float, default=90.0)
    parser.add_argument('--after-release', type=float, default=4.0,
                        help='keep running this long after the gripper opens [s]')
    parser.add_argument('--max-step', type=float, default=0.10,
                        help='cap on the per-step joint change [rad]')
    parser.add_argument('--lookahead', type=float, default=1.0,
                        help='time_from_start of each command, in control periods')
    parser.add_argument('--ensemble-m', type=float, default=0.01)
    parser.add_argument('--min-tcp-z', type=float, default=0.035)
    parser.add_argument('--dry-seconds', type=float, default=10.0)
    parser.add_argument('--no-prompt', action='store_true',
                        help='do not wait for Enter between trials')
    args = parser.parse_args()
    results_path = args.results or args.ckpt.parent / 'eval.jsonl'

    policy = Policy(args.ckpt, 'cuda', args.ensemble_m)
    print(f'loaded {args.ckpt}: {policy.config["policy"]}, chunk {policy.chunk}, '
          f'{policy.config["hz"]:g} Hz, step limit '
          f'{np.round(np.minimum(policy.stats_np["max_step"], args.max_step), 3).tolist()}')

    rclpy.init()
    motion = Fr3Motion()
    camera = CameraReceiver()
    robot = RobotState()
    rng_off = MotionNoise(None, enabled=False)
    try:
        if not args.execute:
            reason, info = run_policy_loop(args, policy, robot, camera, motion, execute=False)
            print(f'{reason} ({info["steps"]} steps). Add --execute to move the robot.')
            return
        motion.ensure_table_collision()
        for trial in range(1, args.trials + 1):
            if not args.no_prompt:
                answer = input(f'\n[trial {trial}/{args.trials}] Enter to start, q to quit: ')
                if answer.strip().lower() == 'q':
                    break
            move_home(motion, rng_off, 'HOME view')
            motion.open_gripper(width=0.08, speed=0.05)
            try:
                before = auto_detect_cube(motion, camera, args.calibration, timeout=10.0)
                source_xy = before['visible_top_center_fr3_link0_m'][:2]
            except RuntimeError as exc:
                print(f'cube not found at HOME ({exc}); place it on a tray and retry')
                continue
            print(f'cube at ({source_xy[0]:+.3f}, {source_xy[1]:+.3f}); running policy')
            started = time.time()
            motion.activate_arm()
            try:
                reason, info = run_policy_loop(args, policy, robot, camera, motion, execute=True)
            finally:
                q, _ = robot.get()
                if q is not None:
                    robot.command(q, 0.2)
                time.sleep(0.3)
                motion.deactivate_arm()
            duration = time.time() - started
            print(f'policy stopped: {reason}')
            move_home(motion, rng_off, 'return to HOME')
            try:
                after = auto_detect_cube(motion, camera, args.calibration, timeout=10.0)
                final_xy = after['visible_top_center_fr3_link0_m'][:2]
            except RuntimeError:
                final_xy = None
            success = (reason == 'finished' and final_xy is not None
                       and (final_xy[1] > 0) != (source_xy[1] > 0))
            result = {'trial': trial, 'ckpt': str(args.ckpt), 'policy': policy.config['policy'],
                      'success': success, 'reason': reason, 'source_xy': source_xy,
                      'final_xy': final_xy, 'duration_s': duration, 'steps': info['steps'],
                      'time': time.strftime('%Y-%m-%d %H:%M:%S')}
            with open(results_path, 'a') as f:
                f.write(json.dumps(result) + '\n')
            print(f'trial {trial}: {"SUCCESS" if success else "FAIL"} -> {results_path}')
    finally:
        robot.close()
        camera.close()
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
