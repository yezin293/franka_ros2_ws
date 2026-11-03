#!/usr/bin/env python3
"""Convert recorded demos (data/demos/episode_*/bag) into one HDF5 file for
BC / ACT training.

Per kept frame (sampled at --hz from the 15 Hz colour stream):
  images         (N, H, W, 3) uint8 RGB, resized wrist camera
  qpos           (N, 7)  arm joints [rad], fr3_joint1..7, nearest 1 kHz sample
  gripper_width  (N,)    finger opening [m]
  gripper_cmd    (N,)    1 = closed by grasp, 0 = opened (from script events)
  action         (N, 8)  next kept frame's [qpos(7), gripper_cmd]
  stamp_ns       (N,)    colour image header stamp

Frames where nothing moves (detection waits, the hover re-detection pause,
planning gaps, the final check at HOME) are dropped. Otherwise the policy
learns to stand still in those states and can stall there forever.

Usage (ROS 2 sourced, from the workspace root):
  python3 il/convert_bags.py                    # successful episodes >= 32
  python3 il/convert_bags.py --min-episode 0    # all successful episodes
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

import cv2
import h5py
import numpy as np

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
COLOR_TOPIC = '/camera/camera/color/image_raw'
JOINT_TOPIC = '/franka/joint_states'
GRIPPER_TOPIC = '/franka_gripper/joint_states'


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[COLOR_TOPIC, JOINT_TOPIC, GRIPPER_TOPIC]))
    colors, joint_t, joint_q, grip_t, grip_w = [], [], [], [], []
    order = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == COLOR_TOPIC:
            colors.append((stamp_ns(msg.header), msg))
        elif topic == JOINT_TOPIC:
            if order is None:
                order = [list(msg.name).index(j) for j in JOINTS]
            joint_t.append(stamp_ns(msg.header))
            joint_q.append([msg.position[i] for i in order])
        else:
            grip_t.append(stamp_ns(msg.header))
            grip_w.append(float(sum(msg.position)))
    colors.sort(key=lambda item: item[0])
    return (colors, np.asarray(joint_t, np.int64), np.asarray(joint_q, np.float64),
            np.asarray(grip_t, np.int64), np.asarray(grip_w, np.float64))


def nearest(times, values, query):
    index = np.clip(np.searchsorted(times, query), 1, len(times)-1)
    left_closer = (query - times[index-1]) < (times[index] - query)
    return values[np.where(left_closer, index-1, index)]


def gripper_command_events(events):
    """[(ros_time_ns, cmd)] from the script's gripper calls."""
    changes = []
    for event in events:
        if event['name'] == 'grasp_start':
            changes.append((event['ros_time_ns'], 1.0))
        elif event['name'] in ('open_gripper_start', 'home_gripper_start'):
            changes.append((event['ros_time_ns'], 0.0))
    return sorted(changes)


def command_at(changes, t):
    value = 0.0
    for time_ns, cmd in changes:
        if time_ns <= t:
            value = cmd
        else:
            break
    return value


def convert_episode(job):
    directory, args = job
    meta = json.loads((directory / 'meta.json').read_text())
    colors, joint_t, joint_q, grip_t, grip_w = read_bag(directory / 'bag')
    if not colors or len(joint_t) < 2 or len(grip_t) < 2:
        return directory.name, None, 'missing streams'

    # Subsample the colour stream: nearest frame to each point of a --hz grid.
    period = int(1e9 / args.hz)
    color_t = np.asarray([t for t, _ in colors], np.int64)
    grid = np.arange(color_t[0], color_t[-1] + 1, period)
    picks = np.unique(nearest(color_t, np.arange(len(colors)), grid))
    kept = [colors[i] for i in picks]
    times = color_t[picks]
    qpos = nearest(joint_t, joint_q, times)
    width = nearest(grip_t, grip_w, times)
    changes = gripper_command_events(meta['events'])
    cmd = np.asarray([command_at(changes, t) for t in times])

    # Moving = joints, finger width or gripper command change within the next
    # --idle-window seconds.
    look = int(args.idle_window * 1e9)
    q_future = nearest(joint_t, joint_q, times + look)
    w_future = nearest(grip_t, grip_w, times + look)
    cmd_future = np.asarray([command_at(changes, t + look) for t in times])
    moving = ((np.abs(q_future - qpos).max(axis=1) > args.idle_joint_rad)
              | (np.abs(w_future - width) > args.idle_width_m)
              | (cmd_future != cmd))
    # The first episode of each run homes the gripper (close + open) before the
    # task starts; skip everything before the pick's first open command.
    opens = [e['ros_time_ns'] for e in meta['events'] if e['name'] == 'open_gripper_start']
    if opens:
        moving &= times >= opens[0]
    keep = np.flatnonzero(moving)
    if len(keep) < 20:
        return directory.name, None, f'only {len(keep)} moving frames'

    images = []
    for index in keep:
        msg = kept[index][1]
        if msg.encoding not in ('rgb8', 'bgr8'):
            return directory.name, None, f'unsupported encoding {msg.encoding}'
        frame = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            frame = frame[:, :, ::-1]
        images.append(cv2.resize(frame, (args.width, args.height),
                                 interpolation=cv2.INTER_AREA))
    qpos, width, cmd, times = qpos[keep], width[keep], cmd[keep], times[keep]
    state_next = np.concatenate([qpos, cmd[:, None]], axis=1)
    action = np.concatenate([state_next[1:], state_next[-1:]], axis=0)
    return directory.name, {
        'images': np.stack(images),
        'qpos': qpos.astype(np.float32),
        'gripper_width': width.astype(np.float32),
        'gripper_cmd': cmd.astype(np.float32),
        'action': action.astype(np.float32),
        'stamp_ns': times,
        'raw_frames': len(kept),
    }, None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--demos', type=Path, default=Path('data/demos'))
    parser.add_argument('--out', type=Path, default=None,
                        help='default: data/il/demos_<hz>hz_<H>x<W>.hdf5')
    parser.add_argument('--min-episode', type=int, default=32,
                        help='first episode index to use (32 = hover re-detection on)')
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--height', type=int, default=192)
    parser.add_argument('--width', type=int, default=256)
    parser.add_argument('--idle-window', type=float, default=0.3, help='[s]')
    parser.add_argument('--idle-joint-rad', type=float, default=0.002)
    parser.add_argument('--idle-width-m', type=float, default=0.001)
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    if args.out is None:
        args.out = Path(f'data/il/demos_{args.hz:g}hz_{args.height}x{args.width}.hdf5')

    summaries = [json.loads(line) for line in
                 (args.demos / 'episodes.jsonl').read_text().splitlines() if line.strip()]
    chosen = sorted({s['episode_index'] for s in summaries
                     if s['success'] and s['episode_index'] >= args.min_episode})
    directories = [args.demos / f'episode_{i:06d}' for i in chosen]
    print(f'{len(directories)} successful episodes >= {args.min_episode}')

    args.out.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.out.with_suffix('.tmp')
    total_kept = total_raw = 0
    with h5py.File(tmp, 'w') as out, ProcessPoolExecutor(args.workers) as pool:
        out.attrs['hz'] = args.hz
        out.attrs['image_hw'] = (args.height, args.width)
        out.attrs['joints'] = JOINTS
        out.attrs['action_layout'] = 'next kept frame: fr3_joint1..7 [rad], gripper_cmd (1=closed)'
        out.attrs['min_episode'] = args.min_episode
        for name, data, error in pool.map(convert_episode,
                                          [(d, args) for d in directories]):
            if data is None:
                print(f'  skip {name}: {error}')
                continue
            group = out.create_group(name)
            group.create_dataset('images', data=data['images'],
                                 chunks=(1, args.height, args.width, 3), compression='lzf')
            for key in ('qpos', 'gripper_width', 'gripper_cmd', 'action', 'stamp_ns'):
                group.create_dataset(key, data=data[key])
            total_kept += len(data['qpos'])
            total_raw += data['raw_frames']
            print(f'  {name}: {len(data["qpos"])} / {data["raw_frames"]} frames kept')
        out.attrs['episodes'] = len(out.keys())
    tmp.replace(args.out)
    print(f'wrote {args.out}: {total_kept} frames '
          f'({total_raw-total_kept} idle frames dropped)')


if __name__ == '__main__':
    main()
