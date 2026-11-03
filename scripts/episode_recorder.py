#!/usr/bin/env python3
"""Record one rosbag2 + meta.json per demonstration episode.

Layout (one directory per episode, numbering continues across runs):

    data/demos/
      episodes.jsonl              one summary line per finished episode
      episode_000000/
        bag/                      rosbag2 (sqlite3, zstd per message)
        meta.json                 detection, target, events, success, ...

The bag holds the raw sensor streams. meta.json holds what only the script
knows: what it detected, where it decided to place, when each arm/gripper
command started and finished, and whether the episode succeeded.
"""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

from std_msgs.msg import String


RECORD_TOPICS = [
    # observations
    '/camera/camera/color/image_raw',
    '/camera/camera/color/camera_info',
    '/camera/camera/aligned_depth_to_color/image_raw',
    '/camera/camera/aligned_depth_to_color/camera_info',
    '/franka/joint_states',
    '/franka_gripper/joint_states',
    '/franka_robot_state_broadcaster/current_pose',
    '/tf',
    '/tf_static',
    # actions: commanded joint reference while a trajectory runs
    '/fr3_arm_controller/controller_state',
    # script events (same content as meta.json "events"), for aligning in time
    '/episode/events',
]
# The episode does not start until the recorder is subscribed to these.
REQUIRED_TOPICS = [
    '/camera/camera/color/image_raw',
    '/camera/camera/aligned_depth_to_color/image_raw',
    '/franka/joint_states',
    '/episode/events',
]
EVENT_TOPIC = '/episode/events'


def _jsonable(value):
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if hasattr(value, 'tolist'):
        return value.tolist()
    return repr(value)


def _git_commit():
    try:
        return subprocess.run(
            ['git', 'rev-parse', 'HEAD'], capture_output=True, text=True,
            timeout=2.0).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


class EpisodeRecorder:
    def __init__(self, node, root, run_info=None):
        self.node = node
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.run_info = dict(run_info or {})
        self.run_info.setdefault('git_commit', _git_commit())
        self.publisher = node.create_publisher(String, EVENT_TOPIC, 100)
        self.process = None
        self.meta = None
        self.directory = None
        self._subscribed = set()
        self._log_lines = []
        self._ready = threading.Event()

    @property
    def active(self):
        return self.meta is not None

    def _next_index(self):
        indices = [
            int(path.name.split('_')[1]) for path in self.root.glob('episode_*')
            if path.is_dir() and path.name.split('_')[1].isdigit()]
        return max(indices, default=-1) + 1

    def _read_output(self, stream):
        for line in stream:
            self._log_lines.append(line)
            if 'Subscribed to topic' in line:
                for topic in REQUIRED_TOPICS:
                    if f"'{topic}'" in line:
                        self._subscribed.add(topic)
                if self._subscribed >= set(REQUIRED_TOPICS):
                    self._ready.set()

    def start(self, timeout=15.0):
        """Start the bag and block until every required topic is subscribed."""
        if self.active:
            raise RuntimeError('episode already recording')
        index = self._next_index()
        self.directory = self.root / f'episode_{index:06d}'
        self.directory.mkdir()
        self._subscribed.clear()
        self._log_lines = []
        self._ready.clear()
        command = [
            'ros2', 'bag', 'record',
            '-o', str(self.directory / 'bag'),
            '--compression-mode', 'message', '--compression-format', 'zstd',
            '--compression-threads', '2',
            *RECORD_TOPICS,
        ]
        # Own session: Ctrl+C on the main script must not kill the bag before
        # stop() has written meta.json; stop() sends SIGINT itself.
        self.process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        threading.Thread(
            target=self._read_output, args=(self.process.stdout,),
            daemon=True).start()
        deadline = time.monotonic() + timeout
        while not self._ready.is_set():
            if self.process.poll() is not None:
                self._abort()
                raise RuntimeError(
                    'ros2 bag record exited early:\n' + ''.join(self._log_lines[-20:]))
            if time.monotonic() > deadline:
                missing = sorted(set(REQUIRED_TOPICS) - self._subscribed)
                self._abort()
                raise RuntimeError(f'recorder did not subscribe to {missing} in {timeout:.0f} s '
                                   '(is the camera / robot running?)')
            time.sleep(0.05)
        self.meta = {
            'episode_index': index,
            'directory': str(self.directory),
            'bag': str(self.directory / 'bag'),
            'topics': RECORD_TOPICS,
            'run': self.run_info,
            'start_wall_time': time.time(),
            'start_ros_time_ns': self.node.get_clock().now().nanoseconds,
            'success': None,
            'events': [],
        }
        self.event('episode_start')
        print(f'[recorder] recording {self.directory}')
        return self.directory

    def event(self, name, **data):
        """Timestamp a script event in meta.json and on /episode/events."""
        if not self.active:
            return
        record = {
            'name': name,
            'ros_time_ns': self.node.get_clock().now().nanoseconds,
            'wall_time': time.time(),
            **_jsonable(data),
        }
        self.meta['events'].append(record)
        try:
            self.publisher.publish(String(data=json.dumps(record)))
        except Exception:  # context already shut down by Ctrl+C
            pass

    def update(self, **fields):
        if self.active:
            self.meta.update(_jsonable(fields))

    def stop(self, success, **fields):
        """Stop the bag and write meta.json. Safe to call when not recording."""
        if not self.active:
            return None
        self.event('episode_end', success=success)
        time.sleep(0.3)  # let the final events reach the bag
        self.meta.update(_jsonable(fields))
        self.meta['success'] = success
        self.meta['end_wall_time'] = time.time()
        self.meta['end_ros_time_ns'] = self.node.get_clock().now().nanoseconds
        self.meta['duration_s'] = self.meta['end_wall_time'] - self.meta['start_wall_time']
        self._stop_process()
        meta, self.meta = self.meta, None
        (self.directory / 'meta.json').write_text(json.dumps(meta, indent=2))
        summary = {key: meta.get(key) for key in (
            'episode_index', 'directory', 'success', 'duration_s', 'error',
            'place_error_m')}
        with open(self.root / 'episodes.jsonl', 'a') as handle:
            handle.write(json.dumps(summary) + '\n')
        print(f'[recorder] saved {self.directory} (success={success}, '
              f'{meta["duration_s"]:.1f} s)')
        return meta

    def _stop_process(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.process = None

    def _abort(self):
        self._stop_process()
        self.meta = None
        shutil.rmtree(self.directory, ignore_errors=True)

    def close(self):
        if self.active:
            self.stop(False, error='recorder closed mid-episode')
        self._stop_process()


def instrument_motion(motion, recorder):
    """Log the start/end of every arm and gripper command as recorder events."""
    def wrap(name, describe):
        original = getattr(motion, name)

        def wrapper(*args, **kwargs):
            recorder.event(f'{name}_start', **describe(*args, **kwargs))
            try:
                result = original(*args, **kwargs)
            except BaseException as exc:
                recorder.event(f'{name}_failed', error=repr(exc))
                raise
            recorder.event(f'{name}_done')
            return result
        setattr(motion, name, wrapper)

    def trajectory_info(trajectory, *_, **__):
        points = trajectory.joint_trajectory.points
        return {
            'joint_names': list(trajectory.joint_trajectory.joint_names),
            'num_points': len(points),
            'duration_s': (points[-1].time_from_start.sec
                           + points[-1].time_from_start.nanosec*1e-9) if points else 0.0,
            'goal_positions': list(points[-1].positions) if points else [],
        }

    def gripper_args(*args, **kwargs):
        return {'args': list(args), 'kwargs': kwargs}

    wrap('execute', trajectory_info)
    for name in ('open_gripper', 'close_gripper', 'grasp', 'home_gripper'):
        wrap(name, gripper_args)
