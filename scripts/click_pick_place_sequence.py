#!/usr/bin/env python3
"""Two-click RGB-D pick/place sequence for one 40 mm cube.

Default: click cube top -> pick -> lift 20 cm -> click tray floor -> place.
Use --place-held for a cube that is already held by the gripper.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from fr3_skills.motion import BASE_FRAME, Fr3Motion, TCP_LINK, _quat_rotate, load_pose


SURFACE_Z = 0.0222
CUBE_SIZE = 0.040
GRASP_TCP_Z = SURFACE_Z + 0.028
PLACE_TCP_Z = SURFACE_Z + 0.028
LIFT_HEIGHT = 0.20
GRASP_WIDTH = 0.038
GRASP_FORCE = 10.0
ARM_CARTESIAN_SCALING = 0.32
ARM_JOINT_SCALING = 0.20


def click(output, image, label):
    print(f'CLICK REQUIRED: {label}')
    command = [
        sys.executable, str(Path(__file__).with_name('select_black_cube_rgbd.py')),
        '--surface-z', str(SURFACE_Z), '--cube-size-mm', str(CUBE_SIZE * 1000),
        '--out', str(output), '--save', str(image),
    ]
    subprocess.run(command, check=True)
    return json.loads(output.read_text())


def pose(x, y, z, quaternion):
    result = PoseStamped()
    result.header.frame_id = BASE_FRAME
    result.pose.position.x = float(x)
    result.pose.position.y = float(y)
    result.pose.position.z = float(z)
    result.pose.orientation.x = quaternion[0]
    result.pose.orientation.y = quaternion[1]
    result.pose.orientation.z = quaternion[2]
    result.pose.orientation.w = quaternion[3]
    return result


def current_downward(motion):
    current = motion.current_tcp_pose()
    quaternion = current[3:7]
    tool_z = _quat_rotate(quaternion, (0.0, 0.0, 1.0))
    if tool_z[2] > -0.95:
        raise RuntimeError(
            f'TCP is not vertical downward (tool z in base={tool_z}); refusing sequence')
    return current, quaternion


def square_grasp_quaternion(motion, selection):
    square = selection.get('depth_square')
    if not selection.get('depth_square_valid') or not square:
        raise RuntimeError(
            'clicked object did not pass the 40 mm depth-square check: '
            + str(selection.get('depth_square_error')))
    current, _ = current_downward(motion)
    current_q = current[3:7]
    current_x = _quat_rotate(current_q, (1.0, 0.0, 0.0))
    current_yaw = math.atan2(current_x[1], current_x[0])
    edge_yaw = float(square['edge_yaw_base_rad'])
    candidates = [edge_yaw + k*math.pi/2.0 for k in range(-2, 3)]
    yaw = min(candidates, key=lambda value: abs(math.atan2(
        math.sin(value-current_yaw), math.cos(value-current_yaw))))
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    print('depth square: '
          f"{square['side_lengths_m'][0]*1000:.1f} x "
          f"{square['side_lengths_m'][1]*1000:.1f} mm, "
          f'edge yaw={math.degrees(edge_yaw):+.1f} deg, '
          f'selected gripper yaw={math.degrees(yaw):+.1f} deg')
    # Rz(yaw) * Rx(pi), TCP z points down. Franka finger travel is along
    # local Y, which is normal to one of the square's two equivalent faces.
    return (math.cos(yaw/2.0), math.sin(yaw/2.0), 0.0, 0.0)


def execute_cartesian(motion, target, label, scaling=ARM_CARTESIAN_SCALING):
    start = motion.current_joints()
    trajectory = motion.plan_cartesian_path(
        [target], scaling=scaling, max_step=0.005, link_name=TCP_LINK)
    print(f'[{label}]')
    print(motion.describe(trajectory, start))
    motion.activate_arm()
    try:
        motion.execute(trajectory)
    finally:
        motion.deactivate_arm()


def execute_near_home(motion):
    """Carry the grasped cube to the configured high HOME/READY pose."""
    start = motion.current_joints()
    trajectory = motion.plan_to_joints(load_pose('home'), scaling=ARM_JOINT_SCALING)
    print('[carry to near-home pose]')
    print(motion.describe(trajectory, start))
    motion.activate_arm()
    try:
        motion.execute(trajectory)
    finally:
        motion.deactivate_arm()


def verify_gripper_open(motion, minimum_width=0.075, timeout=4.0):
    messages = []
    subscription = motion.create_subscription(
        JointState, '/franka_gripper/joint_states', messages.append, 10)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not messages:
            rclpy.spin_once(motion, timeout_sec=0.1)
        if not messages:
            raise RuntimeError('no gripper joint state; refusing arm motion')
        state = dict(zip(messages[-1].name, messages[-1].position))
        names = ['fr3_finger_joint1', 'fr3_finger_joint2']
        if not all(name in state for name in names):
            raise RuntimeError('gripper joint state is missing finger joints')
        width = state[names[0]] + state[names[1]]
        print(f'verified gripper opening: {width*1000:.1f} mm')
        if width < minimum_width:
            raise RuntimeError(
                f'gripper opened only {width*1000:.1f} mm; '
                f'require at least {minimum_width*1000:.1f} mm before descent')
    finally:
        motion.destroy_subscription(subscription)


def xy(data):
    point = data['visible_top_center_fr3_link0_m']
    return float(point[0]), float(point[1])


def validate_workspace(x, y):
    if not (0.20 <= x <= 0.60 and -0.28 <= y <= 0.28):
        raise RuntimeError(
            f'clicked point ({x:.3f}, {y:.3f}) is outside the allowed tray workspace')


def validate_place_clearance(x, y):
    # Conservative central region measured from the tray view used in this
    # session. This intentionally excludes the wall area involved in the
    # earlier contact. Recalibrate if the tray itself is moved.
    if not (0.36 <= x <= 0.47 and 0.08 <= abs(y) <= 0.19):
        raise RuntimeError(
            f'place point ({x:.3f}, {y:.3f}) is too close to the known tray-wall region; '
            'click nearer the tray centre')


def pick_and_lift(motion, selection, home_gripper=True):
    x, y = xy(selection)
    validate_workspace(x, y)
    quaternion = square_grasp_quaternion(motion, selection)
    hover_z = GRASP_TCP_Z + LIFT_HEIGHT
    # Homing establishes the physical maximum width after a gripper-node or
    # robot recovery. In a validated continuous session it is needed only for
    # the first episode; every episode still verifies the measured opening.
    if home_gripper:
        motion.home_gripper()
    motion.open_gripper(width=0.08, speed=0.05)
    verify_gripper_open(motion)
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'move above pick')
    execute_cartesian(motion, pose(x, y, GRASP_TCP_Z, quaternion), 'descend to grasp')
    motion.grasp(GRASP_WIDTH, speed=0.03, force=GRASP_FORCE,
                 epsilon_inner=0.005, epsilon_outer=0.005)
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'lift 20 cm')
    execute_near_home(motion)
    print('PICK COMPLETE: cube is held at the configured HOME/READY pose.')


def place_and_retreat(motion, selection):
    x, y = xy(selection)
    validate_workspace(x, y)
    validate_place_clearance(x, y)
    _, quaternion = current_downward(motion)
    hover_z = PLACE_TCP_Z + LIFT_HEIGHT
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'move above place')
    execute_cartesian(motion, pose(x, y, PLACE_TCP_Z, quaternion), 'descend to place')
    motion.open_gripper(width=0.08, speed=0.05)
    execute_cartesian(motion, pose(x, y, hover_z, quaternion), 'retreat 20 cm')
    execute_near_home(motion)
    print('PLACE COMPLETE: cube released and robot returned to HOME/READY pose.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--place-held', action='store_true',
                        help='skip pick click/sequence; place the cube already held')
    parser.add_argument('--pick-selection', type=Path,
                        help='reuse an already validated, still-current pick selection JSON')
    parser.add_argument('--place-selection', type=Path,
                        help='reuse an already validated, still-current place selection JSON')
    args = parser.parse_args()
    data_dir = Path('calibration_data')
    data_dir.mkdir(exist_ok=True)

    pick_selection = None
    if not args.place_held:
        if args.pick_selection:
            pick_selection = json.loads(args.pick_selection.read_text())
        else:
            pick_selection = click(
                data_dir/'sequence_pick.json', data_dir/'sequence_pick.jpg',
                'click the centre of the cube top surface')

    rclpy.init()
    motion = Fr3Motion()
    try:
        motion.ensure_table_collision()
        if pick_selection is not None:
            pick_and_lift(motion, pick_selection)
        if args.place_selection:
            place_selection = json.loads(args.place_selection.read_text())
        else:
            place_selection = click(
                data_dir/'sequence_place.json', data_dir/'sequence_place.jpg',
                'click a clear tray-floor location, away from every wall')
        place_and_retreat(motion, place_selection)
    finally:
        motion.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
