"""Fr3Motion: plan with MoveIt, show the plan, execute only when asked."""
import copy
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.action import Grasp, Homing, Move
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
    PlanningScene, PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
GROUP = 'fr3_arm'
CONTROLLER = 'fr3_arm_controller'
# Tip link of the fr3_arm group (see franka_description group_definition.xacro);
# only valid while load_gripper:=false. Planning frame is fr3_link0 (moveit.rviz).
EE_LINK = 'fr3_link8'
# The real fingertip contact point, 0.1034m further out than fr3_link8 along the
# hand's z-axis (franka_description end_effectors/common/franka_hand.xacro,
# tcp_xyz default). Not usable directly as a plan_to_pose link_name -- the
# kinematics solver for the fr3_arm group only solves IK for fr3_link8, so a
# goal constrained on fr3_hand_tcp comes back GOAL_STATE_INVALID (-27). Instead,
# pick()/place() convert the desired TCP pose to the equivalent EE_LINK pose
# with this fixed offset before planning (see _tcp_pose_to_ee_pose below).
TCP_LINK = 'fr3_hand_tcp'
TCP_OFFSET_Z = 0.1034  # [m] link8 -> hand_tcp, along link8's own z-axis
TCP_YAW_OFFSET = math.pi / 4  # hand frame is link8 rotated by -pi/4 about z (rpy_ee)
MAX_SCALING = 0.2  # OMPL (plan_to_joints/plan_to_pose) cap while testing -- relative
# to the hard joint velocity limits, so 0.2 is already a meaningful slowdown.
# Cartesian paths (plan_cartesian_path) use a different, higher cap: scaling
# there stretches MoveIt's own already-moderate natural timing (measured peak
# ~0.88 rad/s at scaling=1.0 for a 10cm move, 2026-10-12), not the hard limits,
# so 0.2 there would crawl (10cm takes 20+ seconds).
CARTESIAN_MAX_SCALING = 1.0
GRIPPER_MAX_WIDTH = 0.08  # franka_hand full open [m]
BASE_FRAME = 'fr3_link0'
# Real table: 600x600x800mm profile table, base bolted to the middle of one edge
# with the base's back flush against that edge. Assumes the arm's +X faces out
# over the table (the usual Franka convention); flip the sign on TABLE_START_X /
# TABLE_CENTER_X if that's backwards for this mount.
# 2026-10-08: measured + confirmed against the real setup (RViz screenshots vs.
# photos of the table): X/Y footprint and TABLE_TOP_Z=0 match reality exactly --
# this box is the real table, not padded. Don't grow it for safety margin (an
# earlier +0.10m top-margin version visually swallowed most of the robot's own
# base, which is fixed and was never actually at risk). X start measured from
# the collision mesh (franka_description/meshes/robot_arms/fr3/collision/
# link0.stl: spans x=-0.153..+0.070, z=0.000..+0.140) plus a small margin, so
# the base foot plate is fully covered.
BASE_BACK_MARGIN = 0.16  # [m] how far behind x=0 the box starts (measured 0.153 + margin)
TABLE_START_X = -BASE_BACK_MARGIN
TABLE_WIDTH_X = 0.6 + BASE_BACK_MARGIN  # [m] table depth, start (above) to the far edge
TABLE_WIDTH_Y = 0.6  # [m] table width, centered on y=0
TABLE_FLOOR_Z = -0.8  # [m] real floor, from the 800mm table height
TABLE_TOP_Z = 0.0  # [m] real table surface -- keep this exact, see note above
TABLE_HEIGHT = TABLE_TOP_Z - TABLE_FLOOR_Z

# Safety buffer for *target points* only (plan_to_pose's quick z check below) --
# separate from the table model above, which stays true to reality. This is
# what actually keeps commanded targets from grazing the table; the real
# whole-arm protection is still the accurate 'table_floor' collision object.
# 2026-10-08: a real collision happened from a wrist/forearm link clipping a
# profile rail near the table edge, above the true z=0 surface -- hence a
# buffer here, instead of (again) padding the table model itself.
TARGET_Z_MARGIN = 0.03  # [m] in BASE_FRAME
MIN_Z = TARGET_Z_MARGIN
TABLE_CENTER_X = TABLE_START_X + TABLE_WIDTH_X / 2.0
TABLE_CENTER_Z = (TABLE_TOP_Z + TABLE_FLOOR_Z) / 2.0


def load_pose(name):
    path = os.path.join(get_package_share_directory('fr3_skills'), 'config', 'poses.yaml')
    with open(path) as f:
        values = yaml.safe_load(f)[name]
    return dict(zip(JOINTS, values))


def _quat_mul(a, b):
    """Hamilton product a*b for quaternions given as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q=(x,y,z,w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _tcp_pose_to_ee_pose(pose_stamped):
    """Convert a desired TCP_LINK (fr3_hand_tcp) pose to the equivalent EE_LINK
    (fr3_link8) pose, using the fixed offset between them (see TCP_OFFSET_Z /
    TCP_YAW_OFFSET above). Needed because the IK solver only solves for EE_LINK."""
    q = pose_stamped.pose.orientation
    q_tcp = (q.x, q.y, q.z, q.w)
    offset = _quat_rotate(q_tcp, (0.0, 0.0, -TCP_OFFSET_Z))
    yaw_fix = (0.0, 0.0, math.sin(TCP_YAW_OFFSET / 2.0), math.cos(TCP_YAW_OFFSET / 2.0))
    q_ee = _quat_mul(q_tcp, yaw_fix)

    ee_pose = copy.deepcopy(pose_stamped)
    ee_pose.pose.position.x += offset[0]
    ee_pose.pose.position.y += offset[1]
    ee_pose.pose.position.z += offset[2]
    ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, \
        ee_pose.pose.orientation.z, ee_pose.pose.orientation.w = q_ee
    return ee_pose


class Fr3Motion(Node):

    def __init__(self):
        super().__init__('fr3_motion')
        self._latest = None
        # Joint order on /franka/joint_states is not 1..7: always map by name.
        self.create_subscription(JointState, '/franka/joint_states', self._on_state, 10)
        self._move = ActionClient(self, MoveGroup, 'move_action')
        self._execute = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._list = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self._apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._cartesian_path = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._home_gripper = ActionClient(self, Homing, '/franka_gripper/homing')
        self._move_gripper = ActionClient(self, Move, '/franka_gripper/move')
        self._grasp_gripper = ActionClient(self, Grasp, '/franka_gripper/grasp')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def current_tcp_pose(self, timeout=5.0):
        """Current fr3_hand_tcp pose in BASE_FRAME, as (x, y, z, qx, qy, qz, qw).
        Used for calibration -- e.g. jog the robot to touch a known physical
        point, then read this to get that point's coordinate in robot frame
        (more accurate than measuring with a ruler)."""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        last_exc = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            try:
                t = self._tf_buffer.lookup_transform(BASE_FRAME, TCP_LINK, rclpy.time.Time())
                tr, q = t.transform.translation, t.transform.rotation
                return (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
            except TransformException as e:
                last_exc = e
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f'no TF {BASE_FRAME}->{TCP_LINK} after {timeout}s: {last_exc}')

    def _on_state(self, msg):
        self._latest = dict(zip(msg.name, msg.position))

    def current_joints(self, timeout=5.0):
        self._latest = None
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self._latest is None:
            if self.get_clock().now().nanoseconds > end:
                raise RuntimeError('no /franka/joint_states (is bringup running?)')
            rclpy.spin_once(self, timeout_sec=0.1)
        return {j: self._latest[j] for j in JOINTS}

    def controller_active(self):
        if not self._list.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_manager not reachable (is bringup running?)')
        future = self._list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future)
        for c in future.result().controller:
            if c.name == CONTROLLER:
                return c.state == 'active'
        return False

    def _wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def ensure_table_collision(self):
        """Add a collision box for the real table (see TABLE_* above) below
        z=0 in BASE_FRAME, so MoveIt's own collision checking refuses any plan
        where ANY link of the arm (elbow, shoulder, ...) would cross it -- not
        just the single target point that plan_to_pose's MIN_Z check covers.
        Safe to call every time; re-adding the same object id just replaces it."""
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = 'table_floor'
        obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[TABLE_WIDTH_X, TABLE_WIDTH_Y, TABLE_HEIGHT])]
        pose = Pose()
        pose.position.x = TABLE_CENTER_X
        pose.position.z = TABLE_CENTER_Z
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]

        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]

        if not self._apply_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/apply_planning_scene not reachable (is MoveIt running?)')
        result = self._wait(self._apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)))
        if not result.success:
            raise RuntimeError('failed to add table_floor to the planning scene')

    def _switch_controller(self, activate, deactivate):
        if not self._switch.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('controller_manager not reachable (is bringup running?)')
        req = SwitchController.Request(
            activate_controllers=activate, deactivate_controllers=deactivate,
            strictness=SwitchController.Request.STRICT)
        result = self._wait(self._switch.call_async(req))
        if not result.ok:
            raise RuntimeError(f'switch_controller failed (activate={activate}, deactivate={deactivate})')

    def activate_arm(self):
        if self.controller_active():
            return
        self._switch_controller([CONTROLLER], [])

    def deactivate_arm(self):
        if not self.controller_active():
            return
        self._switch_controller([], [CONTROLLER])

    def home_gripper(self):
        if not self._home_gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/franka_gripper/homing not reachable (is load_gripper:=true?)')
        handle = self._wait(self._home_gripper.send_goal_async(Homing.Goal()))
        result = self._wait(handle.get_result_async()).result
        if not result.success:
            raise RuntimeError(f'gripper homing failed: {result.error}')

    def open_gripper(self, width=GRIPPER_MAX_WIDTH, speed=0.1):
        self._move_gripper_to(width, speed)

    def close_gripper(self, speed=0.1):
        self._move_gripper_to(0.0, speed)

    def _move_gripper_to(self, width, speed):
        if not self._move_gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/franka_gripper/move not reachable (is load_gripper:=true?)')
        goal = Move.Goal(width=width, speed=speed)
        handle = self._wait(self._move_gripper.send_goal_async(goal))
        result = self._wait(handle.get_result_async()).result
        if not result.success:
            raise RuntimeError(f'gripper move failed: {result.error}')

    def grasp(self, width, speed=0.1, force=20.0, epsilon_inner=0.005, epsilon_outer=0.005):
        if not self._grasp_gripper.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/franka_gripper/grasp not reachable (is load_gripper:=true?)')
        goal = Grasp.Goal(
            width=width, speed=speed, force=force,
            epsilon=GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer))
        handle = self._wait(self._grasp_gripper.send_goal_async(goal))
        result = self._wait(handle.get_result_async()).result
        if not result.success:
            raise RuntimeError(f'grasp failed: {result.error}')

    def _approach_pose(self, pose_stamped, height):
        approach = copy.deepcopy(pose_stamped)
        approach.pose.position.z += height
        return approach

    def _plan_leg(self, label, pose_stamped, scaling, dry_run, cartesian=False):
        """Plan one leg; in dry_run just plan+describe (also visible in RViz via
        move_group's display_planned_path), without touching hardware.
        cartesian=True uses plan_cartesian_path (straight line -- for
        descend/retreat, where the end effector should move predictably
        straight down/up, not OMPL's sampling-based wandering path)."""
        start = self.current_joints()
        if cartesian:
            trajectory = self.plan_cartesian_path([pose_stamped], scaling, link_name=TCP_LINK)
        else:
            trajectory = self.plan_to_pose(pose_stamped, scaling, link_name=TCP_LINK)
        print(f'[{label}]')
        print(self.describe(trajectory, start))
        if not dry_run:
            self.execute(trajectory)

    def pick(self, pose_stamped, approach_height=0.1, width=0.0, speed=0.1, force=20.0,
              scaling=0.1, cartesian_scaling=1.0, dry_run=False, epsilon=0.005):
        """Approach above pose_stamped (TCP_LINK) freely, descend/retreat in a
        straight line, grasp in between. dry_run=True only plans+prints each
        leg (check RViz) -- no activate/execute/gripper calls, nothing moves
        for real. cartesian_scaling defaults to 1.0 (MoveIt's own natural,
        already-moderate Cartesian speed -- unlike OMPL's scaling, this is not
        relative to the hard joint limits, so 0.1 here would crawl, e.g. 20+
        seconds for a 10cm descend)."""
        approach = self._approach_pose(pose_stamped, approach_height)
        if dry_run:
            self._plan_leg('approach', approach, scaling, dry_run)
            print(f'[gripper] would open_gripper(speed={speed})')
            self._plan_leg('descend', pose_stamped, cartesian_scaling, dry_run, cartesian=True)
            print(f'[gripper] would grasp(width={width}, speed={speed}, force={force})')
            self._plan_leg('retreat', approach, cartesian_scaling, dry_run, cartesian=True)
            return
        self.activate_arm()
        try:
            self._plan_leg('approach', approach, scaling, dry_run)
            self.open_gripper(speed=speed)
            self._plan_leg('descend', pose_stamped, cartesian_scaling, dry_run, cartesian=True)
            self.grasp(width, speed=speed, force=force,
                       epsilon_inner=epsilon, epsilon_outer=epsilon)
            self._plan_leg('retreat', approach, cartesian_scaling, dry_run, cartesian=True)
        finally:
            self.deactivate_arm()

    def place(self, pose_stamped, approach_height=0.1, speed=0.1, scaling=0.1,
              cartesian_scaling=1.0, dry_run=False):
        """Approach above pose_stamped (TCP_LINK) freely, descend/retreat in a
        straight line, release in between. dry_run=True only plans+prints each
        leg (check RViz) -- no activate/execute/gripper calls, nothing moves
        for real. See pick() for why cartesian_scaling defaults to 1.0."""
        approach = self._approach_pose(pose_stamped, approach_height)
        if dry_run:
            self._plan_leg('approach', approach, scaling, dry_run)
            self._plan_leg('descend', pose_stamped, cartesian_scaling, dry_run, cartesian=True)
            print(f'[gripper] would open_gripper(speed={speed})')
            self._plan_leg('retreat', approach, cartesian_scaling, dry_run, cartesian=True)
            return
        self.activate_arm()
        try:
            self._plan_leg('approach', approach, scaling, dry_run)
            self._plan_leg('descend', pose_stamped, cartesian_scaling, dry_run, cartesian=True)
            self.open_gripper(speed=speed)
            self._plan_leg('retreat', approach, cartesian_scaling, dry_run, cartesian=True)
        finally:
            self.deactivate_arm()

    def plan_to_joints(self, target, scaling=0.1):
        """Return a planned RobotTrajectory to the joint target, or raise."""
        if not 0.0 < scaling <= MAX_SCALING:
            raise ValueError(f'scaling must be in (0, {MAX_SCALING}]')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        # OMPL is sampling-based -- a single attempt can fail on a reachable,
        # collision-free goal purely from bad luck, especially with tight
        # position/orientation tolerances. Let it retry internally with
        # different random seeds instead of surfacing a one-off failure.
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = scaling
        req.max_acceleration_scaling_factor = scaling
        req.start_state.is_diff = True
        c = Constraints()
        for j in JOINTS:
            c.joint_constraints.append(JointConstraint(
                joint_name=j, position=target[j],
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        req.goal_constraints = [c]
        goal.planning_options.plan_only = True

        if not self._move.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('move_group not reachable (is MoveIt running?)')
        handle = self._wait(self._move.send_goal_async(goal))
        result = self._wait(handle.get_result_async()).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'planning failed: error_code {result.error_code.val}')
        return result.planned_trajectory

    def plan_to_pose(self, pose_stamped, scaling=0.1, tolerance_m=0.01, tolerance_rad=0.01,
                      link_name=EE_LINK):
        """Return a planned RobotTrajectory to a Cartesian pose target for link_name."""
        if not 0.0 < scaling <= MAX_SCALING:
            raise ValueError(f'scaling must be in (0, {MAX_SCALING}]')
        if pose_stamped.header.frame_id != BASE_FRAME:
            raise ValueError(
                f'pose frame_id must be {BASE_FRAME!r} (got {pose_stamped.header.frame_id!r}) '
                f'-- the z >= {MIN_Z} safety check below assumes that frame')
        if pose_stamped.pose.position.z < MIN_Z:
            raise ValueError(
                f'target z={pose_stamped.pose.position.z:.3f} is below the MIN_Z safety '
                f'floor ({MIN_Z}) in {BASE_FRAME} -- refusing to plan')
        if link_name == TCP_LINK:
            # The IK solver only solves for EE_LINK -- convert the TCP target to
            # the equivalent EE_LINK target via the known fixed offset.
            pose_stamped = _tcp_pose_to_ee_pose(pose_stamped)
            link_name = EE_LINK
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        # OMPL is sampling-based -- a single attempt can fail on a reachable,
        # collision-free goal purely from bad luck, especially with tight
        # position/orientation tolerances. Let it retry internally with
        # different random seeds instead of surfacing a one-off failure.
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = scaling
        req.max_acceleration_scaling_factor = scaling
        req.start_state.is_diff = True

        position = PositionConstraint()
        position.header = pose_stamped.header
        position.link_name = link_name
        position.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance_m]))
        position.constraint_region.primitive_poses.append(pose_stamped.pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = pose_stamped.header
        orientation.link_name = link_name
        orientation.orientation = pose_stamped.pose.orientation
        orientation.absolute_x_axis_tolerance = tolerance_rad
        orientation.absolute_y_axis_tolerance = tolerance_rad
        orientation.absolute_z_axis_tolerance = tolerance_rad
        orientation.weight = 1.0

        c = Constraints()
        c.position_constraints = [position]
        c.orientation_constraints = [orientation]
        req.goal_constraints = [c]
        goal.planning_options.plan_only = True

        if not self._move.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('move_group not reachable (is MoveIt running?)')
        handle = self._wait(self._move.send_goal_async(goal))
        result = self._wait(handle.get_result_async()).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'planning failed: error_code {result.error_code.val}')
        return result.planned_trajectory

    def plan_cartesian_path(self, poses, scaling=0.1, max_step=0.01, link_name=TCP_LINK):
        """Plan a straight-line path for link_name through poses (list of
        PoseStamped, all in BASE_FRAME), via MoveIt's compute_cartesian_path.
        Unlike plan_to_pose (OMPL), the path between waypoints is a straight
        line in task space -- no wandering/twisting through intermediate
        joint configurations. Use this for approach/retreat/descend, where the
        end effector should move in a predictable straight line (e.g. straight
        down onto an object), not plan_to_pose's sampling-based path.

        compute_cartesian_path already returns a time-parameterized trajectory
        (real velocities, not just waypoint positions) -- verified directly
        against this move_group (2026-10-12). It has no velocity-scaling input
        though, so scaling here uniformly stretches time afterward. See
        CARTESIAN_MAX_SCALING above for why this cap differs from MAX_SCALING.
        """
        if not 0.0 < scaling <= CARTESIAN_MAX_SCALING:
            raise ValueError(f'scaling must be in (0, {CARTESIAN_MAX_SCALING}]')
        for p in poses:
            if p.header.frame_id != BASE_FRAME:
                raise ValueError(f'pose frame_id must be {BASE_FRAME!r} (got {p.header.frame_id!r})')
            if p.pose.position.z < MIN_Z:
                raise ValueError(
                    f'target z={p.pose.position.z:.3f} is below the MIN_Z safety '
                    f'floor ({MIN_Z}) in {BASE_FRAME} -- refusing to plan')

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = GROUP
        req.link_name = link_name
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state.joint_state.name = JOINTS
        req.start_state.joint_state.position = [self.current_joints()[j] for j in JOINTS]
        req.start_state.is_diff = False
        req.waypoints = [p.pose for p in poses]

        if not self._cartesian_path.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/compute_cartesian_path not reachable (is MoveIt running?)')
        result = self._wait(self._cartesian_path.call_async(req))
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'cartesian planning failed: error_code {result.error_code.val}')
        if result.fraction < 0.999:
            raise RuntimeError(
                f'cartesian path only {result.fraction * 100:.0f}% complete '
                '(straight line blocked by a jump/collision before the goal)')
        return self._retime(result.solution, scaling)

    @staticmethod
    def _retime(trajectory, scaling):
        """Uniformly stretch time by 1/scaling (slower for scaling<1), scaling
        velocities/accelerations down to match -- preserves the path shape."""
        for pt in trajectory.joint_trajectory.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            t /= scaling
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pt.velocities = [v * scaling for v in pt.velocities]
            pt.accelerations = [a * scaling * scaling for a in pt.accelerations]
        return trajectory

    def describe(self, trajectory, start):
        traj = trajectory.joint_trajectory
        last = traj.points[-1]
        duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        lines = [f'plan: {len(traj.points)} points, {duration:.2f} s']
        for j in JOINTS:
            k = traj.joint_names.index(j)
            goal = last.positions[k]
            lines.append(f'  {j}: {start[j]:+.3f} -> {goal:+.3f}  (change {goal - start[j]:+.3f} rad)')
        return '\n'.join(lines)

    def execute(self, trajectory):
        if not self.controller_active():
            raise RuntimeError(f'{CONTROLLER} is not active: activate it first')
        if not self._execute.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('execute_trajectory not reachable')
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        handle = self._wait(self._execute.send_goal_async(goal))
        code = self._wait(handle.get_result_async()).result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'execution failed: error_code {code}')
