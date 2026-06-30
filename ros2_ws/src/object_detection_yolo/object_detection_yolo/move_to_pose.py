#!/usr/bin/env python3
"""

"""
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume)

GROUP      = "ur_manipulator"
EE_LINK    = "grasp_tcp"   # group tip = grasp point; goal XYZ is the fingertip target
PLAN_FRAME = "world"
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GRIP_CLOSE = 0.6
VEL_SCALE  = 0.3
ACC_SCALE  = 0.3


class MoveToPose(Node):
    def __init__(self):
        super().__init__("move_to_pose")
        self.cb = ReentrantCallbackGroup()

        self.declare_parameter("x", 0.5)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.5)
        self.declare_parameter("qx", 1.0)
        self.declare_parameter("qy", 0.0)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 0.0)
        self.declare_parameter("free_orientation", True)
        self.declare_parameter("ori_tol", 0.2)
        self.declare_parameter("close_gripper", False)
        self.declare_parameter("home_first", True)
        self.declare_parameter("approach", 0.10)   # hover this far above target, then
                                                   # descend straight down (m); 0 = go direct

        self.move_client = ActionClient(self, MoveGroup, "/move_action",
                                        callback_group=self.cb)
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory",
                                 callback_group=self.cb)
        self.arm_traj = ActionClient(self, FollowJointTrajectory,
                                     "/arm_controller/follow_joint_trajectory",
                                     callback_group=self.cb)

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------- generic FollowJointTrajectory send (gripper + home) ----------
    def _send_traj(self, client, goal, timeout):
        done = threading.Event(); res = {}
        def on_res(_):
            res["ok"] = True; done.set()
        def on_goal(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error("trajectory goal REJECTED"); done.set(); return
            gh.get_result_async().add_done_callback(on_res)
        client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=timeout)
        return res.get("ok", False)

    def home_arm(self, t=4.0):
        # All-zero joints = arm straight up, clear of any table -> a collision-free
        # start state. Sent directly to the controller so it works even if MoveIt
        # would refuse the current pose (START_STATE_IN_COLLISION).
        self.get_logger().info("homing arm (straight up) ...")
        if not self.arm_traj.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("arm_controller server not available"); return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * 6
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points = [pt]
        ok = self._send_traj(self.arm_traj, goal, t + 5.0)
        self.get_logger().info("arm homed" if ok else "home FAILED")
        return ok

    def set_gripper(self, pos, t=1.0):
        if not self.grip.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("gripper server not available"); return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["finger_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points = [pt]
        ok = self._send_traj(self.grip, goal, t + 3.0)
        self.get_logger().info(f"gripper -> {pos}")
        return ok

    # ---------- the actual MoveIt goal ----------
    def move_to(self, xyz, quat, free_orientation, ori_tol):
        self.get_logger().info(
            "move_to %.3f %.3f %.3f  (free_orientation=%s)"
            % (xyz[0], xyz[1], xyz[2], free_orientation))
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action not available -- is move_group running?")
            return False

        goal = MoveGroup.Goal()
        req: MotionPlanRequest = goal.request
        req.group_name = GROUP
        req.pipeline_id = "ompl"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        # position: keep tool0 inside a tiny sphere at the goal point
        pc = PositionConstraint()
        pc.header.frame_id = PLAN_FRAME
        pc.link_name = EE_LINK
        bv = BoundingVolume()
        bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        pose = Pose()
        pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        pose.orientation.w = 1.0
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        if not free_orientation:
            oc = OrientationConstraint()
            oc.header.frame_id = PLAN_FRAME
            oc.link_name = EE_LINK
            oc.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                        z=float(quat[2]), w=float(quat[3]))
            oc.absolute_x_axis_tolerance = float(ori_tol)
            oc.absolute_y_axis_tolerance = float(ori_tol)
            oc.absolute_z_axis_tolerance = float(ori_tol)
            oc.weight = 1.0
            c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False   # plan AND execute

        done = threading.Event(); res = {}
        def on_res(fut):
            res["r"] = fut.result().result; done.set()
        def on_goal(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().error("move goal REJECTED"); done.set(); return
            gh.get_result_async().add_done_callback(on_res)
        self.move_client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=60.0)
        r = res.get("r")
        ok = r is not None and r.error_code.val == 1   # MoveItErrorCodes.SUCCESS
        self.get_logger().info(
            "move OK" if ok else
            f"move FAIL (MoveItErrorCode {r.error_code.val if r else 'none'})")
        return ok

    def run(self):
        x, y, z = self._p("x"), self._p("y"), self._p("z")
        quat = [self._p("qx"), self._p("qy"), self._p("qz"), self._p("qw")]
        free = self._p("free_orientation"); tol = self._p("ori_tol")
        approach = self._p("approach")
        if self._p("home_first"):
            self.home_arm()
        # 1) pre-grasp: hover 'approach' m above the target (same x, y)
        if approach > 0.0:
            if not self.move_to([x, y, z + approach], quat, free, tol):
                self.get_logger().info("=== SMOKE TEST FAILED (pre-grasp) ==="); return
        # 2) descend straight down to the target
        ok = self.move_to([x, y, z], quat, free, tol)
        # 3) close on arrival
        if ok and self._p("close_gripper"):
            self.set_gripper(GRIP_CLOSE)
        self.get_logger().info("=== SMOKE TEST PASSED ===" if ok
                               else "=== SMOKE TEST FAILED (see above) ===")


def main():
    rclpy.init()
    node = MoveToPose()
    ex = MultiThreadedExecutor(2)
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
