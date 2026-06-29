#!/usr/bin/env python3
# Pick & place with NO third-party deps: talks to the running move_group via the
# official MoveGroup action, the gripper via FollowJointTrajectory, and the planning
# scene via ApplyPlanningScene. All from rclpy + moveit_msgs (already installed).
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from vision_msgs.msg import Detection3DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, PlanningScene,
                             CollisionObject, AttachedCollisionObject)

# ---------------- TUNE THESE (see object_pick_place.md Step 1) ----------------
GROUP        = "ur_manipulator"
EE_LINK      = "tool0"
PLAN_FRAME   = "world"
TCP_OFFSET   = 0.16            # tool0 above the grasp point (m)
APPROACH     = 0.12            # pre-grasp height above grasp (m)
LIFT         = 0.20            # how high to lift after grasping (m)
GRASP_QUAT   = [1.0, 0.0, 0.0, 0.0]   # gripper pointing down (x,y,z,w)
GRIP_OPEN    = 0.0
GRIP_CLOSE   = 0.6
TARGET_CLASSES = ["cup"]   # exact YOLO names
BIN_FRAME    = "bin"           # your bin TF frame
PLACE_CLEAR  = 0.15            # height above the bin to release (m)
VEL_SCALE    = 0.3
ACC_SCALE    = 0.3
TOUCH_LINKS  = ["left_inner_finger_pad", "right_inner_finger_pad",
                "left_inner_finger", "right_inner_finger"]
ARM_JOINTS   = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # all zero = arm straight up
# -----------------------------------------------------------------------------


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.cb = ReentrantCallbackGroup()

        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb)
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory",
                                 callback_group=self.cb)
        self.arm_traj = ActionClient(self, FollowJointTrajectory,
                                     "/arm_controller/follow_joint_trajectory",
                                     callback_group=self.cb)
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                            callback_group=self.cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._latest = {}   # class_id -> (x,y,z)
        self.create_subscription(Detection3DArray, "/detected_objects_3d",
                                 self.det_cb, 10, callback_group=self.cb)

    # ---------- detection ----------
    def det_cb(self, msg: Detection3DArray):
        for d in msg.detections:
            if not d.results:
                continue
            p = d.results[0].pose.pose.position
            self._latest[d.results[0].hypothesis.class_id] = (p.x, p.y, p.z)

    def wait_for_object(self):
        for _ in range(60):
            for cls in TARGET_CLASSES:
                xyz = self._latest.get(cls)
                if xyz is not None:
                    self.get_logger().info(f"target object: {cls} @ {xyz}")
                    return xyz
            threading.Event().wait(0.5)   # executor spins in the bg thread
        return None

    def lookup_bin(self):
        for _ in range(20):
            try:
                t = self.tf_buffer.lookup_transform(PLAN_FRAME, BIN_FRAME, rclpy.time.Time())
                tr = t.transform.translation
                return (tr.x, tr.y, tr.z)
            except Exception:
                self.get_logger().warn(f"waiting for TF {PLAN_FRAME}->{BIN_FRAME} ...")
                threading.Event().wait(0.5)
        return None

    # ---------- arm: MoveGroup action (plan + execute) ----------
    def move_to(self, xyz, quat=GRASP_QUAT):
        self.get_logger().info("move_to %.3f %.3f %.3f" % tuple(xyz))
        goal = MoveGroup.Goal()
        req: MotionPlanRequest = goal.request
        req.group_name = GROUP
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnectkConfigDefault"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        pc = PositionConstraint()
        pc.header.frame_id = PLAN_FRAME
        pc.link_name = EE_LINK
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        bv = BoundingVolume()
        bv.primitives.append(region)
        pose = Pose()
        pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        pose.orientation.w = 1.0
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = PLAN_FRAME
        oc.link_name = EE_LINK
        oc.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                    z=float(quat[2]), w=float(quat[3]))
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal.planning_options.plan_only = False   # plan AND execute via move_group

        self.move_client.wait_for_server()
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
        self.get_logger().info("move %s" % ("OK" if ok else
                               f"FAIL (code {r.error_code.val if r else 'none'})"))
        return ok

    # ---------- home the arm straight up (direct controller, no collision check) ----------
    def home_arm(self, t=4.0):
        # Sent straight to arm_controller so it works even if the arm currently rests
        # on/in the table (MoveIt would refuse: START_STATE_IN_COLLISION). All-zero
        # joints = arm pointing straight up, clear of the table -> safe planning start.
        self.get_logger().info("homing arm (straight up) ...")
        self.arm_traj.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in HOME_POSITIONS]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points = [pt]
        done = threading.Event()
        def on_res(_): done.set()
        def on_goal(fut):
            gh = fut.result()
            if not gh.accepted:
                done.set(); return
            gh.get_result_async().add_done_callback(on_res)
        self.arm_traj.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=t + 5.0)
        self.get_logger().info("arm homed (straight)")

    # ---------- gripper: FollowJointTrajectory action ----------
    def set_gripper(self, pos, t=1.0):
        self.grip.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["finger_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points = [pt]
        done = threading.Event()
        def on_res(_): done.set()
        def on_goal(fut):
            gh = fut.result()
            if not gh.accepted:
                done.set(); return
            gh.get_result_async().add_done_callback(on_res)
        self.grip.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=t + 3.0)
        self.get_logger().info(f"gripper -> {pos}")

    # ---------- planning scene: ApplyPlanningScene service ----------
    def _apply(self, scene: PlanningScene):
        self.scene_cli.wait_for_service()
        done = threading.Event()
        self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene)
                                  ).add_done_callback(lambda f: done.set())
        done.wait(timeout=5.0)

    def attach_object(self, oid, xyz, size=(0.06, 0.06, 0.10)):
        co = CollisionObject()
        co.id = oid
        co.header.frame_id = PLAN_FRAME
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
        pose = Pose(); pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2]); pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        aco = AttachedCollisionObject()
        aco.link_name = EE_LINK
        aco.object = co
        aco.touch_links = TOUCH_LINKS
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        self._apply(scene)
        self.get_logger().info(f"attached '{oid}'")

    def detach_object(self, oid):
        aco = AttachedCollisionObject()
        aco.link_name = EE_LINK
        aco.object.id = oid
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        rm = CollisionObject(); rm.id = oid; rm.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(rm)
        self._apply(scene)
        self.get_logger().info(f"detached '{oid}'")

    # ---------- sequence ----------
    def run(self):
        self.home_arm()          # get the arm straight & clear of the table first
        obj = self.wait_for_object()
        if obj is None:
            self.get_logger().error(f"no {TARGET_CLASSES} detected"); return
        bin_xyz = self.lookup_bin()
        if bin_xyz is None:
            self.get_logger().error(f"bin frame '{BIN_FRAME}' not in TF"); return

        ox, oy, oz = obj
        grasp_z = oz + TCP_OFFSET
        self.get_logger().info(f"object @ {obj}, bin @ {bin_xyz}")

        # PICK
        self.set_gripper(GRIP_OPEN)
        if not self.move_to([ox, oy, grasp_z + APPROACH]): return
        if not self.move_to([ox, oy, grasp_z]): return
        self.set_gripper(GRIP_CLOSE)
        self.attach_object("target", [ox, oy, oz])
        if not self.move_to([ox, oy, grasp_z + LIFT]): return

        # PLACE
        bx, by, bz = bin_xyz
        place_z = bz + TCP_OFFSET + PLACE_CLEAR
        if not self.move_to([bx, by, place_z + APPROACH]): return
        if not self.move_to([bx, by, place_z]): return
        self.set_gripper(GRIP_OPEN)
        self.detach_object("target")
        self.move_to([bx, by, place_z + LIFT])
        self.get_logger().info("pick & place DONE")


def main():
    rclpy.init()
    node = PickPlace()
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
