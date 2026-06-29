# Pick & Place — detected object → bin (UR10e + Robotiq, Isaac Sim)

Goal: take the 3D object pose from `object_detection_yolo` (`/detected_objects_3d`,
frame `world`), pick it, and drop it in the **bin frame you publish in TF**.

Sequence:
```
open gripper
 → pre-grasp  (above object, gripper pointing down)
 → grasp      (descend onto object)
 → close gripper  → attach object to tool0
 → lift
 → pre-place  (above bin)
 → place      (descend into bin)
 → open gripper   → detach object
 → retreat → home
```

We use **pymoveit2** for the arm (pose goals + collision objects, talking to your
running `move_group`), and a direct **FollowJointTrajectory** action for the gripper.

---

## Key facts / params (for this robot)

| Thing | Value |
|---|---|
| Arm group | `ur_manipulator` (tip `tool0`, base `base_link`) |
| Arm joints | `shoulder_pan_joint … wrist_3_joint` |
| Gripper action | `/gripper_controller/follow_joint_trajectory` (joint `finger_joint`) |
| Gripper open / close | `0.0` / ~`0.6` (full close 0.695; use a bit less to grip) |
| Detections topic | `/detected_objects_3d` (`vision_msgs/Detection3DArray`, frame `world`) |
| Object class ids | `32` sports ball, `41` cup |
| Bin frame | **`bin`** ← change to your actual TF frame name |
| Planning frame | `world` |
| `TCP_OFFSET` | ~`0.16` m (tool0 above grasp point) — **measure** (Step 1) |
| Top-down orientation | `quat_xyzw = [1, 0, 0, 0]` (tune in Step 1) |

---

## Step 0 — Install pymoveit2

```bash
cd ~/witsense/moveit-ros/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/witsense/moveit-ros/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pymoveit2
source install/setup.bash
python3 -c "import pymoveit2; print('pymoveit2 ok')"
```

---

## Step 1 — Measure the two things you must tune

Bring up the robot (bringup + move_group) first, then:

**a) TCP offset** — distance from `tool0` to the grasp point (between the fingertips),
with the gripper **open**:
```bash
ros2 run tf2_ros tf2_echo tool0 left_inner_finger_pad
ros2 run tf2_ros tf2_echo tool0 right_inner_finger_pad
```
The grasp point is midway between the two pads. Use the translation magnitude as
`TCP_OFFSET` (≈ 0.16 m). If the gripper stops short of / crashes into the object later,
nudge this value.

**b) Top-down orientation** — in RViz, drag the arm so the gripper points straight
**down** at the table, then read `tool0`'s orientation:
```bash
ros2 run tf2_ros tf2_echo world tool0      # copy the quaternion (x y z w)
```
Use that as `GRASP_QUAT` (`[x, y, z, w]`). `[1, 0, 0, 0]` is the usual top-down value
for a UR; tune if the gripper tilts.

**c) Bin frame** — confirm your bin TF resolves and note its name:
```bash
ros2 run tf2_ros tf2_echo world bin        # replace 'bin' with your frame
```

---

## Step 2 — Package deps

Add to `object_detection_yolo/package.xml`:
```xml
<exec_depend>pymoveit2</exec_depend>
<exec_depend>control_msgs</exec_depend>
<exec_depend>trajectory_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
<exec_depend>vision_msgs</exec_depend>
```

---

## Step 3 — The pick-and-place node

Create `object_detection_yolo/object_detection_yolo/pick_place.py`:

```python
#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from builtin_interfaces.msg import Duration
from vision_msgs.msg import Detection3DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pymoveit2 import MoveIt2

# ---------------- TUNE THESE (see object_pick_place.md Step 1) ----------------
ARM_JOINTS   = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TCP_OFFSET   = 0.16            # tool0 above the grasp point (m)
APPROACH     = 0.12            # pre-grasp height above grasp (m)
LIFT         = 0.20            # how high to lift after grasping (m)
GRASP_QUAT   = [1.0, 0.0, 0.0, 0.0]   # gripper pointing down (x,y,z,w)
GRIP_OPEN    = 0.0
GRIP_CLOSE   = 0.6
TARGET_CLASS = "cup"           # "cup" or "sports ball"
BIN_FRAME    = "bin"           # your bin TF frame
PLACE_CLEAR  = 0.15            # height above the bin to release (m)
# -----------------------------------------------------------------------------


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.cb = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ur_manipulator",
            callback_group=self.cb,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        self.grip = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory", callback_group=self.cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._latest = {}   # class_id -> (x,y,z)
        self.create_subscription(Detection3DArray, "/detected_objects_3d",
                                 self.det_cb, 10, callback_group=self.cb)

    # ---------- helpers ----------
    def det_cb(self, msg: Detection3DArray):
        for d in msg.detections:
            if not d.results:
                continue
            name = d.results[0].hypothesis.class_id
            p = d.results[0].pose.pose.position
            self._latest[name] = (p.x, p.y, p.z)

    def move_to(self, xyz, quat=GRASP_QUAT, cartesian=False):
        self.get_logger().info(f"move_to {('%.3f '*3) % tuple(xyz)} cart={cartesian}")
        self.moveit2.move_to_pose(position=list(xyz), quat_xyzw=quat, cartesian=cartesian)
        return self.moveit2.wait_until_executed()

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

    def lookup_bin(self):
        for _ in range(20):
            try:
                t = self.tf_buffer.lookup_transform("world", BIN_FRAME, rclpy.time.Time())
                tr = t.transform.translation
                return (tr.x, tr.y, tr.z)
            except Exception:
                self.get_logger().warn(f"waiting for TF world->{BIN_FRAME} ...")
                threading.Event().wait(0.5)   # executor spins in the bg thread
        return None

    def wait_for_object(self):
        for _ in range(60):
            xyz = self._latest.get(TARGET_CLASS)
            if xyz is not None:
                return xyz
            threading.Event().wait(0.5)
        return None

    # ---------- main sequence ----------
    def run(self):
        obj = self.wait_for_object()
        if obj is None:
            self.get_logger().error(f"no '{TARGET_CLASS}' detected"); return
        bin_xyz = self.lookup_bin()
        if bin_xyz is None:
            self.get_logger().error(f"bin frame '{BIN_FRAME}' not in TF"); return

        ox, oy, oz = obj
        grasp_z = oz + TCP_OFFSET
        self.get_logger().info(f"object @ {obj}, bin @ {bin_xyz}")

        # PICK
        self.set_gripper(GRIP_OPEN)
        if not self.move_to([ox, oy, grasp_z + APPROACH]): return
        if not self.move_to([ox, oy, grasp_z], cartesian=True): return
        self.set_gripper(GRIP_CLOSE)

        # attach a box so MoveIt knows the carried object (collision-aware transport)
        self.moveit2.add_collision_box(
            id="target", size=[0.06, 0.06, 0.10], position=[ox, oy, oz],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0])
        self.moveit2.attach_collision_object(
            id="target", link_name="tool0",
            touch_links=["left_inner_finger_pad", "right_inner_finger_pad",
                         "left_inner_finger", "right_inner_finger"])

        if not self.move_to([ox, oy, grasp_z + LIFT], cartesian=True): return

        # PLACE
        bx, by, bz = bin_xyz
        place_z = bz + TCP_OFFSET + PLACE_CLEAR
        if not self.move_to([bx, by, place_z + APPROACH]): return
        if not self.move_to([bx, by, place_z], cartesian=True): return
        self.set_gripper(GRIP_OPEN)
        self.moveit2.detach_collision_object("target")
        self.moveit2.remove_collision_object("target")
        self.move_to([bx, by, place_z + LIFT], cartesian=True)
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
```

> Note: `move_to_pose`/`add_collision_box`/`attach_collision_object` signatures can
> differ slightly between pymoveit2 versions — if Python complains about an argument,
> check `pymoveit2/examples/ex_pose_goal.py` and `ex_collision_primitive.py` in the
> cloned repo and adjust.

---

## Step 4 — Register node + launch

`setup.py` entry point:
```python
'pick_place = object_detection_yolo.pick_place:main',
```

Create `object_detection_yolo/launch/pick_place.launch.py` (it must pass the MoveIt
config so pymoveit2/move_group params resolve):
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e", package_name="ur10e_moveit_config").to_moveit_configs()

    pick = Node(
        package="object_detection_yolo", executable="pick_place",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )
    return LaunchDescription([pick])
```

Build:
```bash
colcon build --packages-select object_detection_yolo
source install/setup.bash
```

---

## Step 5 — Run order

Each in its own sourced terminal (conda off):

```bash
# 0) Isaac Sim playing
# 1) controllers
ros2 launch ur10e_isaac_bringup bringup.launch.py
# 2) planner (pymoveit2 sends goals to this move_group -> shared scene)
ros2 launch ur10e_moveit_config move_group.launch.py
# 3) table into the scene (and your bin TF publisher must be running)
python3 src/add_table.py
# 4) detection (venv on PYTHONPATH)
ros2 launch object_detection_yolo detection.launch.py
# 5) pick & place
ros2 launch object_detection_yolo pick_place.launch.py
```

Watch the `pick_place` logs: `object @ ...`, `bin @ ...`, then each `move_to`, then
`pick & place DONE`.

---

## Tuning & troubleshooting

| Symptom | Fix |
|---|---|
| Gripper stops above / crashes into object | adjust `TCP_OFFSET` (Step 1a) |
| Gripper tilts, not straight down | retune `GRASP_QUAT` (Step 1b) |
| `no 'cup' detected` | detection not running / wrong `TARGET_CLASS` / lower YOLO `conf` |
| `bin frame 'bin' not in TF` | wrong `BIN_FRAME`, or your bin TF publisher isn't running |
| Plan fails near object | the object/table collision blocks IK — raise `APPROACH`, or shrink the attached box, or confirm the table box isn't swallowing the target |
| Object slips in Isaac after close | sim grasp physics — increase finger/ object friction in Isaac, close a bit more (`GRIP_CLOSE`), or add a fixed/surface-gripper joint on contact. MoveIt's *attach* only makes planning collision-aware; it doesn't create physical force |
| Arm executes but also RViz fights it | don't drive RViz goals while pick_place runs (both use the same controllers) |

---

## Next ideas
- Trigger the pick from a service/topic instead of running once at startup.
- Pick the **closest** detected object, or loop over all detections.
- Replace the fixed top-down grasp with a grasp pose derived from the object's shape
  (e.g. orient the gripper across a cup handle).
