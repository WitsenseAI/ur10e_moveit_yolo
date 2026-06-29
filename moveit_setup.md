# MoveIt Setup Assistant — UR10e + Robotiq 2F-140 (Isaac Sim)

Step-by-step for generating the `ur10e_moveit_config` package from
`ur10e_robot_description/urdf/ur10e_robotiq.urdf.xacro`.

This file documents **every screen** of the Setup Assistant and the exact values
to enter for this robot, plus what each screen actually generates and why.

---

## Robot cheat-sheet (used throughout)

| Thing | Value |
|---|---|
| Model file | `ur10e_robot_description/urdf/ur10e_robotiq.urdf.xacro` |
| Arm joints (6) | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| Arm base link | `base_link` |
| Arm tip link | `tool0` |
| Gripper driver joint | `finger_joint` (revolute, 0.0 = open … 0.695 = closed) |
| Gripper passive (mimic) joints | `left_inner_knuckle_joint`, `left_inner_finger_joint`, `right_outer_knuckle_joint`, `right_inner_knuckle_joint`, `right_inner_finger_joint` |
| Gripper mount link | `tool0` → `robotiq_base_link` |
| Isaac command topic | `/isaac_joint_commands` |
| Isaac state topic | `/isaac_joint_states` |
| Camera depth | `/front_stereo_camera/depth/ground_truth` |
| Camera frame | `front_stereo_camera_left` |

---

## 0. Before you launch (do this every time)

ROS Jazzy needs the system Python, **not** miniconda. In every ROS terminal:

```bash
conda deactivate                         # until "(base)" disappears
source /opt/ros/jazzy/setup.bash
source ~/witsense/moveit-ros/ros2_ws/install/setup.bash
```

Sanity check (both must print a path, and the model must parse):

```bash
ros2 pkg prefix ur_description robotiq_description
xacro ~/witsense/moveit-ros/ros2_ws/src/ur10e_robot_description/urdf/ur10e_robotiq.urdf.xacro | check_urdf /dev/stdin
```

Launch the assistant:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

---

## 1. Start

- Click **Create New MoveIt Configuration Package**.
- **Browse** → select `ur10e_robotiq.urdf.xacro` → **Load Files**.
- The combined UR10e + gripper should render in the viewport.

> If you get *"URDF/COLLADA file is not a valid robot model"*: `robotiq_description`
> isn't installed/sourced, or you didn't source the workspace. Fix the terminal
> (section 0) and relaunch.

---

## 2. Self-Collisions

Generates the **Allowed Collision Matrix (ACM)** — pairs of links MoveIt may skip
during collision checking (adjacent links, links that can never touch, etc.).

- **Sampling Density:** slide to **maximum** (it's a one-time offline computation;
  high density avoids wrongly disabling the many small gripper-finger pairs).
- Click **Generate Collision Matrix**.
- Skim the list: disabled pairs should be *adjacent* links or links that genuinely
  cannot reach each other. Leave the rest enabled.

Generates the `disable_collisions` entries in the SRDF.

---

## 3. Virtual Joints

Connects the robot's root link to an external world frame **when the URDF doesn't
already contain one**.

- **Skip this step.** Our URDF already defines a `world` link with a fixed joint to
  `base_link`, so no virtual joint is needed.
- *(Only if MoveIt later complains the robot isn't attached to a frame:* add one —
  Name `virtual_joint`, Child Link `base_link`, Parent Frame `world`, Type `fixed`.)*

---

## 4. Planning Groups

The most important step. Create **two** groups.

### Group A — `ur_manipulator` (the arm)
- Click **Add Group**.
- Group Name: `ur_manipulator`
- Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- Group Default Planner: `RRTConnectkConfigDefault` (OMPL)
- Click **Add Kin. Chain** → Base Link: `base_link`, Tip Link: `tool0` → **Save**.

> Tip link is `tool0` (the arm flange). The gripper length is handled by the
> end-effector definition. If you later want IK solved straight to the fingertips,
> add a fixed `tcp`/`grasp_frame` link in the xacro and use it as the tip.

### Group B — `gripper`
- Click **Add Group**.
- Group Name: `gripper`
- Kinematic Solver: `None` (a 1-DOF gripper doesn't need IK)
- Group Default Planner: leave default / `None`
- Click **Add Joints** → add **only `finger_joint`** → **Save**.
  (The 5 mimic joints follow automatically; do **not** add them here.)

Generates the `<group>` entries in the SRDF.

---

## 5. Robot Poses

Named joint targets you can command by name later (great for testing).

For **`ur_manipulator`**:
| Pose name | Joint values |
|---|---|
| `home` | `shoulder_pan=0`, `shoulder_lift=-1.5708`, `elbow=0`, `wrist_1=-1.5708`, `wrist_2=0`, `wrist_3=0` |
| `up` | all `0` (arm pointing straight up) |

For **`gripper`**:
| Pose name | `finger_joint` |
|---|---|
| `open` | `0.0` |
| `closed` | `0.695` |

Click **Save** after each pose. Generates `<group_state>` entries in the SRDF.

---

## 6. End Effectors

Tells MoveIt that `gripper` is an end effector attached to the arm — enables grasp
APIs and proper attach/detach of picked objects.

- Click **Add End Effector**.
- End Effector Name: `gripper_ee`
- End Effector Group: `gripper`
- Parent Link: `tool0`
- Parent Group: `ur_manipulator`
- **Save**.

Generates the `<end_effector>` entry in the SRDF.

---

## 7. Passive Joints

Joints MoveIt must **not** plan for (driven mechanically). Mark all 5 mimic joints:

- `left_inner_knuckle_joint`
- `left_inner_finger_joint`
- `right_outer_knuckle_joint`
- `right_inner_knuckle_joint`
- `right_inner_finger_joint`

Move them to the "Passive Joints" side and **Save**. Generates `<passive_joint>`
entries in the SRDF.

---

## 8. ros2_control URDF Modifications

This screen wants to add `<ros2_control>` command interfaces to the robot.

**Our URDF already defines `ros2_control` (the `UR10eArm` and `RobotiqGripper`
systems bridged to Isaac).** So:

- Confirm/select the **`position`** command interface for the 6 arm joints and
  `finger_joint`.
- MSA will still emit a `config/ros2_controllers.xacro` using `mock_components`
  for its built-in `demo.launch.py`. **We won't use that mock file** — we run the
  real Isaac bridge from `ur10e_robotiq.urdf.xacro` instead (see section 13).

You don't need to do anything special here beyond confirming `position` interfaces.

---

## 9. ROS 2 Controllers

These are the **real controllers** `controller_manager` will load. Add:

### `joint_state_broadcaster`
- Add Controller → Name `joint_state_broadcaster`,
  Type `joint_state_broadcaster/JointStateBroadcaster`. (No joints to add.)

### `arm_controller`
- Add Controller → Name `arm_controller`,
  Type `joint_trajectory_controller/JointTrajectoryController`.
- Add the **6 arm joints**.

### `gripper_controller`
- Add Controller → Name `gripper_controller`,
  Type `position_controllers/GripperActionController`.
- Add **`finger_joint`** only.

Generates `config/ros2_controllers.yaml`. Resulting file should look like:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_controller:
      type: position_controllers/GripperActionController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]

gripper_controller:
  ros__parameters:
    joint: finger_joint
```

---

## 10. MoveIt Controllers

This maps MoveIt's execution layer to the controllers from step 9 (via action
servers). Generate/define:

- **`arm_controller`** → action `follow_joint_trajectory`, type `FollowJointTrajectory`,
  the 6 arm joints, `default: true`.
- **`gripper_controller`** → action `gripper_cmd`, type `GripperCommand`,
  joint `finger_joint`, `default: true`.

Generates `config/moveit_controllers.yaml`:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

  gripper_controller:
    action_ns: gripper_cmd
    type: GripperCommand
    default: true
    joints:
      - finger_joint
```

> **Naming rule:** the controller names here, in `ros2_controllers.yaml`, and the
> names you `spawn` in the bringup launch **must all match** (`arm_controller`,
> `gripper_controller`). Mismatch = "controller not found" / no execution.

---

## 11. Perception (3D sensors) — optional but useful

Feeds the camera into MoveIt's **Octomap** so the planner avoids real obstacles on
the table. Your camera publishes a **depth image** (no PointCloud2), so use the
**Depth Map** updater.

In MSA choose **Point Cloud** isn't available without a PointCloud2 topic, so set
up the depth updater manually after generation. `config/sensors_3d.yaml`:

```yaml
sensors:
  - depth_camera
depth_camera:
  sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
  image_topic: /front_stereo_camera/depth/ground_truth
  queue_size: 5
  near_clipping_plane_distance: 0.3
  far_clipping_plane_distance: 5.0
  shadow_threshold: 0.2
  padding_scale: 1.0
  padding_offset: 0.03
  max_update_rate: 1.0
  filtered_cloud_topic: filtered_cloud
```

And add the octomap params to `move_group` (the generated launch already supports
`octomap_resolution`; otherwise add):

```yaml
octomap_frame: world
octomap_resolution: 0.02
max_range: 5.0
```

**Two requirements for this to work:**
1. The camera frame `front_stereo_camera_left` must be in the TF tree. Publish a
   static transform from `world` to the camera matching its Isaac placement
   (replace the numbers with your actual camera pose):
   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x 1.0 --y 0.0 --z 1.5 --roll 0 --pitch 1.2 --yaw 3.14 \
     --frame-id world --child-frame-id front_stereo_camera_left
   ```
2. *(Alternative if you prefer point clouds)* convert depth→PointCloud2 with
   `depth_image_proc point_cloud_xyz` and use `PointCloudOctomapUpdater` on
   `/front_stereo_camera/points` instead.

You can **skip perception for now** and add it before doing real pick-and-place.

---

## 12. Launch Files

Select which launch files MSA generates. Keep the defaults (all checked). The ones
that matter for us:

- `move_group.launch.py` — the planner/execution node (we use this).
- `moveit_rviz.launch.py` — RViz with MotionPlanning (we use this).
- `rsp.launch.py` — robot_state_publisher.
- `spawn_controllers.launch.py` — controller spawners.
- `demo.launch.py` — all-in-one with **mock** hardware (handy to verify planning in
  RViz before touching Isaac; does **not** move the Isaac robot).

---

## 13. Author Info → Generate

- Fill Name + Email (required to enable Generate).
- **Browse** to `~/witsense/moveit-ros/ros2_ws/src/` and create folder
  `ur10e_moveit_config`.
- Click **Generate Package**, then **Exit**.

Build it:

```bash
cd ~/witsense/moveit-ros/ros2_ws
colcon build --packages-select ur10e_moveit_config
source install/setup.bash
```

---

## 14. After generation — Isaac wiring (important)

The generated `demo.launch.py` uses **mock hardware**. To drive the **Isaac** robot
you separate concerns:

1. **Control layer (your bringup)** — robot_state_publisher + `controller_manager`
   loading `ur10e_robotiq.urdf.xacro` (the real Isaac `ros2_control`), spawning
   `joint_state_broadcaster`, `arm_controller`, `gripper_controller`. This replaces
   the mock `ros2_controllers.xacro`.
2. **Planning layer** — `move_group.launch.py` + `moveit_rviz.launch.py` from
   `ur10e_moveit_config`.

**Everything must use sim time** (Isaac publishes `/clock`): pass
`use_sim_time:=true` to bringup, move_group, and RViz.
