# ur10e_moveit_yolo

A complete **pick-and-place pipeline** for a **UR10e arm + Robotiq 2F-140 gripper**
simulated in **NVIDIA Isaac Sim**, driven by **ROS 2 Jazzy + MoveIt 2**, with
**YOLO** object detection from an overhead camera.

The robot detects a cup/ball on a table, deprojects its pixel into a 3D world
pose, plans a collision-free motion with MoveIt, grasps the object, and places
it in a bin.

> **Purpose:** this repo is a learning project for ROS 2, MoveIt and object
> detection. It deliberately uses **only official ROS 2 / MoveIt packages** (no
> `pymoveit2` or other wrappers) so you can see exactly how the action/service
> interfaces work underneath.

---

## Architecture

```
            ┌──────────────────────────────────────────────────────────┐
            │                      Isaac Sim                            │
            │  UR10e + Robotiq 2F-140, overhead RGB-D camera            │
            │  pub: /isaac_joint_states, /clock, /front_stereo_camera/* │
            │  sub: /isaac_joint_commands                               │
            └───────────────┬───────────────────────────┬──────────────┘
                            │ joint states / commands    │ RGB + depth + info
                            ▼                            ▼
   ┌─────────────────────────────────┐   ┌────────────────────────────────┐
   │ ros2_control (topic_based)      │   │ object_detection_yolo           │
   │ arm_controller, gripper_ctrl,   │   │ yolo_3d_detector                │
   │ joint_state_broadcaster         │   │  -> /detected_objects_3d        │
   └───────────────┬─────────────────┘   └────────────────┬───────────────┘
                   │ FollowJointTrajectory                 │ Detection3DArray
                   ▼                                       ▼
   ┌─────────────────────────────────┐   ┌────────────────────────────────┐
   │ MoveIt move_group               │◄──┤ pick_place                      │
   │ planning, /move_action,         │   │  MoveGroup action +             │
   │ /apply_planning_scene           │   │  FollowJointTrajectory (gripper)│
   └─────────────────────────────────┘   └────────────────────────────────┘
```

## Repository layout

```
.
├── README.md                  # you are here
├── requirements.txt           # YOLO/torch venv (see Setup)
├── moveit_setup.md            # guide: building the MoveIt config from scratch
├── object_detection.md        # guide: the YOLO 3D detector
├── object_pick_place.md       # guide: the pick-and-place node
├── ur10e_moveit.usd           # Isaac scene reference
└── ros2_ws/
    ├── source_all.sh          # sources base ROS + MoveIt + this workspace
    ├── yolo26_cup.pt          # custom-trained cup detector (committed)
    └── src/
        ├── ur10e_robot_description/  # URDF/xacro, meshes, Isaac robot USD
        ├── ur10e_moveit_config/      # MoveIt config (SRDF, controllers, limits)
        ├── ur10e_isaac_bringup/      # ros2_control bringup + all-in-one launch
        ├── object_detection_yolo/    # YOLO detector + pick_place nodes
        └── add_table.py              # helper: add the table to the planning scene
```

---

## Prerequisites

| Component | Version / note |
|-----------|----------------|
| Ubuntu | 24.04 |
| ROS 2 | **Jazzy** |
| MoveIt 2 | built/installed at `~/ws_moveit` (override with `MOVEIT_WS=...`) |
| Isaac Sim | 4.x, with the UR10e scene publishing `/isaac_joint_states`, `/clock`, `/front_stereo_camera/*` |
| Python | 3.12 (the system interpreter ROS Jazzy uses) |

### apt dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ur-description \
  ros-jazzy-robotiq-description \
  ros-jazzy-topic-based-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro
# MoveIt is expected as a source overlay at ~/ws_moveit. If you use the apt
# binary instead, install ros-jazzy-moveit and edit source_all.sh accordingly.
```

---

## Setup

```bash
# 1) clone
git clone https://github.com/WitsenseAI/ur10e_moveit_yolo.git
cd ur10e_moveit_yolo

# 2) Python venv for YOLO (kept separate from system python on purpose)
python3 -m venv --system-site-packages .yolo_moveit
source .yolo_moveit/bin/activate
pip install -r requirements.txt
deactivate

# 3) build the ROS 2 workspace
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash         # your MoveIt overlay
colcon build --symlink-install
```

> ⚠️ **Do not** have a conda env or the YOLO venv *active* when running ROS nodes —
> they shadow `/usr/bin/python3` (which has `rclpy`). The detector reaches its
> venv through `PYTHONPATH` inside its launch file, not by activation.
> `source_all.sh` warns you if a conda/venv is active.

---

## Running

In every new terminal, first:

```bash
source ros2_ws/source_all.sh
```

**Start Isaac Sim** with the UR10e scene playing, then launch everything at once:

```bash
ros2 launch ur10e_isaac_bringup system.launch.py
```

This brings up the control bridge + controllers, `move_group`, RViz, and the YOLO
detector. The pick node is optional (`pick:=true`).

### Or run the stack piece by piece (useful while learning)

```bash
# 1) control bridge + controllers
ros2 launch ur10e_isaac_bringup bringup.launch.py

# 2) MoveIt
ros2 launch ur10e_moveit_config move_group.launch.py use_sim_time:=true

# 3) RViz
ros2 launch ur10e_moveit_config moveit_rviz.launch.py use_sim_time:=true

# 4) detection
ros2 launch object_detection_yolo detection.launch.py

# 5) (optional) the table in the planning scene, then pick-and-place
python3 ros2_ws/src/add_table.py
ros2 launch object_detection_yolo pick_place.launch.py
```

In RViz (**Displays → MotionPlanning**): set the planning group to `ur_manipulator`,
drag the orange marker to a goal, **Plan**, then **Plan & Execute** — the arm moves
in Isaac. Switch the group to `gripper` to test open/close.

---

## Learning guides

These walk through *how* each part was built, not just how to run it:

- [`moveit_setup.md`](moveit_setup.md) — importing the URDF and every step of the MoveIt Setup Assistant.
- [`object_detection.md`](object_detection.md) — the YOLO 3D detector (deprojection, TF, depth caching).
- [`object_pick_place.md`](object_pick_place.md) — the pick-and-place node using raw MoveIt actions/services.

---

## Troubleshooting / known limitations

- **`START_STATE_IN_COLLISION`** — MoveIt cannot plan from a start state already in
  collision. The table box must be sized to the *real* table, not a giant slab that
  engulfs the arm. Lift the arm clear of any collision before planning.
- **Detection Z looks wrong / marker doesn't sit on the object** — the camera's
  static TF (`world → front_stereo_camera_left`) must match the camera pose in
  Isaac exactly. Make sure only **one** publisher provides that transform (the
  static one *or* Isaac, not both).
- **TF "extrapolation into the past"** — Isaac image stamps are out of sync with
  `/clock`; the detector looks up transforms at `Time(0)` (latest) because the
  camera is static.
- **Grasp doesn't physically hold in sim** — MoveIt's attach is *kinematic only*;
  PhysX won't grip unless you add a fixed joint / surface gripper in Isaac.

---

## License

Apache-2.0 — see [`LICENSE`](LICENSE).
