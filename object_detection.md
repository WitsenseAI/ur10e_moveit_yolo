# Object Detection → 3D Pose → Pick (UR10e + Robotiq, Isaac Sim)

Goal: detect an object on the table from the overhead camera, compute its **3D pose
in the robot frame**, drop it into the MoveIt planning scene, and pick it.

Pipeline:
```
RGB ──► YOLO (2D box/class) ─┐
                             ├─► pixel (u,v) + depth(u,v) ──► deproject with K ──► 3D point (camera optical frame)
Depth (32FC1, meters) ───────┘                                                        │
camera_info (fx,fy,cx,cy) ───────────────────────────────────────────────────────────┘
                                                                                       ▼
                                                              TF: camera_optical → world ──► 3D pose in "world"
                                                                                       ▼
                                                  add as collision object  +  grasp pose  ──► MoveIt pick
```

## Camera cheat-sheet (from your running sim)

| Thing | Value |
|---|---|
| RGB topic | `/front_stereo_camera/rgb` (`rgb8`, 1920×1200) |
| Depth topic | `/front_stereo_camera/depth` (`32FC1`, **meters**) |
| camera_info | `/front_stereo_camera/camera_info` |
| Intrinsics | `fx=fy=957.81`, `cx=960.0`, `cy=600.0` |
| Camera frame | `front_stereo_camera_left` |
| Target/planning frame | `world` |

---

## Step 0 — Install YOLO (one time)

`ultralytics` + `torch` are missing. Install into the **ROS Python** (`/usr/bin/python3`),
not conda:

```bash
conda deactivate
/usr/bin/python3 -m pip install --user ultralytics
# if Ubuntu 24.04 blocks it (externally-managed):
# /usr/bin/python3 -m pip install --user --break-system-packages ultralytics
```\

This pulls a **CPU** torch by default — good, because Isaac is already using your GPU.
Verify:
```bash
/usr/bin/python3 -c "import ultralytics, torch; print(ultralytics.__version__, torch.__version__)"
```

We'll use `yolov8n.pt` (COCO pretrained, auto-downloads on first run — needs internet
once). COCO detects common objects (bottle, cup, bowl…). For your own sim objects you'll
later train a custom model, but start with COCO + a known object.

---

## Step 1 — Publish the camera TF (CRITICAL — it's missing)

Right now `/tf` only has robot frames; `world → front_stereo_camera_left` does **not**
resolve, so detections can't be put in the robot frame. You must publish it.

The deprojection math produces a point in the **ROS optical convention**
(x-right, y-down, z-forward-into-scene). So the `front_stereo_camera_left` frame must be
an **optical frame**. For a camera mounted above the table looking straight **down**,
the optical frame is the world rotated 180° about X (`roll = π`): z points down, x right.

Get the camera **position** (x y z) from Isaac (select the camera prim → Transform →
Translate, in meters relative to the world origin = robot base), then publish:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <CAM_X> --y <CAM_Y> --z <CAM_Z> \
  --roll 3.14159 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id front_stereo_camera_left
```

Verify it resolves:
```bash
ros2 run tf2_ros tf2_echo world front_stereo_camera_left
```

> **Tuning:** roll=π assumes a perfect top-down camera with image-right = world +X. If
> your detections land rotated/mirrored on the table, adjust `--yaw` (try `1.5708`,
> `3.14159`, `-1.5708`) until the published marker (Step 6) sits on the real object. If
> the camera is **tilted** (not straight down), read the camera prim's full orientation
> (quaternion) from Isaac and pass `--qx --qy --qz --qw` instead of roll/pitch/yaw —
> but remember it must represent the *optical* frame.
>
> **Cleaner alternative:** add a `ROS2 Publish Transform Tree` node in the Isaac Action
> Graph (parent = world, target = camera prim) so Isaac publishes the pose automatically.
> You may still need a body→optical static rotation since Isaac cameras use the USD
> convention (−z forward).

Put the static publisher in your launch file (Step 5) so you don't type it each time.

---

## Step 2 — Package dependencies

Add to `object_detection_yolo/package.xml` (before `</package>`):

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>vision_msgs</exec_depend>
<exec_depend>visualization_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>image_geometry</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
<exec_depend>message_filters</exec_depend>
```

---

## Step 3 — The detection + 3D pose node

Create `object_detection_yolo/object_detection_yolo/yolo_3d_detector.py`:

```python
#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs  # registers PointStamped transforms
from ultralytics import YOLO


class Yolo3DDetector(Node):
    def __init__(self):
        super().__init__("yolo_3d_detector")

        # ---- params ----
        self.declare_parameter("rgb_topic", "/front_stereo_camera/rgb")
        self.declare_parameter("depth_topic", "/front_stereo_camera/depth")
        self.declare_parameter("info_topic", "/front_stereo_camera/camera_info")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("classes", [39, 41, 45])  # COCO: bottle, cup, bowl ([] = all)

        self.target_frame = self.get_parameter("target_frame").value
        self.conf = float(self.get_parameter("conf").value)
        cls = list(self.get_parameter("classes").value)
        self.classes = cls if len(cls) else None

        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter("model").value)
        self.K = None  # fx, fy, cx, cy

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(Detection3DArray, "/detected_objects_3d", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "/detected_objects_markers", 10)

        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self.info_cb,
            qos_profile_sensor_data)

        rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("rgb_topic").value, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter("depth_topic").value, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.sync.registerCallback(self.cb)

        self.get_logger().info("yolo_3d_detector ready")

    def info_cb(self, msg: CameraInfo):
        k = msg.k
        self.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def sample_depth(self, depth, u, v, win=4):
        h, w = depth.shape
        u0, u1 = max(0, u - win), min(w, u + win + 1)
        v0, v1 = max(0, v - win), min(h, v + win + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32).ravel()
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0.05]          # drop zeros/holes
        return float(np.median(patch)) if patch.size else None

    def cb(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            return
        fx, fy, cx, cy = self.K
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        # If depth is a different resolution than RGB, scale pixel coords here.
        sx = depth.shape[1] / bgr.shape[1]
        sy = depth.shape[0] / bgr.shape[0]

        results = self.model.predict(bgr, conf=self.conf, classes=self.classes, verbose=False)[0]

        det_array = Detection3DArray()
        det_array.header.frame_id = self.target_frame
        det_array.header.stamp = rgb_msg.header.stamp
        markers = MarkerArray()
        mid = 0

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int((x1 + x2) / 2); v = int((y1 + y2) / 2)
            cls_id = int(box.cls[0]); conf = float(box.conf[0])
            name = self.model.names.get(cls_id, str(cls_id))

            z = self.sample_depth(depth, int(u * sx), int(v * sy))
            if z is None:
                continue

            # deproject (optical frame: x right, y down, z forward)
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            p_cam = PointStamped()
            p_cam.header.frame_id = rgb_msg.header.frame_id  # front_stereo_camera_left
            p_cam.header.stamp = rgb_msg.header.stamp
            p_cam.point.x, p_cam.point.y, p_cam.point.z = X, Y, z

            try:
                p_world = self.tf_buffer.transform(
                    p_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception as e:
                self.get_logger().warn(f"TF failed for {name}: {e}")
                continue

            self.get_logger().info(
                f"{name} ({conf:.2f}) -> world "
                f"[{p_world.point.x:.3f}, {p_world.point.y:.3f}, {p_world.point.z:.3f}]")

            det = Detection3D()
            det.header = det_array.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            hyp.pose.pose.position = p_world.point
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center.position = p_world.point
            det.bbox.size.x = det.bbox.size.y = det.bbox.size.z = 0.05
            det_array.detections.append(det)

            m = Marker()
            m.header = det_array.header
            m.ns = "objects"; m.id = mid; mid += 1
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = p_world.point; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 0.9
            markers.markers.append(m)

        self.det_pub.publish(det_array)
        self.mk_pub.publish(markers)


def main():
    rclpy.init()
    node = Yolo3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Step 4 — Register the node (`setup.py`)

In `object_detection_yolo/setup.py`, add the entry point:

```python
    entry_points={
        'console_scripts': [
            'yolo_3d_detector = object_detection_yolo.yolo_3d_detector:main',
        ],
    },
```

---

## Step 5 — Launch file

Create `object_detection_yolo/launch/detection.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # EDIT CAM_X/Y/Z (and yaw) to your Isaac camera (see object_detection.md Step 1)
    cam_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=[
            "--x", "0.6", "--y", "0.0", "--z", "1.5",
            "--roll", "3.14159", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "world", "--child-frame-id", "front_stereo_camera_left",
        ],
    )
    detector = Node(
        package="object_detection_yolo", executable="yolo_3d_detector",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([cam_tf, detector])
```

Add to `setup.py` `data_files` so the launch installs:
```python
import os
from glob import glob
# ...
        data_files=[
            # keep the existing entries, then add:
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ],
```

---

## Step 6 — Build, run, verify

```bash
cd ~/witsense/moveit-ros/ros2_ws
colcon build --packages-select object_detection_yolo
source install/setup.bash
ros2 launch object_detection_yolo detection.launch.py
```

Checks:
- Node logs `name -> world [x, y, z]` for each detected object.
- `ros2 topic echo /detected_objects_3d` shows poses in `world`.
- In RViz add a **MarkerArray** display on `/detected_objects_markers` (Fixed Frame `world`).
  The red sphere should sit **on the real object** in the scene.
- If the sphere is offset/rotated/mirrored → fix the camera TF (Step 1 yaw / orientation).
  Sanity: the sphere's `z` should be ~the table height, `x,y` over the object.

---

## Step 7 — Use the detection (collision object + pick)

Once the world pose is correct:

1. **Add the object to the planning scene** as a collision object (same mechanism as
   `add_table.py`, with the detected `x,y,z`), so MoveIt avoids it on approach and you
   can attach it on grasp.
2. **Build a grasp pose**: position = object xyz with a small z offset; orientation =
   gripper pointing **down** (tool0 z aligned with −world z). Add a pre-grasp pose
   ~10 cm above.
3. **Pick sequence**: open gripper → move to pre-grasp → move to grasp → close gripper →
   attach object to `tool0` → lift. We'll write this as a small MoveIt node next.

Tell me when the marker lands correctly on the object and we'll do the pick node.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `TF failed ... lookup would require extrapolation` | camera TF not published or `use_sim_time` mismatch — ensure Step 1 publisher is running and node has `use_sim_time:=true` |
| Marker offset / mirrored on table | wrong camera TF orientation — tune `--yaw` / use real quaternion (Step 1) |
| `z is None` / no detections deprojected | depth holes at center — increase `win` in `sample_depth`, or object out of `conf` |
| No detections at all | COCO model doesn't know your object — lower `conf`, set `classes: []`, or train custom |
| Depth pixel misaligned with RGB | depth resolution ≠ RGB — the `sx,sy` scaling handles it; verify with `ros2 topic echo /front_stereo_camera/depth --field height --once` |
| Slow / GPU OOM | YOLO on CPU (default) is fine; keep camera resolution/rate modest to protect Isaac's GPU budget |
