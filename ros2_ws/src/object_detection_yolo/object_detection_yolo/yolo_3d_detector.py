import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # registers PointStamped transforms (must be imported)
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO


class Yolo3DDetector(Node):
    def __init__(self):
        super().__init__("yolo_3d_detector")

        # --- topics / frames ---
        self.declare_parameter("rgb_topic", "/front_stereo_camera/rgb")
        self.declare_parameter("depth_topic", "/front_stereo_camera/depth")
        self.declare_parameter("info_topic", "/front_stereo_camera/camera_info")
        self.declare_parameter("target_frame", "world")
        # The optical child we publish off Isaac's USD camera frame (see launch file).
        self.declare_parameter("optical_frame", "front_stereo_camera_left_optical")

        # --- model / detection ---
        self.declare_parameter("model", "yolo26l.pt")
        # COCO ids: 41 = cup, 32 = sports ball. [-1] = no filter (all classes).
        self.declare_parameter("classes", [41])
        self.declare_parameter("conf", 0.4)

        # --- DEBUG TOGGLES (set false once everything works) ---
        # detect_all: ignore class filter + use a low conf, and log every class found.
        self.declare_parameter("detect_all", False)
        self.declare_parameter("debug_conf", 0.10)
        # dump_frames: save the first N frames YOLO actually sees to /tmp for inspection.
        self.declare_parameter("dump_frames", 5)

        self.target_frame = self.get_parameter("target_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.conf = float(self.get_parameter("conf").value)
        cls = list(self.get_parameter("classes").value)
        self.classes = None if cls == [-1] else cls

        self.detect_all = bool(self.get_parameter("detect_all").value)
        self.debug_conf = float(self.get_parameter("debug_conf").value)
        self.dump_frames = int(self.get_parameter("dump_frames").value)
        self._dumped = 0

        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter("model").value)
        self.K = None             # fx, fy, cx, cy
        self.latest_depth = None  # cached 32FC1 depth image (meters)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.det_pub = self.create_publisher(Detection3DArray, "/detected_objects_3d", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "/detected_objects_markers", 10)
        # annotated RGB so you can SEE what YOLO boxes (view with rqt_image_view)
        self.dbg_pub = self.create_publisher(Image, "/yolo_3d_detector/debug_image", 10)

        # Decoupled subscriptions: RGB and depth publish at different rates / stamps
        # in Isaac, so a strict time-sync rarely matches. The scene is static, so we
        # just cache the latest depth and run detection on every RGB frame.
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self.info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.depth_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("rgb_topic").value, self.rgb_cb,
            qos_profile_sensor_data)

        self.get_logger().info(
            f"yolo_3d_detector ready | detect_all={self.detect_all} "
            f"conf={self.debug_conf if self.detect_all else self.conf} "
            f"optical_frame={self.optical_frame}")

    def info_cb(self, msg: CameraInfo):
        k = msg.k
        self.K = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    def depth_cb(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def sample_depth(self, depth, x1, y1, x2, y2):
        # The object is the NEAREST surface inside its box; the floor/background
        # behind it is farther. A median at the box center catches the table seen
        # through/past the cup's rim (symptom: world z pinned to the table plane).
        # Taking a NEAR percentile over the box interior locks onto the object.
        h, w = depth.shape
        bw, bh = x2 - x1, y2 - y1
        # shrink to the central 60% so background at the box edges doesn't bleed in
        u0 = max(0, int(x1 + 0.2 * bw)); u1 = min(w, int(x2 - 0.2 * bw))
        v0 = max(0, int(y1 + 0.2 * bh)); v1 = min(h, int(y2 - 0.2 * bh))
        patch = depth[v0:v1, u0:u1].astype(np.float32).ravel()
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0.05]          # drop zeros/holes
        if patch.size == 0:
            return None
        return float(np.percentile(patch, 20))   # near surface = the object, not the floor

    def rgb_cb(self, rgb_msg: Image):
        if self.K is None or self.latest_depth is None:
            return
        fx, fy, cx, cy = self.K
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.latest_depth

        # --- DEBUG: dump the exact frame YOLO sees + its stats ---
        if self._dumped < self.dump_frames:
            path = f"/tmp/yolo_frame_{self._dumped}.png"
            cv2.imwrite(path, bgr)
            self.get_logger().info(
                f"saved {path}  shape={bgr.shape} dtype={bgr.dtype} "
                f"min={int(bgr.min())} max={int(bgr.max())} mean={bgr.mean():.1f}")
            self._dumped += 1

        # If depth is a different resolution than RGB, scale pixel coords here.
        sx = depth.shape[1] / bgr.shape[1]
        sy = depth.shape[0] / bgr.shape[0]

        if self.detect_all:
            results = self.model.predict(bgr, conf=self.debug_conf, verbose=False)[0]
        else:
            results = self.model.predict(
                bgr, conf=self.conf, classes=self.classes, verbose=False)[0]

        self.get_logger().info(f"YOLO boxes: {len(results.boxes)}")
        if self.detect_all:
            for b in results.boxes:
                cid = int(b.cls[0])
                self.get_logger().info(
                    f"  cls={cid} {self.model.names.get(cid, cid)} "
                    f"conf={float(b.conf[0]):.2f}")

        det_array = Detection3DArray()
        det_array.header.frame_id = self.target_frame
        det_array.header.stamp = self.get_clock().now().to_msg()  # node clock, not the (bad) image stamp
        markers = MarkerArray()
        mid = 0

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int((x1 + x2) / 2); v = int((y1 + y2) / 2)
            cls_id = int(box.cls[0]); conf = float(box.conf[0])
            name = self.model.names.get(cls_id, str(cls_id))

            # draw the 2D box + label so we can see exactly what YOLO fired on
            cv2.rectangle(bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(bgr, f"{name} {conf:.2f}", (int(x1), max(0, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            z = self.sample_depth(depth, x1 * sx, y1 * sy, x2 * sx, y2 * sy)
            if z is None:
                self.get_logger().warn(f"{name}: no valid depth at pixel")
                continue

            # deproject (optical frame: x right, y down, z forward)
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy

            p_cam = PointStamped()
            p_cam.header.frame_id = self.optical_frame  # NOT rgb_msg.header.frame_id
            # Time(0) = latest available transform. Camera stamps are out of sync with
            # /clock in sim and the camera is static, so latest is correct and avoids
            # "extrapolation into the past".
            p_cam.header.stamp = rclpy.time.Time().to_msg()
            p_cam.point.x, p_cam.point.y, p_cam.point.z = X, Y, z

            try:
                p_world = self.tf_buffer.transform(
                    p_cam, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception as e:
                self.get_logger().warn(f"TF failed for {name}: {e}")
                continue

            self.get_logger().info(
                f"{name} ({conf:.2f}) cam[{X:.3f},{Y:.3f},{z:.3f}] -> "
                f"world[{p_world.point.x:.3f},{p_world.point.y:.3f},{p_world.point.z:.3f}]")

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
        # publish the annotated frame (boxes drawn above) for visual debugging
        self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(bgr, "bgr8"))


def main():
    rclpy.init()
    node = Yolo3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()