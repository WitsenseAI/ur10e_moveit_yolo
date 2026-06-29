#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# ---- EDIT to match your Isaac table (meters, expressed in the robot "world" frame) ----
FRAME_ID     = "world"
TABLE_SIZE   = (1.0, 1.0, 0.01)     # x=length, y=width, z=thickness
TABLE_CENTER = (0.55, 0.00, -0.02)    # center of the box (so top surface = z=0 here)

TOUCH_LINKS  = ["base_link", "base_link_inertia", "base"]
ATTACH_LINK  = "base_link"            # table is "attached" to the fixed base
# --------------------------------------------------------------------------------------

class AddTable(Node):
    def __init__(self):
        super().__init__("add_table_collision")
        cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        while not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for /apply_planning_scene ...")

        co = CollisionObject()
        co.header.frame_id = FRAME_ID
        co.id = "table"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(TABLE_SIZE))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = TABLE_CENTER
        pose.orientation.w = 1.0
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

       
        aco = AttachedCollisionObject()
        aco.link_name = ATTACH_LINK
        aco.object = co
        aco.touch_links = TOUCH_LINKS

        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request(scene=scene)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info("Table added â" if ok else "FAILED to add table")

def main():
    rclpy.init(); AddTable(); rclpy.shutdown()

if __name__ == "__main__":
    main()