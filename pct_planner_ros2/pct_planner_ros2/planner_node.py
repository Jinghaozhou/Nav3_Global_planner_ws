import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _repo_root():
    return os.environ.get(
        "PCT_PLANNER_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")),
    )


ROOT = _repo_root()
sys.path.insert(0, os.path.join(ROOT, "planner", "scripts"))
sys.path.insert(0, os.path.join(ROOT, "planner", "lib"))

from config import Config  # noqa: E402
from planner_wrapper import TomogramPlanner  # noqa: E402


class PlannerNode(Node):
    def __init__(self):
        super().__init__("pct_planner")
        self.declare_parameter("tomogram", "building2_9")
        self.declare_parameter("start_x", 5.0)
        self.declare_parameter("start_y", 5.0)
        self.declare_parameter("goal_x", -6.0)
        self.declare_parameter("goal_y", -1.0)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self._planned = False
        self.create_timer(0.1, self.plan_once)

    def plan_once(self):
        if self._planned:
            return
        self._planned = True
        name = self.get_parameter("tomogram").value
        planner = TomogramPlanner(Config())
        try:
            planner.loadTomogram(name)
            start = np.array([
                self.get_parameter("start_x").value,
                self.get_parameter("start_y").value,
            ], dtype=np.float32)
            goal = np.array([
                self.get_parameter("goal_x").value,
                self.get_parameter("goal_y").value,
            ], dtype=np.float32)
            trajectory = planner.plan(start, goal)
            if trajectory is None:
                self.get_logger().error("A* could not find a path")
                return
            msg = Path()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            for point in trajectory:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = float(point[2])
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
            self.path_pub.publish(msg)
            self.get_logger().info("Published /pct_path with %d poses" % len(msg.poses))
        except Exception as exc:
            self.get_logger().error("Planning failed: %s" % exc)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

