import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .trajectory_conversion import to_path_message


def _repo_root():
    return os.environ.get(
        "PCT_PLANNER_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")),
    )


ROOT = _repo_root()
# The legacy planner remains outside this ROS2 package during phase one.
for path in (ROOT, os.path.join(ROOT, "planner"), os.path.join(ROOT, "planner", "scripts"), os.path.join(ROOT, "planner", "lib")):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import Config  # noqa: E402
from planner_wrapper import TomogramPlanner  # noqa: E402


class PlannerNode(Node):
    def __init__(self):
        super().__init__("pct_planner")
        self.declare_parameter("tomogram", "building2_9")
        self.declare_parameter("tomogram_dir", os.path.join(ROOT, "rsc", "tomogram"))
        self.declare_parameter("start_x", 5.0)
        self.declare_parameter("start_y", 5.0)
        self.declare_parameter("goal_x", -6.0)
        self.declare_parameter("goal_y", -1.0)
        self.declare_parameter("frame_id", "map")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self._timer = self.create_timer(0.1, self.plan_once)
        self._planned = False

    def plan_once(self):
        if self._planned:
            return
        self._planned = True
        self._timer.cancel()
        try:
            planner = TomogramPlanner(
                Config(),
                tomo_dir=self.get_parameter("tomogram_dir").value,
            )
            planner.loadTomogram(str(self.get_parameter("tomogram").value))
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
                self.get_logger().error("Planner did not find a path")
                return
            self.path_pub.publish(
                to_path_message(
                    self, trajectory, str(self.get_parameter("frame_id").value)
                )
            )
            self.get_logger().info("Published /pct_path with %d poses" % len(trajectory))
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


if __name__ == "__main__":
    main()
