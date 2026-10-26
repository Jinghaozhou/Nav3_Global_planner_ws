import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
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
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("goal_x", -6.0)
        self.declare_parameter("goal_y", -1.0)
        self.declare_parameter("goal_z", 0.0)
        self.declare_parameter("frame_id", "map")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self.plan_service = self.create_service(
            GetPlan,
            "/pct_planner/get_plan",
            self.handle_get_plan,
        )
        self._planner = None
        self._timer = self.create_timer(0.1, self.plan_once)
        self._initial_plan_pending = True

    def plan_once(self):
        if not self._initial_plan_pending:
            return
        self._initial_plan_pending = False
        self._timer.cancel()
        try:
            start = np.array([
                self.get_parameter("start_x").value,
                self.get_parameter("start_y").value,
                self.get_parameter("start_z").value,
            ], dtype=np.float32)
            goal = np.array([
                self.get_parameter("goal_x").value,
                self.get_parameter("goal_y").value,
                self.get_parameter("goal_z").value,
            ], dtype=np.float32)
            self._plan_and_publish(start, goal)
        except Exception as exc:
            self.get_logger().error("Initial planning failed: %s" % exc)

    def handle_get_plan(self, request, response):
        if self._initial_plan_pending:
            self._initial_plan_pending = False
            self._timer.cancel()

        frame_id = str(self.get_parameter("frame_id").value)
        try:
            self._validate_request_frame(request.start.header.frame_id, "start", frame_id)
            self._validate_request_frame(request.goal.header.frame_id, "goal", frame_id)
            if request.tolerance != 0.0:
                self.get_logger().warning(
                    "GetPlan tolerance is not supported and will be ignored"
                )

            start = np.array([
                request.start.pose.position.x,
                request.start.pose.position.y,
                request.start.pose.position.z,
            ], dtype=np.float32)
            goal = np.array([
                request.goal.pose.position.x,
                request.goal.pose.position.y,
                request.goal.pose.position.z,
            ], dtype=np.float32)
            response.plan = self._plan_and_publish(start, goal)
        except Exception as exc:
            response.plan = self._empty_path(frame_id)
            self.get_logger().error("GetPlan request failed: %s" % exc)
        return response

    def _ensure_planner(self):
        if self._planner is None:
            self.get_logger().info("Loading tomogram and initializing planner")
            planner = TomogramPlanner(
                Config(),
                tomo_dir=self.get_parameter("tomogram_dir").value,
            )
            planner.loadTomogram(str(self.get_parameter("tomogram").value))
            self._planner = planner
            self.get_logger().info("Planner initialized")
        return self._planner

    def _plan_and_publish(self, start, goal):
        trajectory = self._ensure_planner().plan(start, goal)
        if trajectory is None:
            raise RuntimeError("planner did not find a path")

        path = to_path_message(
            self,
            trajectory,
            str(self.get_parameter("frame_id").value),
        )
        self.path_pub.publish(path)
        self.get_logger().info("Published /pct_path with %d poses" % len(trajectory))
        return path

    @staticmethod
    def _validate_request_frame(request_frame, label, expected_frame):
        if request_frame and request_frame != expected_frame:
            raise ValueError(
                "%s frame '%s' does not match planner frame '%s'"
                % (label.capitalize(), request_frame, expected_frame)
            )

    def _empty_path(self, frame_id):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        return path


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
