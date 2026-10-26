"""Service-driven ROS 2 adapter for the offline PCT global planner."""

from dataclasses import asdict
import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pct_planner_interfaces.srv import Plan
from pct_planner.core.config import MapConfig, PCTConfig, PlannerConfig, TraversabilityConfig
from pct_planner.core.map_loader import load_or_build_tomogram, map_signature, resolve_map_path


def default_map_dir():
    override = os.environ.get("PCT_MAP_DIR")
    if override:
        return str(Path(override).expanduser().resolve())
    for origin in (Path.cwd(), Path(__file__).resolve()):
        for parent in (origin, *origin.parents):
            candidate = parent / "map"
            if candidate.is_dir():
                return str(candidate)
    return str(Path.cwd() / "map")


def to_path_message(node, trajectory):
    path = RosPath()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = "map"
    for xyz in trajectory:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


class PCTPlannerNode(Node):
    def __init__(self):
        super().__init__("pct_planner")
        self.declare_parameter("map", "building2_9.pcd")
        self.declare_parameter("map_dir", "")
        self.declare_parameter("cache_dir", "")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("ground_h", 0.0)
        self.declare_parameter("slice_dh", 0.5)
        self.declare_parameter("kernel_size", 7)
        self.declare_parameter("interval_min", 0.50)
        self.declare_parameter("interval_free", 0.65)
        self.declare_parameter("slope_max", 0.40)
        self.declare_parameter("step_max", 0.17)
        self.declare_parameter("standable_ratio", 0.20)
        self.declare_parameter("cost_barrier", 50.0)
        self.declare_parameter("safe_margin", 0.4)
        self.declare_parameter("inflation", 0.2)
        self.declare_parameter("use_quintic", True)
        self.declare_parameter("max_heading_rate", 10.0)
        self.declare_parameter("start_cost_threshold", 20.0)
        self.declare_parameter("safe_cost_margin", 15.0)
        self.declare_parameter("step_cost_weight", 0.2)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(RosPath, "/pct/global_path", qos)
        self.plan_service = self.create_service(Plan, "/pct/plan", self.handle_plan)
        self._planner = None
        self._planner_key = None
        self.get_logger().info("Ready: call /pct/plan with start and goal in map frame")

    def _value(self, name):
        return self.get_parameter(name).value

    def _config(self):
        config = PCTConfig(
            map=MapConfig(
                resolution=float(self._value("resolution")),
                ground_h=float(self._value("ground_h")),
                slice_dh=float(self._value("slice_dh")),
            ),
            trav=TraversabilityConfig(**{
                name: self._value(name) for name in asdict(TraversabilityConfig())
            }),
            planner=PlannerConfig(**{
                name: self._value(name) for name in asdict(PlannerConfig())
            }),
        )
        config.validate()
        return config

    def _ensure_planner(self):
        config = self._config()
        map_dir = str(self._value("map_dir")).strip() or default_map_dir()
        map_path = resolve_map_path(map_dir, self._value("map"))
        signature = map_signature(map_path, config)
        key = (signature, config.planner)
        if self._planner is not None and self._planner_key == key:
            return self._planner

        cache_dir = str(self._value("cache_dir")).strip() or str(Path(map_dir) / "cache")
        self.get_logger().info("Loading PCD map: %s" % map_path)
        data, _, cached = load_or_build_tomogram(map_path, config, cache_dir)
        from pct_planner.core.planner import TomogramPlanner

        planner = TomogramPlanner(config)
        planner.load_tomogram(data)
        self._planner = planner
        self._planner_key = key
        self.get_logger().info("Tomogram %s; planner initialized" % ("loaded from cache" if cached else "generated"))
        return planner

    @staticmethod
    def _point(message, label):
        if message.header.frame_id != "map":
            raise ValueError("%s frame_id must be 'map'" % label)
        position = message.pose.position
        xyz = np.array([position.x, position.y, position.z], dtype=np.float64)
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("%s XYZ must be finite" % label)
        return xyz

    def handle_plan(self, request, response):
        try:
            start = self._point(request.start, "start")
            goal = self._point(request.goal, "goal")
            trajectory = self._ensure_planner().plan(start, goal)
            if trajectory is None or len(trajectory) == 0:
                raise RuntimeError("no feasible path found")
            trajectory = np.asarray(trajectory, dtype=np.float64)
            if trajectory.ndim != 2 or trajectory.shape[1] != 3 or not np.isfinite(trajectory).all():
                raise RuntimeError("planner returned an invalid XYZ trajectory")
            response.path = to_path_message(self, trajectory)
            response.success = True
            response.message = "planned %d poses" % len(response.path.poses)
            self.path_pub.publish(response.path)
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.path = to_path_message(self, [])
            self.path_pub.publish(response.path)
            self.get_logger().error("Planning failed: %s" % exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PCTPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
