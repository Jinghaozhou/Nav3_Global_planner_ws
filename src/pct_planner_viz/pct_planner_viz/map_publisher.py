"""Publish a sampled view of the map selected by the live planner node."""

import os
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from .pcd import read_sample


def default_map_dir():
    override = os.environ.get("PCT_MAP_DIR")
    if override:
        return Path(override).expanduser().resolve()
    for origin in (Path.cwd(), Path(__file__).resolve()):
        for parent in (origin, *origin.parents):
            candidate = parent / "map"
            if candidate.is_dir():
                return candidate.resolve()
    return (Path.cwd() / "map").resolve()


def cloud_message(points, stamp):
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.width * message.point_step
    message.is_dense = True
    message.data = points.tobytes()
    return message


class MapPublisher(Node):
    def __init__(self):
        super().__init__("pct_map_visualizer")
        self.declare_parameter("planner_node", "/pct_planner")
        self.declare_parameter("max_points", 200000)
        self.max_points = int(self.get_parameter("max_points").value)
        if self.max_points < 1:
            raise ValueError("max_points must be positive")
        planner_node = str(self.get_parameter("planner_node").value).rstrip("/")
        if not planner_node:
            raise ValueError("planner_node must name a ROS 2 node")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PointCloud2, "/pct/map_points", qos)
        self.parameter_client = self.create_client(
            GetParameters, planner_node + "/get_parameters"
        )
        self._pending = None
        self._last_key = None
        self._loaded_key = None
        self._failed_key = None
        self._waiting_logged = False
        self.create_timer(2.0, self._poll_map)
        self.get_logger().info("Waiting for map parameters from %s" % planner_node)

    def _poll_map(self):
        if self._pending is not None:
            return
        if not self.parameter_client.service_is_ready():
            if not self._waiting_logged:
                self.get_logger().info("Planner parameter service is not available yet")
                self._waiting_logged = True
            return
        self._waiting_logged = False
        request = GetParameters.Request()
        request.names = ["map", "map_dir"]
        self._pending = self.parameter_client.call_async(request)
        self._pending.add_done_callback(self._parameters_ready)

    def _parameters_ready(self, future):
        self._pending = None
        try:
            values = future.result().values
            if len(values) != 2 or any(
                value.type != ParameterType.PARAMETER_STRING for value in values
            ):
                raise ValueError("Planner map and map_dir parameters must be strings")
            self._refresh(values[0].string_value, values[1].string_value)
        except Exception as exc:
            self.get_logger().error("Cannot read planner map parameters: %s" % exc)

    def _publish_empty(self):
        import numpy as np

        self.publisher.publish(cloud_message(
            np.empty((0, 3), dtype="<f4"), self.get_clock().now().to_msg()
        ))

    def _refresh(self, map_name, map_dir):
        try:
            name = str(map_name).strip()
            if not name or Path(name).name != name or not name.lower().endswith(".pcd"):
                raise ValueError("map must be a PCD file name within map_dir")
            directory = Path(map_dir).expanduser().resolve() if map_dir.strip() else default_map_dir()
            path = directory / name
            stat = path.stat()
            if not path.is_file():
                raise ValueError("PCD map is not a file: %s" % path)
            key = (str(path), stat.st_size, stat.st_mtime_ns)
        except (OSError, ValueError) as exc:
            key = (str(map_name), str(map_dir), str(exc))
            if key != self._last_key:
                self._publish_empty()
                self.get_logger().error("Cannot select PCD map: %s" % exc)
                self._last_key = key
                self._loaded_key = None
            return

        if key == self._loaded_key:
            return
        if key != self._last_key:
            self._publish_empty()
            self._last_key = key
            self._loaded_key = None
        try:
            points, source_count, used_open3d = read_sample(path, self.max_points)
            self.publisher.publish(cloud_message(points, self.get_clock().now().to_msg()))
            self._loaded_key = key
            self._failed_key = None
            self.get_logger().info(
                "Showing %s: %d of %d points%s" % (
                    path, len(points), source_count,
                    " (Open3D reader)" if used_open3d else "",
                )
            )
        except Exception as exc:
            if key != self._failed_key:
                self.get_logger().error("Cannot load PCD map %s: %s" % (path, exc))
                self._failed_key = key


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
