"""Publish sampled PCD points and colored occupied voxels from the same map."""

import math
import os
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from pct_planner_interfaces.msg import TomogramCost
from pct_planner_interfaces.srv import PublishCostMap

from .pcd import read_sample
from .cost_projection import TomogramLookup, delete_voxel_marker, voxel_marker

_COST_PARAMETER_NAMES = (
    "resolution", "ground_h", "slice_dh", "kernel_size", "interval_min",
    "interval_free", "slope_max", "step_max", "standable_ratio",
    "cost_barrier", "safe_margin", "inflation",
)


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
        self.declare_parameter("display_voxel_size", 0.10)
        self.max_points = int(self.get_parameter("max_points").value)
        self.display_voxel_size = float(self.get_parameter("display_voxel_size").value)
        if self.max_points < 1:
            raise ValueError("max_points must be positive")
        if not math.isfinite(self.display_voxel_size) or self.display_voxel_size <= 0:
            raise ValueError("display_voxel_size must be positive and finite")
        planner_node = str(self.get_parameter("planner_node").value).rstrip("/")
        if not planner_node:
            raise ValueError("planner_node must name a ROS 2 node")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PointCloud2, "/pct/map_points", qos)
        self.voxel_publisher = self.create_publisher(Marker, "/pct/map_cost_voxels", qos)
        self.tomogram_subscription = self.create_subscription(
            TomogramCost, "/pct/tomogram_layers", self._tomogram_ready, qos
        )
        self.parameter_client = self.create_client(
            GetParameters, planner_node + "/get_parameters"
        )
        self.cost_client = self.create_client(
            PublishCostMap, "/pct/publish_cost_map"
        )
        self._pending = None
        self._cost_pending = None
        self._cost_requested_key = None
        self._cost_pending_key = None
        self._cost_parameter_key = None
        self._expected_signature = None
        self._latest_tomogram = None
        self._colored_key = None
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
        request.names = ["map", "map_dir", *_COST_PARAMETER_NAMES]
        self._pending = self.parameter_client.call_async(request)
        self._pending.add_done_callback(self._parameters_ready)

    def _parameters_ready(self, future):
        self._pending = None
        try:
            values = future.result().values
            if len(values) != 2 + len(_COST_PARAMETER_NAMES) or any(
                value.type != ParameterType.PARAMETER_STRING for value in values[:2]
            ):
                raise ValueError("Planner map and map_dir parameters must be strings")
            cost_key = tuple(
                (value.type, value.integer_value, value.double_value)
                for value in values[2:]
            )
            if cost_key != self._cost_parameter_key:
                self._cost_parameter_key = cost_key
                self._expected_signature = None
                self._colored_key = None
                self._publish_empty_voxels()
            self._refresh(values[0].string_value, values[1].string_value)
            self._request_cost_map()
            self._maybe_publish_cost()
        except Exception as exc:
            self.get_logger().error("Cannot read planner map parameters: %s" % exc)

    def _request_cost_map(self):
        if self._loaded_key is None or self._cost_pending is not None:
            return
        key = (self._loaded_key, self._cost_parameter_key)
        if key == self._cost_requested_key or not self.cost_client.service_is_ready():
            return
        request = PublishCostMap.Request()
        self._cost_requested_key = key
        self._cost_pending_key = key
        self._cost_pending = self.cost_client.call_async(request)
        self._cost_pending.add_done_callback(self._cost_map_ready)

    def _cost_map_ready(self, future):
        key = self._cost_pending_key
        self._cost_pending = None
        self._cost_pending_key = None
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Cost map: %s" % response.message)
                if key == (self._loaded_key, self._cost_parameter_key):
                    self._expected_signature = response.map_signature
                    self._maybe_publish_cost()
            else:
                self.get_logger().error("Cost map: %s" % response.message)
                self._cost_requested_key = None
        except Exception as exc:
            self.get_logger().error("Cost map request failed: %s" % exc)
            self._cost_requested_key = None

    def _tomogram_ready(self, message):
        self._latest_tomogram = message
        self._maybe_publish_cost()

    def _maybe_publish_cost(self):
        message = self._latest_tomogram
        if self._loaded_key is None or message is None:
            return
        if not self._expected_signature or message.map_signature != self._expected_signature:
            return
        if message.map_path != self._loaded_key[0]:
            return
        key = (self._loaded_key, self._cost_parameter_key, message.map_signature)
        if key == self._colored_key:
            return
        try:
            lookup = TomogramLookup(message)
            points, centers, costs, source_count, used_open3d = read_sample(
                self._loaded_key[0], self.max_points, self.display_voxel_size, lookup
            )
            stat = Path(self._loaded_key[0]).stat()
            if (stat.st_size, stat.st_mtime_ns) != self._loaded_key[1:]:
                raise RuntimeError("PCD map changed while being read; retrying")
            stamp = self.get_clock().now().to_msg()
            self.publisher.publish(cloud_message(points, stamp))
            self.voxel_publisher.publish(voxel_marker(
                centers, costs, self.display_voxel_size, lookup.barrier, stamp
            ))
            self._colored_key = key
            self._failed_key = None
            self.get_logger().info("Showing %s: %d occupied voxels from %d PCD points%s; %d unknown costs" % (
                self._loaded_key[0], len(centers), source_count,
                " (Open3D reader)" if used_open3d else "",
                int(np.count_nonzero(~np.isfinite(costs))),
            ))
        except Exception as exc:
            if key != self._failed_key:
                self.get_logger().error("Cannot build cost voxels: %s" % exc)
                self._failed_key = key

    def _publish_empty_voxels(self):
        self.voxel_publisher.publish(delete_voxel_marker(self.get_clock().now().to_msg()))

    def _publish_empty(self):
        self.publisher.publish(cloud_message(
            np.empty((0, 3), dtype="<f4"), self.get_clock().now().to_msg()
        ))
        self._publish_empty_voxels()

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
                self._expected_signature = None
                self._cost_requested_key = None
            return

        if key == self._loaded_key:
            return
        if key != self._last_key:
            self._publish_empty()
            self._last_key = key
        self._loaded_key = key
        self._expected_signature = None
        self._colored_key = None
        self._cost_requested_key = None


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
