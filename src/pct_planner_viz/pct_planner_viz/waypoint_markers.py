"""Persist and update start/goal markers placed by RViz toolbar tools."""

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PointStamped, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback,
    Marker, MarkerArray,
)


_WAYPOINTS = {
    "start": ("START", (0.27, 0.66, 0.91)),
    "goal": ("GOAL", (0.95, 0.59, 0.31)),
}


def _pose(xyz):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    pose.orientation.w = 1.0
    return pose


class WaypointMarkers(Node):
    def __init__(self):
        super().__init__("pct_waypoint_markers")
        self.declare_parameter("marker_size", 0.8)
        self.marker_size = float(self.get_parameter("marker_size").value)
        if not math.isfinite(self.marker_size) or self.marker_size <= 0:
            raise ValueError("marker_size must be positive and finite")

        self.server = InteractiveMarkerServer(self, "/pct/waypoints")
        self.poses = {}
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscription = self.create_subscription(
            PointCloud2, "/pct/map_points", self._map_ready, qos
        )
        self.state_publisher = self.create_publisher(MarkerArray, "/pct/ui/waypoint_state", qos)
        self.click_subscriptions = [
            self.create_subscription(
                PointStamped, "/pct/ui/%s_click" % name,
                lambda message, role=name: self._place(role, message), 10,
            ) for name in _WAYPOINTS
        ]
        self.reset_subscriptions = [
            self.create_subscription(
                Empty, "/pct/ui/%s_reset" % name,
                lambda _message, role=name: self._reset(role), 10,
            ) for name in _WAYPOINTS
        ]
        self._publish_state()
        self.get_logger().info("Use RViz Place Start / Place Goal tools to choose map XYZ")

    def _map_ready(self, cloud):
        if cloud.width == 0 and self.poses:
            self.server.clear()
            self.server.applyChanges()
            self.poses.clear()
            self._publish_state()

    def _place(self, name, point):
        if point.header.frame_id != "map":
            self.get_logger().error("Waypoint frame_id must be map")
            return
        xyz = (point.point.x, point.point.y, point.point.z)
        if not all(math.isfinite(v) for v in xyz):
            self.get_logger().error("Ignoring non-finite waypoint coordinates")
            return
        pose = _pose(xyz)
        self.poses[name] = pose
        self.server.insert(self._marker(name, pose), feedback_callback=self._feedback)
        self.server.applyChanges()
        self._publish_state()
        self.get_logger().info("%s map XYZ: %.3f, %.3f, %.3f" % (_WAYPOINTS[name][0], *xyz))

    def _reset(self, name):
        self.poses.pop(name, None)
        self.server.erase(name)
        self.server.applyChanges()
        self._publish_state()

    def _publish_state(self):
        state = MarkerArray()
        for name in _WAYPOINTS:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = name
            marker.id = 0
            marker.action = Marker.ADD if name in self.poses else Marker.DELETE
            if name in self.poses:
                marker.pose = deepcopy(self.poses[name])
            state.markers.append(marker)
        self.state_publisher.publish(state)

    def _marker(self, name, pose):
        title, rgb = _WAYPOINTS[name]
        xyz = pose.position
        label = "%s (%.3f, %.3f, %.3f)" % (title, xyz.x, xyz.y, xyz.z)
        marker = InteractiveMarker()
        marker.header.frame_id = "map"
        marker.name = name
        marker.description = label
        marker.pose = deepcopy(pose)
        marker.scale = self.marker_size

        visual = InteractiveMarkerControl()
        visual.name = "move_3d"
        visual.always_visible = True
        visual.interaction_mode = InteractiveMarkerControl.MOVE_3D
        visual.orientation.w = 1.0
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.38 * self.marker_size
        sphere.color.r, sphere.color.g, sphere.color.b = rgb
        sphere.color.a = 0.95
        visual.markers.append(sphere)
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.orientation.w = 1.0
        text.pose.position.z = 0.55 * self.marker_size
        text.scale.z = 0.22 * self.marker_size
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        visual.markers.append(text)
        marker.controls.append(visual)

        half = math.sqrt(0.5)
        axes = (
            ("move_x", (0.0, 0.0, 0.0, 1.0)),
            ("move_y", (0.0, 0.0, half, half)),
            ("move_z", (0.0, -half, 0.0, half)),
        )
        for axis_name, (qx, qy, qz, qw) in axes:
            control = InteractiveMarkerControl()
            control.name = axis_name
            control.orientation.x = qx
            control.orientation.y = qy
            control.orientation.z = qz
            control.orientation.w = qw
            control.orientation_mode = InteractiveMarkerControl.FIXED
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)
        return marker

    def _feedback(self, feedback):
        if feedback.marker_name not in _WAYPOINTS or feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP
        ):
            return
        position = feedback.pose.position
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
            self.get_logger().error("Ignoring non-finite waypoint coordinates")
            return
        self.poses[feedback.marker_name] = deepcopy(feedback.pose)
        self._publish_state()
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            name = feedback.marker_name
            self.server.insert(self._marker(name, self.poses[name]), feedback_callback=self._feedback)
            self.server.applyChanges()
            self.get_logger().info("%s map XYZ: %.3f, %.3f, %.3f" % (
                _WAYPOINTS[name][0], position.x, position.y, position.z
            ))

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
