#!/usr/bin/env python3
"""Construct a tomogram without requiring a ROS 1 installation.

This installs tiny in-process ROS 1 stubs, then reuses the original
tomography.py implementation.  Point-cloud publishing is a no-op; the
computed tomogram is still exported to rsc/tomogram/.
"""
import runpy
import sys
import types


def _install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.loginfo = lambda *args, **kwargs: print("[INFO]", *args)
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    rospy.init_node = lambda *args, **kwargs: None
    rospy.spin = lambda: None
    rospy.Publisher = lambda *args, **kwargs: types.SimpleNamespace(
        publish=lambda message: None
    )
    sys.modules["rospy"] = rospy

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = type("Header", (), {"__init__": lambda self: None})
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class PointField:
        FLOAT32 = 7

        def __init__(self, name, offset, datatype, count):
            self.name, self.offset = name, offset
            self.datatype, self.count = datatype, count

    sensor_msgs_msg.PointField = PointField
    sensor_msgs_msg.PointCloud2 = type("PointCloud2", (), {})
    point_cloud2 = types.ModuleType("sensor_msgs.point_cloud2")
    point_cloud2.create_cloud_xyz32 = lambda *args, **kwargs: None
    point_cloud2.create_cloud = lambda *args, **kwargs: None
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["sensor_msgs.point_cloud2"] = point_cloud2


if __name__ == "__main__":
    _install_ros_stubs()
    sys.argv = ["tomography.py", "--scene", "Building"]
    runpy.run_path("tomography.py", run_name="__main__")
