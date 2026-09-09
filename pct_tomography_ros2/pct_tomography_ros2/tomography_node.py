import os
import sys

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .pointcloud_conversion import xyzi, xyz32


def _repo_root():
    return os.environ.get(
        "PCT_PLANNER_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")),
    )


ROOT = _repo_root()
sys.path.insert(0, os.path.join(ROOT, "tomography", "scripts"))
sys.path.insert(0, os.path.join(ROOT, "tomography"))

from config import SceneBuilding, ScenePlaza, SceneSpiral  # noqa: E402
from tomogram import Tomogram  # noqa: E402


SCENES = {
    "building": SceneBuilding,
    "spiral": SceneSpiral,
    "plaza": ScenePlaza,
}


class TomographyNode(Node):
    def __init__(self):
        super().__init__("pct_tomography")
        self.declare_parameter("scene", "building")
        self.declare_parameter("pcd_dir", os.path.join(ROOT, "rsc", "pcd"))
        self.declare_parameter("output_dir", os.path.join(ROOT, "rsc", "tomogram"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_visualization", True)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._qos = qos
        self.pointcloud_pub = self.create_publisher(PointCloud2, "/global_points", qos)
        self.tomogram_pub = self.create_publisher(PointCloud2, "/tomogram", qos)
        self.layer_g_pubs = []
        self.layer_c_pubs = []
        self._timer = self.create_timer(0.1, self.generate_once)
        self._done = False

    def generate_once(self):
        if self._done:
            return
        self._done = True
        scene_name = str(self.get_parameter("scene").value).lower()
        scene_cfg = SCENES.get(scene_name)
        if scene_cfg is None:
            self.get_logger().error("Unknown scene '%s'; choose building, spiral, or plaza" % scene_name)
            return

        try:
            pcd_path = os.path.join(
                str(self.get_parameter("pcd_dir").value), scene_cfg.pcd.file_name
            )
            points = np.asarray(o3d.io.read_point_cloud(pcd_path).points, dtype=np.float32)
            if points.ndim != 2 or points.shape[1] < 3 or points.shape[0] == 0:
                raise ValueError("PCD contains no XYZ points: %s" % pcd_path)
            points = points[:, :3]

            core = Tomogram(scene_cfg)
            points_min = np.min(points, axis=0)
            points_max = np.max(points, axis=0)
            points_min[-1] = scene_cfg.map.ground_h
            map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / scene_cfg.map.resolution)) + 4
            map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / scene_cfg.map.resolution)) + 4
            n_slice = int(np.ceil((points_max[2] - points_min[2]) / scene_cfg.map.slice_dh))
            center = (points_max[:2] + points_min[:2]) / 2
            slice_h0 = points_min[-1] + scene_cfg.map.slice_dh
            core.initMappingEnv(center, map_dim_x, map_dim_y, n_slice, slice_h0)
            layers_t, grad_x, grad_y, layers_g, layers_c, timing = core.point2map(points)

            os.makedirs(str(self.get_parameter("output_dir").value), exist_ok=True)
            output = os.path.join(
                str(self.get_parameter("output_dir").value),
                os.path.splitext(scene_cfg.pcd.file_name)[0] + ".pickle",
            )
            import pickle
            with open(output, "wb") as handle:
                pickle.dump({
                    "data": np.stack((layers_t, grad_x, grad_y, layers_g, layers_c)).astype(np.float16),
                    "resolution": scene_cfg.map.resolution,
                    "center": center,
                    "slice_h0": slice_h0,
                    "slice_dh": scene_cfg.map.slice_dh,
                }, handle, protocol=pickle.HIGHEST_PROTOCOL)
            self.get_logger().info("Tomogram saved to %s" % output)

            if self.get_parameter("publish_visualization").value:
                from std_msgs.msg import Header

                msg_header = Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=str(self.get_parameter("map_frame").value),
                )
                self.pointcloud_pub.publish(xyz32(msg_header, points))
                self._create_layer_publishers(len(layers_g), self._qos)
                self._publish_layers(msg_header, layers_g, layers_t, center, scene_cfg.map.resolution)
                self._publish_layers(msg_header, layers_c, None, center, scene_cfg.map.resolution, ceiling=True)
                self.tomogram_pub.publish(
                    xyzi(msg_header, self._tomogram_points(
                        layers_g, layers_t, center, scene_cfg.map.resolution,
                        scene_cfg.map.slice_dh,
                    ))
                )
        except Exception as exc:
            self.get_logger().error("Tomogram generation failed: %s" % exc)

    @staticmethod
    def _grid_points(rows, cols, center, resolution):
        xx, yy = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
        return xx, yy

    def _create_layer_publishers(self, n_layers, qos):
        self.layer_g_pubs = [
            self.create_publisher(PointCloud2, "/layer_G_%d" % i, qos)
            for i in range(n_layers)
        ]
        self.layer_c_pubs = [
            self.create_publisher(PointCloud2, "/layer_C_%d" % i, qos)
            for i in range(n_layers)
        ]

    def _publish_layers(self, header, layers, color, center, resolution, ceiling=False):
        rows, cols = layers.shape[1:]
        xx, yy = self._grid_points(rows, cols, center, resolution)
        publishers = self.layer_c_pubs if ceiling else self.layer_g_pubs
        for index, layer in enumerate(layers):
            mask = ~np.isnan(layer)
            if not np.any(mask):
                points = np.empty((0, 4), dtype=np.float32)
            else:
                intensity = np.ones(np.count_nonzero(mask), dtype=np.float32)
                if color is not None:
                    intensity = np.asarray(color[index][mask], dtype=np.float32)
                points = np.column_stack((
                    (xx[mask] - rows / 2) * resolution + center[0],
                    (yy[mask] - cols / 2) * resolution + center[1],
                    layer[mask],
                    intensity,
                )).astype(np.float32)
            publishers[index].publish(xyzi(header, points))

    @staticmethod
    def _tomogram_points(layers_g, layers_t, center, resolution, slice_dh):
        """Build the same merged layer visualization as the legacy node."""
        vis_g = np.array(layers_g, copy=True)
        vis_t = np.array(layers_t, copy=True)
        rows, cols = vis_g.shape[1:]
        xx, yy = np.meshgrid(np.arange(rows), np.arange(cols), indexing="ij")
        points = []
        for index in range(max(0, vis_g.shape[0] - 1)):
            mask_h = (vis_g[index + 1] - vis_g[index]) < slice_dh
            vis_g[index, mask_h] = np.nan
            vis_t[index + 1, mask_h] = np.minimum(
                vis_t[index, mask_h], vis_t[index + 1, mask_h]
            )
            mask = ~np.isnan(vis_g[index])
            points.append(np.column_stack((
                (xx[mask] - rows / 2) * resolution + center[0],
                (yy[mask] - cols / 2) * resolution + center[1],
                vis_g[index][mask],
                vis_t[index][mask],
            )))
        if vis_g.shape[0]:
            mask = ~np.isnan(vis_g[-1])
            points.append(np.column_stack((
                (xx[mask] - rows / 2) * resolution + center[0],
                (yy[mask] - cols / 2) * resolution + center[1],
                vis_g[-1][mask],
                vis_t[-1][mask],
            )))
        return np.concatenate(points, axis=0).astype(np.float32) if points else np.empty((0, 4), dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = TomographyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
