"""Read final Tomogram costs and render occupied PCD voxels in RViz."""

import math

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


_COST_PALETTE = np.array([
    [100, 143, 135],  # zero cost: muted teal
    [196, 170, 117],  # half barrier: sand
    [185, 101, 93],   # barrier: muted brick red
], dtype=np.float32) / 255.0
_UNKNOWN_COLOR = (0.55, 0.57, 0.58)


class TomogramLookup:
    """Vectorized lookup of the nearest retained cut above each PCD point."""

    def __init__(self, tomogram):
        if tomogram.header.frame_id != "map":
            raise ValueError("Tomogram frame_id must be map")
        self.nx, self.ny = int(tomogram.size_x), int(tomogram.size_y)
        indices = np.asarray(tomogram.source_layer_indices, dtype=np.int32)
        if self.nx < 1 or self.ny < 1 or len(indices) < 1 or np.any(np.diff(indices) <= 0):
            raise ValueError("Tomogram geometry is invalid")
        if len(tomogram.costs) != len(indices) * self.nx * self.ny:
            raise ValueError("Tomogram cost array has the wrong length")
        self.resolution = float(tomogram.resolution)
        self.barrier = float(tomogram.cost_barrier)
        slice_dh = float(tomogram.slice_dh)
        scalars = (self.resolution, slice_dh, self.barrier, tomogram.slice_h0,
                   tomogram.center_x, tomogram.center_y)
        if not all(math.isfinite(value) for value in scalars) or min(self.resolution, slice_dh, self.barrier) <= 0:
            raise ValueError("Tomogram scale must be positive and finite")
        self.center_x = float(tomogram.center_x)
        self.center_y = float(tomogram.center_y)
        self.costs = np.asarray(tomogram.costs, dtype=np.float32).reshape((len(indices), self.nx, self.ny))
        self.heights = float(tomogram.slice_h0) + indices.astype(np.float64) * slice_dh

    def lookup(self, points):
        values = np.full(len(points), np.nan, dtype=np.float32)
        if len(points) == 0:
            return values
        ix = np.rint((points[:, 0].astype(np.float64) - self.center_x) / self.resolution).astype(np.int64) + self.nx // 2
        iy = np.rint((points[:, 1].astype(np.float64) - self.center_y) / self.resolution).astype(np.int64) + self.ny // 2
        layer = np.searchsorted(self.heights, points[:, 2], side="left")
        valid = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny) & (layer < len(self.heights))
        selected = np.flatnonzero(valid)
        values[selected] = self.costs[layer[selected], ix[selected], iy[selected]]
        values[~np.isfinite(values)] = np.nan
        return values


def voxel_marker(centers, costs, voxel_size, barrier, stamp):
    if len(centers) != len(costs):
        raise ValueError("Voxel centers and costs must have the same length")
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = "cost_voxels"
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = float(voxel_size)
    marker.color.a = 1.0
    marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in centers]
    t = np.clip(np.nan_to_num(costs, nan=0.0) / barrier, 0.0, 1.0)
    stops = (0.0, 0.5, 1.0)
    red, green, blue = (
        np.interp(t, stops, _COST_PALETTE[:, channel]) for channel in range(3)
    )
    colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
              for r, g, b in zip(red, green, blue)]
    for index in np.flatnonzero(~np.isfinite(costs)):
        colors[int(index)] = ColorRGBA(
            r=_UNKNOWN_COLOR[0], g=_UNKNOWN_COLOR[1], b=_UNKNOWN_COLOR[2], a=1.0
        )
    marker.colors = colors
    return marker


def delete_voxel_marker(stamp):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = "cost_voxels"
    marker.id = 0
    marker.action = Marker.DELETE
    return marker
