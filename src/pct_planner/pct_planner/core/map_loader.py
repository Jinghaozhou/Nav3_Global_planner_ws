"""Read an offline PCD and build/cache its tomogram without ROS imports."""

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import pickle

import numpy as np

TOMOGRAM_CACHE_VERSION = 2

def resolve_map_path(map_dir, map_name):
    name = str(map_name).strip()
    if not name or Path(name).name != name or not name.lower().endswith(".pcd"):
        raise ValueError("map must be a PCD file name within map_dir")
    path = Path(map_dir).expanduser().resolve() / name
    if not path.is_file():
        raise FileNotFoundError("PCD map not found: %s" % path)
    return path


def map_signature(path, config):
    stat = path.stat()
    details = {
        "cache_version": TOMOGRAM_CACHE_VERSION,
        "path": str(path),
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "map": asdict(config.map),
        "traversability": asdict(config.trav),
    }
    encoded = json.dumps(details, sort_keys=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:20]


def read_pcd(path):
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("Open3D is required to read PCD maps") from exc
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("PCD map has no XYZ points: %s" % path)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError("PCD map has no finite XYZ points: %s" % path)
    return np.ascontiguousarray(points)


def build_tomogram(points, config):
    from .tomogram import Tomogram

    config.validate()
    bounds_min = np.min(points, axis=0)
    bounds_max = np.max(points, axis=0)
    ground = float(config.map.ground_h)
    if bounds_max[2] <= ground:
        raise ValueError("map points do not extend above ground_h")
    resolution = config.map.resolution
    slice_dh = config.map.slice_dh
    map_dim_x = int(np.ceil((bounds_max[0] - bounds_min[0]) / resolution)) + 4
    map_dim_y = int(np.ceil((bounds_max[1] - bounds_min[1]) / resolution)) + 4
    n_slice = max(1, int(np.ceil((bounds_max[2] - ground) / slice_dh)))
    center = (bounds_max[:2] + bounds_min[:2]) / 2
    slice_h0 = ground + slice_dh

    core = Tomogram(config)
    core.initMappingEnv(center, map_dim_x, map_dim_y, n_slice, slice_h0)
    layers_t, grad_x, grad_y, layers_g, layers_c, _timing = core.point2map(points)
    return {
        "data": np.stack((layers_t, grad_x, grad_y, layers_g, layers_c)).astype(np.float16),
        "resolution": resolution,
        "center": center,
        "slice_h0": slice_h0,
        "slice_dh": slice_dh,
        "layer_indices": core.layer_indices,
    }


def load_or_build_tomogram(map_path, config, cache_dir):
    signature = map_signature(map_path, config)
    cache_path = Path(cache_dir) / (map_path.stem + "-" + signature + ".pickle")
    if cache_path.is_file():
        with cache_path.open("rb") as handle:
            return pickle.load(handle), signature, True

    result = build_tomogram(read_pcd(map_path), config)
    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_name(cache_path.name + ".tmp")
        with temporary.open("wb") as handle:
            pickle.dump(result, handle, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(temporary, cache_path)
    except OSError:
        # A read-only map directory still permits planning; it only disables caching.
        pass
    return result, signature, False
