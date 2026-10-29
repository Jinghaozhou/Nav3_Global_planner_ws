"""Bounded, spatially distributed XYZ sampling for offline PCD maps."""

import heapq
from pathlib import Path

import numpy as np


_NUMPY_TYPES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
    ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
}


def _header(path):
    values = {}
    with Path(path).open("rb") as handle:
        for _ in range(100):
            line = handle.readline(4096)
            if not line:
                break
            if len(line) == 4096 and not line.endswith(b"\n"):
                raise ValueError("PCD header line is too long")
            line = line.strip()
            if not line or line.startswith(b"#"):
                continue
            parts = line.decode("ascii").split()
            values[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                return values, handle.tell()
    raise ValueError("PCD DATA header is missing")


def _voxel_sample(chunks, max_points, voxel_size, lookup=None):
    """Keep bounded occupied voxels and their maximum projected point cost."""
    chosen = {}
    heap = []
    multipliers = np.array([0x9E3779B185EBCA87, 0xC2B2AE3D27D4EB4F,
                            0x165667B19E3779F9], dtype=np.uint64)
    for chunk in chunks:
        points = np.column_stack((chunk["x"], chunk["y"], chunk["z"])) if chunk.dtype.fields else chunk
        points = np.asarray(points, dtype=np.float32)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:
            continue
        coords = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64)
        if lookup is None:
            voxels, first = np.unique(coords, axis=0, return_index=True)
            voxel_costs = np.full(len(voxels), -np.inf, dtype=np.float32)
        else:
            voxels, first, inverse = np.unique(
                coords, axis=0, return_index=True, return_inverse=True
            )
            values = lookup.lookup(points)
            voxel_costs = np.full(len(voxels), -np.inf, dtype=np.float32)
            np.maximum.at(
                voxel_costs, inverse,
                np.where(np.isfinite(values), values, -np.inf),
            )
        representatives = points[first]
        bits = voxels.astype(np.uint64)
        hashes = (bits * multipliers).sum(axis=1, dtype=np.uint64)
        hashes ^= hashes >> np.uint64(33)
        hashes *= np.uint64(0xff51afd7ed558ccd)
        hashes ^= hashes >> np.uint64(33)
        for voxel, point, value, hashed in zip(voxels, representatives, voxel_costs, hashes):
            rank = int(hashed)
            key = tuple(int(v) for v in voxel)
            if key in chosen:
                if value > chosen[key][1]:
                    chosen[key] = (chosen[key][0], value)
                continue
            if len(heap) >= max_points and rank >= -heap[0][0]:
                continue
            if len(heap) >= max_points:
                _, removed = heapq.heappop(heap)
                del chosen[removed]
            chosen[key] = (point, value)
            heapq.heappush(heap, (-rank, key))
    if not chosen:
        raise ValueError("PCD map contains no finite sampled XYZ points")
    keys = sorted(chosen)
    points = np.ascontiguousarray([chosen[key][0] for key in keys], dtype="<f4")
    if lookup is None:
        return points
    centers = (np.asarray(keys, dtype=np.float64) + 0.5) * voxel_size
    costs = np.asarray([chosen[key][1] for key in keys], dtype=np.float32)
    costs[~np.isfinite(costs)] = np.nan
    return points, centers, costs


def read_sample(path, max_points, voxel_size=0.0, lookup=None):
    """Return sampled PCD points; with lookup, also return voxel centers/costs."""
    if max_points < 1:
        raise ValueError("max_points must be positive")
    if not np.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("display_voxel_size must be finite and nonnegative")
    if lookup is not None and voxel_size == 0:
        raise ValueError("display_voxel_size must be positive for voxel display")
    header, offset = _header(path)
    fields = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD field metadata has inconsistent lengths")
    if any(size <= 0 or count <= 0 for size, count in zip(sizes, counts)):
        raise ValueError("PCD field size and count must be positive")
    if not all(axis in fields for axis in ("x", "y", "z")):
        raise ValueError("PCD map must contain x, y and z fields")
    count = int(header["POINTS"][0]) if "POINTS" in header else (
        int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
    )
    if count <= 0:
        raise ValueError("PCD map contains no points")
    encoding = header["DATA"][0].lower()

    if encoding == "binary":
        offsets = {}
        formats = {}
        point_step = 0
        for name, size, kind, field_count in zip(fields, sizes, types, counts):
            if name in ("x", "y", "z"):
                if field_count != 1 or (kind, size) not in _NUMPY_TYPES:
                    raise ValueError("PCD XYZ field has an unsupported numeric type")
                offsets[name] = point_step
                formats[name] = _NUMPY_TYPES[(kind, size)]
            point_step += size * field_count
        if Path(path).stat().st_size < offset + count * point_step:
            raise ValueError("PCD binary payload is truncated")
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [formats[axis] for axis in ("x", "y", "z")],
            "offsets": [offsets[axis] for axis in ("x", "y", "z")],
            "itemsize": point_step,
        })
        cloud = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,))
        if voxel_size > 0:
            sample = _voxel_sample(
                (cloud[start:start + 200000] for start in range(0, count, 200000)),
                max_points, voxel_size, lookup,
            )
        else:
            stride = max(1, (count + max_points - 1) // max_points)
            selected = cloud[::stride]
            points = np.column_stack((selected["x"], selected["y"], selected["z"]))
        del cloud
        used_open3d = False
    elif encoding in ("ascii", "binary_compressed"):
        import open3d as o3d

        cloud = o3d.io.read_point_cloud(str(path))
        source = np.asarray(cloud.points)
        count = len(source)
        if count == 0:
            raise ValueError("PCD map contains no readable XYZ points")
        if voxel_size > 0:
            sample = _voxel_sample(
                (source[start:start + 200000] for start in range(0, count, 200000)),
                max_points, voxel_size, lookup,
            )
        else:
            stride = max(1, (count + max_points - 1) // max_points)
            points = source[::stride]
        used_open3d = True
    else:
        raise ValueError("Unsupported PCD DATA encoding: %s" % encoding)

    if voxel_size > 0:
        if lookup is None:
            points = sample
        else:
            points, centers, costs = sample
    points = np.ascontiguousarray(points, dtype="<f4")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError("PCD map contains no finite sampled XYZ points")
    if lookup is not None:
        return points, centers, costs, count, used_open3d
    return points, count, used_open3d
