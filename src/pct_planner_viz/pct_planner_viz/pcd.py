"""Bounded XYZ sampling for offline PCD maps."""

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


def read_sample(path, max_points):
    """Return finite XYZ points, original count, and whether Open3D was used."""
    if max_points < 1:
        raise ValueError("max_points must be positive")
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
        stride = max(1, (count + max_points - 1) // max_points)
        selected = cloud[::stride]
        points = np.column_stack((selected["x"], selected["y"], selected["z"]))
        del selected, cloud
        used_open3d = False
    elif encoding in ("ascii", "binary_compressed"):
        import open3d as o3d

        cloud = o3d.io.read_point_cloud(str(path))
        source = np.asarray(cloud.points)
        count = len(source)
        if count == 0:
            raise ValueError("PCD map contains no readable XYZ points")
        stride = max(1, (count + max_points - 1) // max_points)
        points = source[::stride]
        used_open3d = True
    else:
        raise ValueError("Unsupported PCD DATA encoding: %s" % encoding)

    points = np.ascontiguousarray(points, dtype="<f4")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError("PCD map contains no finite sampled XYZ points")
    return points, count, used_open3d
