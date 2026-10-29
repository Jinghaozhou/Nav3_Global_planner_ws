"""Serialize the complete final cost grid for visualization consumers."""

from array import array

import numpy as np

from pct_planner_interfaces.msg import TomogramCost


def tomogram_message(data, map_path, signature, cost_barrier, stamp):
    stack = np.asarray(data["data"])
    if stack.ndim != 4 or stack.shape[0] != 5:
        raise ValueError("Tomogram data must have shape [5, layers, x, y]")
    costs = np.ascontiguousarray(stack[0], dtype=np.float32)
    indices = np.asarray(data["layer_indices"], dtype=np.int32)
    if indices.shape != (costs.shape[0],) or np.any(indices < 0) or np.any(np.diff(indices) <= 0):
        raise ValueError("Tomogram layer indices are invalid")

    message = TomogramCost()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.map_path = str(map_path)
    message.map_signature = signature
    message.center_x, message.center_y = (float(v) for v in data["center"])
    message.resolution = float(data["resolution"])
    message.slice_h0 = float(data["slice_h0"])
    message.slice_dh = float(data["slice_dh"])
    message.cost_barrier = float(cost_barrier)
    message.size_x, message.size_y = costs.shape[1:]
    message.source_layer_indices = indices.tolist()
    message.costs = array("f", costs.ravel())
    return message
