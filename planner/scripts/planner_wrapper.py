import os
import sys
import pickle
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(PLANNER_DIR)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
if PLANNER_DIR not in sys.path:
    sys.path.insert(0, PLANNER_DIR)

from utils.convertion import transTrajGrid2Map
from lib import a_star, ele_planner, traj_opt


class TomogramPlanner(object):
    def __init__(self, cfg, tomo_dir=None):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        # Keep the legacy repository-relative default, but allow ROS2 and
        # other callers to provide an explicit tomogram directory.
        if tomo_dir is None:
            tomo_dir = os.path.join(
                REPO_ROOT,
                self.cfg.wrapper.tomo_dir.lstrip(os.sep),
            )
        self.tomo_dir = os.path.abspath(os.path.expanduser(str(tomo_dir)))

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.elev_g_raw = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        tomo_path = os.path.join(self.tomo_dir, str(tomo_file) + '.pickle')
        if not os.path.isfile(tomo_path):
            raise FileNotFoundError("Tomogram file not found: %s" % tomo_path)

        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        self.elev_g_raw = np.asarray(tomogram[3], dtype=np.float32)
        elev_g = np.nan_to_num(self.elev_g_raw, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        # All matrices in the C++ API are Eigen::MatrixXd (float64).  Keep
        # the gateway values numeric but explicitly convert the int mask to
        # a contiguous float64 matrix; older pybind11 builds can otherwise
        # crash while converting an int32 ndarray to Eigen::MatrixXd.
        gateway = np.ascontiguousarray(gateway, dtype=np.double)

        print(
            "init_map matrices:",
            "trav", trav.shape, trav.dtype,
            "elev_g", elev_g.shape, elev_g.dtype,
            "elev_c", elev_c.shape, elev_c.dtype,
            "gateway", gateway.shape, gateway.dtype,
            "grad_y", trav_gy.shape, trav_gy.dtype,
            "grad_x", trav_gx.shape, trav_gx.dtype,
            flush=True,
        )

        cost_map = np.ascontiguousarray(trav.reshape(-1, trav.shape[-1]), dtype=np.double)
        height_map = np.ascontiguousarray(elev_g.reshape(-1, elev_g.shape[-1]), dtype=np.double)
        ceiling_map = np.ascontiguousarray(elev_c.reshape(-1, elev_c.shape[-1]), dtype=np.double)
        ele_map = np.ascontiguousarray(gateway.reshape(-1, gateway.shape[-1]), dtype=np.double)
        grad_y_map = np.ascontiguousarray(trav_gy.reshape(-1, trav_gy.shape[-1]), dtype=np.double)
        grad_x_map = np.ascontiguousarray(-trav_gx.reshape(-1, trav_gx.shape[-1]), dtype=np.double)

        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            cost_map, height_map, ceiling_map, ele_map, grad_y_map, grad_x_map
        )

    def plan(self, start_pos, end_pos):
        """Plan between world-frame positions ``[x, y]`` or ``[x, y, z]``.

        Three-dimensional positions use the requested world height to select
        the closest valid elevation layer at each XY grid cell.  Two-
        dimensional positions remain supported for legacy callers and use
        layer zero.
        """
        self.start_idx = self.position_to_index(start_pos, "start")
        self.end_idx = self.position_to_index(end_pos, "goal")

        print(
            "Planning indices:",
            "start", self.start_idx.tolist(),
            "goal", self.end_idx.tolist(),
            flush=True,
        )

        success = self.planner.plan(self.start_idx, self.end_idx, True)
        if not success:
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d
    
    def pos2idx(self, pos):
        """Convert world XY coordinates to the planner's XY index order."""
        pos = np.asarray(pos, dtype=np.float64)
        if pos.shape != (2,):
            raise ValueError("XY position must contain exactly 2 values")
        raw_idx = np.rint((pos - self.center) / self.resolution).astype(np.int32)
        raw_idx += self.offset
        # Tomogram arrays use [x, y], while the native planner receives
        # [x_idx, y_idx] after the historical axis swap in this project.
        idx = np.array([raw_idx[1], raw_idx[0]], dtype=np.int32)
        if not (0 <= idx[0] < self.map_dim[1] and 0 <= idx[1] < self.map_dim[0]):
            raise ValueError(
                "XY position %s is outside map bounds; planner index=%s, map dimensions=%s"
                % (pos.tolist(), idx.tolist(), self.map_dim)
            )
        return idx

    def position_to_index(self, position, label="position"):
        """Convert a world XY(Z) position to native [layer, x, y] indices."""
        position = np.asarray(position, dtype=np.float64)
        if position.shape not in ((2,), (3,)):
            raise ValueError("%s position must contain [x, y] or [x, y, z]" % label)

        xy_idx = self.pos2idx(position[:2])
        if position.shape == (2,):
            layer = 0
            requested_height = None
            matched_height = None
        else:
            if self.elev_g_raw is None:
                raise RuntimeError("Tomogram ground elevations are not loaded")
            requested_height = float(position[2])
            # Convert native [x_idx, y_idx] back to tomogram [x, y] order.
            raw_x, raw_y = int(xy_idx[1]), int(xy_idx[0])
            heights = self.elev_g_raw[:, raw_x, raw_y]
            valid = np.isfinite(heights) & (heights > -50.0)
            if not np.any(valid):
                raise ValueError(
                    "%s height %.3f has no valid elevation layer at grid cell (%d, %d)"
                    % (label.capitalize(), requested_height, raw_x, raw_y)
                )
            valid_layers = np.flatnonzero(valid)
            valid_heights = heights[valid]

            # A world point can be above a walkable surface (for example a
            # goal specified in mid-air).  Select the highest valid surface
            # that is not above the requested height, rather than the layer
            # with the smallest absolute height error.  This projects the
            # point vertically down onto the nearest walkable floor.
            below = valid_heights <= requested_height + 1e-6
            if not np.any(below):
                raise ValueError(
                    "%s height %.3f is below the lowest valid surface at grid cell (%d, %d); "
                    "lowest height=%.3f"
                    % (
                        label.capitalize(), requested_height, raw_x, raw_y,
                        float(np.min(valid_heights)),
                    )
                )

            candidate_layers = valid_layers[below]
            candidate_heights = valid_heights[below]
            # np.argmax returns the first maximum, so equal-height duplicate
            # layers retain the existing lower-layer tie-break behavior.
            best = int(np.argmax(candidate_heights))
            layer = int(candidate_layers[best])
            matched_height = float(candidate_heights[best])

        index = np.array([layer, xy_idx[0], xy_idx[1]], dtype=np.int32)
        if requested_height is None:
            print(
                "%s world XY: %s -> index: %s (layer 0 legacy mode)"
                % (label.capitalize(), position[:2].tolist(), index.tolist()),
                flush=True,
            )
        else:
            print(
                "%s world XYZ: %s -> index: %s, selected surface height=%.3f "
                "(target is %.3f m above surface)"
                % (
                    label.capitalize(), position.tolist(), index.tolist(),
                    matched_height, requested_height - matched_height,
                ),
                flush=True,
            )
        return index
