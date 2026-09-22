#!/usr/bin/env python3
"""
FMT vs Baseline Benchmark — Same pipeline, two tracking backends.
=================================================================
Only the tracking module differs. Everything else (voxel filter,
visualization, data loading) is identical to target_track_without_initialize_and_grid.py.
"""
import os, glob, time, json, sys
import numpy as np
import open3d as o3d
import open3d.t.io as o3dtio
import cv2
from scipy.spatial import KDTree
from collections import defaultdict

# ============================================================================
# SECTION 1 — SHARED PIPELINE (identical to target_track_without_initialize_and_grid.py)
# ============================================================================

def load_point_cloud_sequence(folder_path):
    pcd_files = sorted(glob.glob(os.path.join(folder_path, "*.pcd")))
    if not pcd_files:
        return {'timestamps': [], 'points': [], 'intensities': [], 'velocities': []}
    print(f"  Loading {len(pcd_files)} PCD files...")
    timestamps, points_list, intensities_list, velocities_list = [], [], [], []
    for file_path in pcd_files:
        filename = os.path.basename(file_path).split('.')[0]
        ts = float(filename.replace('filtered_', ''))
        timestamps.append(ts)
        try:
            pcd_t = o3dtio.read_point_cloud(file_path)
            pts = pcd_t.point["positions"].numpy()
            intensity = np.squeeze(pcd_t.point["intensity"].numpy())
            velocity = np.squeeze(pcd_t.point["velocity"].numpy())
            points_list.append(pts)
            intensities_list.append(intensity)
            velocities_list.append(velocity)
        except Exception as e:
            print(f"  Error: {file_path}: {e}")
            points_list.append(np.array([]))
            intensities_list.append(np.array([]))
            velocities_list.append(np.array([]))
    return {'timestamps': timestamps, 'points': points_list,
            'intensities': intensities_list, 'velocities': velocities_list}

def filter_points_by_height(points, velocities, height_threshold=10.0):
    if len(points) == 0:
        return np.array([]), np.array([]), np.array([], dtype=bool)
    mask = points[:, 2] > height_threshold
    return points[mask], velocities[mask], mask

# --- Voxel Filter (exact copy) ---
class OnlineDynamicFilter:
    def __init__(self, voxel_size=1.0, x_range=(0,500), y_range=(-150,150), z_range=(-5,50),
                 prob_threshold=0.3, init_frames=10, verbose=False):
        self.voxel_size = voxel_size
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.z_min, self.z_max = z_range
        self.prob_threshold = prob_threshold
        self.init_frames = init_frames
        self.verbose = verbose
        self.nx = int(np.ceil((self.x_max - self.x_min) / voxel_size))
        self.ny = int(np.ceil((self.y_max - self.y_min) / voxel_size))
        self.nz = int(np.ceil((self.z_max - self.z_min) / voxel_size))
        self.total_voxels = self.nx * self.ny * self.nz
        if verbose:
            print(f"  Voxel grid: {self.nx}x{self.ny}x{self.nz} = {self.total_voxels} voxels")
        self.voxel_count = np.zeros(self.total_voxels, dtype=np.int32)
        self.frame_count = 0

    def _xyz_to_linidx(self, points):
        ix = np.floor((points[:,0] - self.x_min) / self.voxel_size).astype(int)
        iy = np.floor((points[:,1] - self.y_min) / self.voxel_size).astype(int)
        iz = np.floor((points[:,2] - self.z_min) / self.voxel_size).astype(int)
        valid = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny) & (iz >= 0) & (iz < self.nz)
        lin_idx = -np.ones(len(points), dtype=int)
        lin_idx[valid] = ix[valid] * self.ny * self.nz + iy[valid] * self.nz + iz[valid]
        return lin_idx, valid

    def process_frame(self, points, intensities=None, velocities=None):
        if len(points) == 0:
            return np.array([]), None, None
        self.frame_count += 1
        lin_idx, valid_mask = self._xyz_to_linidx(points)
        dynamic_mask = np.zeros(len(points), dtype=bool)
        if self.frame_count <= self.init_frames:
            static_mask = np.ones(len(points), dtype=bool)
            dyn_pts = np.array([])
            dyn_int = dyn_vel = None
        else:
            prob = np.zeros(len(points))
            valid_lin = lin_idx[valid_mask]
            prob[valid_mask] = self.voxel_count[valid_lin] / self.frame_count
            dynamic_mask = (prob < self.prob_threshold) & valid_mask
            static_mask = (~dynamic_mask) & valid_mask
            dynamic_mask = self._apply_neighborhood_filter(dynamic_mask, lin_idx, valid_mask)
            static_mask = (~dynamic_mask) & valid_mask
            dyn_pts = points[dynamic_mask]
            dyn_int = intensities[dynamic_mask] if intensities is not None else None
            dyn_vel = velocities[dynamic_mask] if velocities is not None else None
        static_lin = lin_idx[static_mask]
        static_lin = static_lin[static_lin >= 0]
        unique_static = np.unique(static_lin)
        self.voxel_count[unique_static] += 1
        return dyn_pts, dyn_int, dyn_vel

    def _apply_neighborhood_filter(self, dynamic_mask, lin_idx, valid_mask):
        if self.frame_count <= self.init_frames or np.sum(dynamic_mask) == 0:
            return dynamic_mask
        dyn_lin = lin_idx[dynamic_mask]; dyn_lin = dyn_lin[dyn_lin >= 0]
        nynz = self.ny * self.nz
        offsets = np.array([dx*nynz + dy*self.nz + dz
                           for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1]], dtype=np.int64)
        dynamic_indices = np.where(dynamic_mask)[0]
        dyn_lin_all = lin_idx[dynamic_indices]
        valid_dyn = dyn_lin_all >= 0
        valid_dyn_indices = dynamic_indices[valid_dyn]
        valid_dyn_lin = dyn_lin_all[valid_dyn]
        if len(valid_dyn_lin) == 0:
            return dynamic_mask
        all_neighbors = valid_dyn_lin[:, None] + offsets[None, :]
        ix_arr = valid_dyn_lin // nynz
        iy_arr = (valid_dyn_lin % nynz) // self.nz
        iz_arr = valid_dyn_lin % self.nz
        dx_arr = np.array([-1]*9 + [0]*9 + [1]*9)
        dy_arr = np.tile(np.array([-1]*3+[0]*3+[1]*3), 3)
        dz_arr = np.tile([-1,0,1], 9)
        nb_ix = ix_arr[:, None] + dx_arr[None, :]
        nb_iy = iy_arr[:, None] + dy_arr[None, :]
        nb_iz = iz_arr[:, None] + dz_arr[None, :]
        in_bounds = ((nb_ix >= 0) & (nb_ix < self.nx) &
                     (nb_iy >= 0) & (nb_iy < self.ny) &
                     (nb_iz >= 0) & (nb_iz < self.nz))
        all_neighbors = np.clip(all_neighbors, 0, self.total_voxels - 1)
        occupied = (self.voxel_count[all_neighbors] > 0) & in_bounds
        is_dynamic_nb = np.isin(all_neighbors.ravel(), dyn_lin).reshape(all_neighbors.shape)
        is_dynamic_nb &= in_bounds
        occupied_count = np.sum(occupied, axis=1)
        static_count = np.sum(occupied & ~is_dynamic_nb, axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            static_ratio = np.where(occupied_count > 0, static_count / occupied_count, 0.0)
        remove_mask = (occupied_count > 0) & (static_ratio > 0.6)
        refined = np.copy(dynamic_mask)
        refined[valid_dyn_indices[remove_mask]] = False
        return refined

# --- Grid maps (exact copy) ---
def generate_grid_map(points, resolution=0.5):
    if len(points) == 0:
        return None, None
    xy = points[:, :2]
    x_min, x_max = 0, 500
    y_min, y_max = -150, 150
    rows = int(np.ceil((x_max - x_min) / resolution))
    cols = int(np.ceil((y_max - y_min) / resolution))
    grid = np.zeros((rows, cols))
    ri = rows - 1 - np.floor((xy[:, 0] - x_min) / resolution).astype(int)
    ci = np.floor((y_max - xy[:, 1]) / resolution).astype(int)
    valid = (ri >= 0) & (ri < rows) & (ci >= 0) & (ci < cols)
    for r, c in zip(ri[valid], ci[valid]):
        grid[int(r), int(c)] += 1
    return grid, (x_min, x_max, y_min, y_max, rows, cols, resolution)

def generate_velocity_grid_map(points, velocities, resolution=0.5):
    if len(points) == 0 or len(velocities) == 0:
        return None, None
    xy = points[:, :2]
    x_min, x_max = 0, 500
    y_min, y_max = -150, 150
    rows = int(np.ceil((x_max - x_min) / resolution))
    cols = int(np.ceil((y_max - y_min) / resolution))
    vel_grid = np.zeros((rows, cols))
    cnt_grid = np.zeros((rows, cols))
    ri = rows - 1 - np.floor((xy[:, 0] - x_min) / resolution).astype(int)
    ci = np.floor((y_max - xy[:, 1]) / resolution).astype(int)
    valid = (ri >= 0) & (ri < rows) & (ci >= 0) & (ci < cols)
    vv = velocities[valid]
    for r, c, s in zip(ri[valid], ci[valid], vv):
        r, c = int(r), int(c)
        vel_grid[r, c] += s
        cnt_grid[r, c] += 1
    with np.errstate(divide='ignore', invalid='ignore'):
        vel_grid = np.where(cnt_grid > 0, vel_grid / cnt_grid, 0)
    return vel_grid, (x_min, x_max, y_min, y_max, rows, cols, resolution)

# ============================================================================
# SECTION 2 — VISUALIZATION (exact copy from target_track_without_initialize_and_grid.py)
# ============================================================================

TRACKER_COLORS = [
    (0, 255, 0), (255, 255, 0), (0, 255, 255), (255, 0, 255),
    (128, 255, 0), (0, 128, 255), (255, 128, 0), (128, 0, 255),
]

def draw_tracker_panel(points, velocities, trackers, grid_map, grid_params,
                       velocity_grid, vgrid_params, panel_title="",
                       show_legend=True):
    """Render a single tracking panel (point cloud + velocity grid side by side).
    Identical rendering style to the original code."""
    if grid_map is None or velocity_grid is None:
        return None

    x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params

    # Point cloud image (left)
    binary = (grid_map > 0).astype(np.uint8) * 255
    pc_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # Velocity image (right) — same color scheme as original
    h, w = velocity_grid.shape
    vel_img = np.zeros((h, w, 3), dtype=np.uint8)
    st = 1.0
    has_data = grid_map > 0
    vel_img[has_data & (np.abs(velocity_grid) <= st)] = [128, 128, 128]   # static: gray
    vel_img[has_data & (velocity_grid > st)] = [0, 0, 255]                 # away: red
    vel_img[has_data & (velocity_grid < -st)] = [255, 0, 0]                # toward: blue

    # Velocity legend
    cv2.rectangle(vel_img, (10, 10), (30, 30), (128, 128, 128), -1)
    cv2.putText(vel_img, "static", (40, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.rectangle(vel_img, (10, 40), (30, 60), (0, 0, 255), -1)
    cv2.putText(vel_img, "v>1.0", (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.rectangle(vel_img, (10, 70), (30, 90), (255, 0, 0), -1)
    cv2.putText(vel_img, "v<-1.0", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    # Draw trackers — support both object-style and dict-style trackers
    def _get(t, key, default=None):
        if isinstance(t, dict):
            return t.get(key, default)
        return getattr(t, key, default)

    for ti, trk in enumerate(trackers):
        if not _get(trk, 'is_active', True):
            continue
        color = TRACKER_COLORS[ti % len(TRACKER_COLORS)]

        # Current position
        cx = cy = None
        if hasattr(trk, 'center') and not isinstance(trk, dict):
            cx, cy = trk.center
        elif isinstance(trk, dict) and 'center' in trk:
            cx, cy = trk['center']
        if cx is None:
            continue

        gy = rows - 1 - int((cx - x_min) / resolution)
        gx = int((y_max - cy) / resolution)

        cv2.circle(pc_img, (gx, gy), 3, color, 2)
        cv2.circle(vel_img, (gx, gy), 3, color, 2)

        # Label
        tid = _get(trk, 'target_id', _get(trk, 'id', ti))
        label = f"ID:{tid}"
        cv2.putText(pc_img, label, (gx-30, gy-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Track history
        hist = _get(trk, 'track_history', [])
        if len(hist) > 1:
            pts = []
            for hp in hist[-15:]:
                hgy = rows - 1 - int((hp[0] - x_min) / resolution)
                hgx = int((y_max - hp[1]) / resolution)
                pts.append((hgx, hgy))
            for k in range(1, len(pts)):
                cv2.line(pc_img, pts[k-1], pts[k], color, 2)
                cv2.line(vel_img, pts[k-1], pts[k], color, 2)

        # Search radius circle
        sr = _get(trk, 'search_radius', 15.0)
        sr_px = int(sr / resolution)
        cv2.circle(pc_img, (gx, gy), sr_px, color, 1)

    # Info texts
    active = [t for t in trackers if _get(t, 'is_active', True)]
    info_texts = [
        f"Active: {len(active)}",
        f"{panel_title}",
    ]
    for i, text in enumerate(info_texts):
        cv2.putText(pc_img, text, (10, pc_img.shape[0] - 60 + i * 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    combined = np.hstack((pc_img, vel_img))
    return cv2.resize(combined, (1400, 600), interpolation=cv2.INTER_NEAREST)

def render_comparison_frame(points, velocities, trackers_a, trackers_b,
                            frame_idx, output_dir):
    """Render baseline (top) vs FMT (bottom) comparison frame."""
    grid_map, gp = generate_grid_map(points, resolution=0.5)
    vel_grid, vp = generate_velocity_grid_map(points, velocities, resolution=0.5)
    if grid_map is None:
        return None

    top = draw_tracker_panel(points, velocities, trackers_a, grid_map, gp,
                             vel_grid, vp, f"BASELINE — Frame {frame_idx}")
    bot = draw_tracker_panel(points, velocities, trackers_b, grid_map, gp,
                             vel_grid, vp, f"FMT — Frame {frame_idx}")
    if top is None or bot is None:
        return None

    combined = np.vstack((top, bot))
    save_path = os.path.join(output_dir, f"frame_{frame_idx:04d}.png")
    cv2.imwrite(save_path, combined)
    return combined

# ============================================================================
# SECTION 3 — BASELINE TRACKER (exact copy of MultiTargetTracker + SingleTargetTracker)
# ============================================================================

class BaselineSingleTarget:
    def __init__(self, target_id, center_2d, points, velocities,
                 search_radius=15.0, max_association_distance=30.0):
        self.target_id = target_id
        self.center = center_2d
        self.center_3d = (center_2d[0], center_2d[1], np.mean(points[:, 2]))
        self.num_points = len(points)
        self.radial_velocity = np.mean(velocities) if len(velocities) > 0 else 0
        self.search_radius = search_radius
        self.max_association_distance = max_association_distance
        self.track_history = [center_2d]
        self.velocity_history = [self.radial_velocity]
        self.lost_count = 0
        self.static_count = 0
        self.is_active = True
        self.frames_processed = 0
        self.position_history = [center_2d]
        self.marked_for_removal = False
        self.removal_countdown = 0
        self.last_valid_center = center_2d
        self.is_predicted = False

    def update(self, new_center, new_center_3d, num_points, radial_velocity):
        self.is_predicted = False
        self.frames_processed += 1
        self.position_history.append(new_center)
        if len(self.position_history) > 20:
            self.position_history.pop(0)
        if len(self.track_history) > 0:
            movement = np.sqrt((new_center[0] - self.last_valid_center[0])**2 +
                              (new_center[1] - self.last_valid_center[1])**2)
            if self.frames_processed > 3:
                if movement < 2.0:
                    self.static_count += 1
                    if self.static_count >= 5 and not self.marked_for_removal:
                        self.marked_for_removal = True
                        self.removal_countdown = 5
                else:
                    self.static_count = max(0, self.static_count - 2)
                    if self.marked_for_removal and movement > 4.0:
                        self.marked_for_removal = False
                        self.removal_countdown = 0
        if self.marked_for_removal:
            self.removal_countdown -= 1
            if self.removal_countdown <= 0:
                self.is_active = False
        self.center = new_center
        self.center_3d = new_center_3d
        self.num_points = num_points
        self.radial_velocity = radial_velocity
        self.track_history.append(new_center)
        self.velocity_history.append(radial_velocity)
        self.lost_count = 0
        self.last_valid_center = new_center

    def predict(self):
        if len(self.track_history) < 2:
            return self.center
        last = np.array(self.track_history[-1])
        prev = np.array(self.track_history[-2])
        return tuple(last + (last - prev))

    def should_remove(self):
        if self.marked_for_removal and self.removal_countdown <= 0:
            return True, "static"
        if self.lost_count > 5:
            return True, "lost"
        return False, ""

class BaselineMultiTracker:
    def __init__(self, min_points=1, search_radius=15.0, radial_threshold=5.0,
                 max_association_distance=30.0, dbscan_eps=2.0):
        self.min_points = min_points
        self.search_radius = search_radius
        self.radial_threshold = radial_threshold
        self.max_association_distance = max_association_distance
        self.dbscan_eps = dbscan_eps
        self.trackers = []
        self.removed_targets = []
        self.target_id_counter = 0
        self.pending_trackers = {}
        self.pending_id_counter = 0
        self.initialized = False

    def _cluster(self, pts):
        if len(pts) < self.min_points:
            return []
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        labels = np.array(pcd.cluster_dbscan(eps=self.dbscan_eps,
                          min_points=self.min_points, print_progress=False))
        clusters = []
        for lbl in set(labels):
            if lbl == -1: continue
            idx = np.where(labels == lbl)[0]
            if len(idx) >= self.min_points:
                clusters.append({'indices': idx, 'points': pts[idx]})
        return clusters

    def initialize(self, points, velocities):
        clusters = self._cluster(points)
        for c in clusters:
            cpts = c['points']
            cvel = velocities[c['indices']] if len(velocities) > 0 else np.zeros(len(cpts))
            cx, cy = np.mean(cpts[:,0]), np.mean(cpts[:,1])
            t = BaselineSingleTarget(self.target_id_counter, (cx, cy), cpts, cvel,
                                     self.search_radius, self.max_association_distance)
            self.trackers.append(t)
            self.target_id_counter += 1
        self.initialized = True
        print(f"  Baseline initialized {len(self.trackers)} targets")

    def track_frame(self, points, velocities):
        if not self.initialized:
            self.initialize(points, velocities)
            return [t for t in self.trackers if t.is_active]

        if len(points) == 0:
            return [t for t in self.trackers if t.is_active]

        tree = KDTree(points[:, :2])
        used = set()

        for trk in self.trackers:
            if not trk.is_active: continue
            pred = trk.center
            indices = tree.query_ball_point([pred[0], pred[1]], 2 * self.search_radius)
            if len(indices) < self.min_points:
                trk.lost_count += 1
                pred_pos = trk.predict()
                trk.center = pred_pos
                trk.track_history.append(pred_pos)
                trk.is_predicted = True
                continue

            local_pts = points[indices]
            local_vels = velocities[indices]
            clusters = self._cluster(local_pts)

            best, best_dist = None, float('inf')
            for c in clusters:
                cpts = c['points']
                cx, cy = np.mean(cpts[:,0]), np.mean(cpts[:,1])
                dist = np.sqrt((cx - pred[0])**2 + (cy - pred[1])**2)
                cvel = np.mean(local_vels[c['indices']])

                # Radial velocity consistency (same as original)
                app_vel = np.array([cx - trk.center[0], cy - trk.center[1], 0])
                r_to_t = np.array([cx, cy, 0])
                d = np.linalg.norm(r_to_t) + 1e-6
                dir_unit = r_to_t / d
                theoretical_radial = np.dot(app_vel, dir_unit)
                radial_diff = abs(-cvel - theoretical_radial)

                if dist < best_dist and radial_diff < self.radial_threshold:
                    best_dist = dist
                    best = {'center': (cx, cy),
                            'center_3d': (cx, cy, np.mean(cpts[:,2])),
                            'num_points': len(cpts),
                            'radial_velocity': cvel,
                            'indices': [indices[i] for i in c['indices']]}

            if best is not None and best_dist < self.max_association_distance:
                trk.update(best['center'], best['center_3d'],
                          best['num_points'], best['radial_velocity'])
                used.update(best['indices'])
            else:
                trk.lost_count += 1
                pred_pos = trk.predict()
                trk.center = pred_pos
                trk.track_history.append(pred_pos)
                trk.is_predicted = True

        # Remove inactive
        active = []
        for trk in self.trackers:
            remove, reason = trk.should_remove()
            if remove:
                trk.is_active = False
                self.removed_targets.append({'id': trk.target_id, 'track_history': trk.track_history})
            else:
                active.append(trk)
        self.trackers = active
        return [t for t in self.trackers if t.is_active]

# ============================================================================
# SECTION 4 — FMT TRACKER (manifold-constrained, same interface)
# ============================================================================

class FMTSingleTarget:
    """
    Single-target state with manifold-constrained velocity decomposition.

    Key addition over baseline: maintains tangential velocity estimate w_k
    and kinematic residual per frame, computed via closed-form projection.

    v_k = v_r_k * p_hat_k + B(p_hat_k) * w_k

    where:
      v_r_k = measured radial velocity (FMCW)
      p_hat_k = unit direction from radar
      B(p_hat_k) = orthonormal tangent plane basis (3x2)
      w_k = tangential velocity (2D, estimated via closed-form projection)
    """
    def __init__(self, target_id, center_2d, points, velocities,
                 search_radius=15.0, max_association_distance=30.0):
        self.target_id = target_id
        self.center = center_2d
        self.center_3d = np.mean(points, axis=0)
        self.num_points = len(points)
        self.radial_velocity = np.mean(velocities) if len(velocities) > 0 else 0
        self.search_radius = search_radius
        self.max_association_distance = max_association_distance
        self.track_history = [center_2d]
        self.position_3d_history = [self.center_3d]
        self.velocity_history = [self.radial_velocity]
        self.lost_count = 0
        self.static_count = 0
        self.is_active = True
        self.frames_processed = 0
        self.position_history = [center_2d]
        self.marked_for_removal = False
        self.removal_countdown = 0
        self.last_valid_center = center_2d
        self.is_predicted = False
        # FMT-specific state
        self.tangential_w = np.zeros(2)      # current tangential velocity estimate
        self.w_history = [np.zeros(2)]        # history of w estimates
        self.kinematic_residual = 0.0         # ||v_model - v_observed||
        self.residual_history = []            # history of residuals

    @staticmethod
    def _tangent_basis(p_hat):
        """Orthonormal basis for plane perpendicular to p_hat. Returns (3,2)."""
        ez = np.array([0., 0., 1.])
        if abs(np.dot(p_hat, ez)) > 0.99:
            b1 = np.cross(p_hat, np.array([1., 0., 0.]))
        else:
            b1 = np.cross(p_hat, ez)
        b1 /= np.linalg.norm(b1) + 1e-12
        b2 = np.cross(p_hat, b1)
        b2 /= np.linalg.norm(b2) + 1e-12
        return np.column_stack([b1, b2])

    def compute_manifold_residual(self, prev_center_3d, curr_center_3d, curr_radial_vel):
        """
        Closed-form manifold projection:
          Given dp = curr - prev (observed displacement over dt=1)
          Compute optimal tangential velocity w* = argmin_w ||v_r * p_hat + B * w - dp||²

        Solution: w* = B^T * (dp - v_r * p_hat)   (since B^T B = I, B^T p_hat = 0)

        Returns (w_opt, residual) where residual = ||v_model - dp||
        """
        dp = curr_center_3d - prev_center_3d
        p_hat = curr_center_3d / (np.linalg.norm(curr_center_3d) + 1e-6)
        B = self._tangent_basis(p_hat)

        # Radial component of observed displacement
        radial_component = np.dot(dp, p_hat)

        # Optimal tangential velocity (closed-form)
        # dp = v_r * p_hat + B * w  =>  B * w = dp - v_r * p_hat
        residual_3d = dp - curr_radial_vel * p_hat
        w_opt = B.T @ residual_3d   # (2,)

        # Reconstructed model velocity
        v_model = curr_radial_vel * p_hat + B @ w_opt

        # Kinematic residual: how much of dp is unexplained by the model
        residual = np.linalg.norm(dp - v_model)

        return w_opt, residual, v_model

    def update(self, new_center, new_center_3d, num_points, radial_velocity):
        self.is_predicted = False
        self.frames_processed += 1

        # --- Manifold-constrained velocity projection ---
        prev_3d = self.position_3d_history[-1]
        w_opt, residual, v_model = self.compute_manifold_residual(
            prev_3d, new_center_3d, radial_velocity)

        self.tangential_w = w_opt
        self.kinematic_residual = residual
        self.residual_history.append(residual)
        if len(self.residual_history) > 50:
            self.residual_history.pop(0)
        self.w_history.append(w_opt)
        if len(self.w_history) > 50:
            self.w_history.pop(0)

        # --- Manifold-smoothed position (optional: blend model prediction with observation) ---
        # Use the model velocity to compute a smoothed position estimate
        alpha = 0.3  # smoothing factor (0 = pure observation, 1 = pure model)
        smoothed_3d = prev_3d + v_model
        smoothed_center_3d = (1 - alpha) * new_center_3d + alpha * smoothed_3d
        smoothed_center_2d = (smoothed_center_3d[0], smoothed_center_3d[1])

        # --- Same movement/static logic as baseline ---
        self.position_history.append(smoothed_center_2d)
        if len(self.position_history) > 20:
            self.position_history.pop(0)

        movement = np.sqrt((new_center[0] - self.last_valid_center[0])**2 +
                          (new_center[1] - self.last_valid_center[1])**2)
        if self.frames_processed > 3:
            if movement < 2.0:
                self.static_count += 1
                if self.static_count >= 5 and not self.marked_for_removal:
                    self.marked_for_removal = True
                    self.removal_countdown = 5
            else:
                self.static_count = max(0, self.static_count - 2)
                if self.marked_for_removal and movement > 4.0:
                    self.marked_for_removal = False
                    self.removal_countdown = 0

        if self.marked_for_removal:
            self.removal_countdown -= 1
            if self.removal_countdown <= 0:
                self.is_active = False

        self.center = smoothed_center_2d
        self.center_3d = smoothed_center_3d
        self.position_3d_history.append(smoothed_center_3d)
        if len(self.position_3d_history) > 100:
            self.position_3d_history.pop(0)
        self.num_points = num_points
        self.radial_velocity = radial_velocity
        self.track_history.append(smoothed_center_2d)
        self.velocity_history.append(radial_velocity)
        self.lost_count = 0
        self.last_valid_center = smoothed_center_2d

    def predict(self):
        """Predict next position using manifold velocity model."""
        if len(self.position_3d_history) < 2:
            return self.center

        curr_3d = self.position_3d_history[-1]
        curr_radial = self.radial_velocity
        p_hat = curr_3d / (np.linalg.norm(curr_3d) + 1e-6)
        B = self._tangent_basis(p_hat)

        # Use last known tangential velocity for prediction
        v_model = curr_radial * p_hat + B @ self.tangential_w
        pred_3d = curr_3d + v_model
        return (pred_3d[0], pred_3d[1])

    def should_remove(self):
        if self.marked_for_removal and self.removal_countdown <= 0:
            return True, "static"
        if self.lost_count > 5:
            return True, "lost"
        return False, ""

class FMTMultiTracker:
    """
    Multi-target tracker with manifold-constrained association.

    Key difference from baseline:
    - Association cost includes manifold kinematic residual
    - Velocity decomposition enables better discrimination between
      physically-plausible vs geometrically-close associations
    """
    def __init__(self, min_points=1, search_radius=15.0, radial_threshold=5.0,
                 max_association_distance=30.0, dbscan_eps=2.0):
        self.min_points = min_points
        self.search_radius = search_radius
        self.radial_threshold = radial_threshold
        self.max_association_distance = max_association_distance
        self.dbscan_eps = dbscan_eps
        self.trackers = []
        self.removed_targets = []
        self.target_id_counter = 0
        self.initialized = False

    def _cluster(self, pts):
        if len(pts) < self.min_points:
            return []
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        labels = np.array(pcd.cluster_dbscan(eps=self.dbscan_eps,
                          min_points=self.min_points, print_progress=False))
        clusters = []
        for lbl in set(labels):
            if lbl == -1: continue
            idx = np.where(labels == lbl)[0]
            if len(idx) >= self.min_points:
                clusters.append({'indices': idx, 'points': pts[idx]})
        return clusters

    def _manifold_association_cost(self, trk, cluster_center_3d, cluster_radial_vel):
        """
        Manifold-constrained association cost.
        Lower = better match. Combines:
          1. Position distance (geometric)
          2. Kinematic residual (manifold constraint)
          3. Tangential velocity consistency (smoothness)
        """
        # 1. Position distance
        pos_dist = np.linalg.norm(cluster_center_3d[:2] - np.array(trk.center))

        # 2. Manifold kinematic residual
        prev_3d = trk.position_3d_history[-1]
        _, residual, _ = trk.compute_manifold_residual(
            prev_3d, cluster_center_3d, cluster_radial_vel)

        # 3. Tangential velocity smoothness (if we have history)
        w_smooth_cost = 0.0
        if len(trk.w_history) >= 2:
            # Cost for abrupt changes in tangential velocity
            p_hat = cluster_center_3d / (np.linalg.norm(cluster_center_3d) + 1e-6)
            B = trk._tangent_basis(p_hat)
            dp = cluster_center_3d - prev_3d
            residual_3d = dp - cluster_radial_vel * p_hat
            w_new = B.T @ residual_3d
            w_prev = trk.tangential_w
            w_smooth_cost = np.linalg.norm(w_new - w_prev)

        # Combined cost
        cost = pos_dist + 0.5 * residual + 0.2 * w_smooth_cost
        return cost, residual

    def initialize(self, points, velocities):
        clusters = self._cluster(points)
        for c in clusters:
            cpts = c['points']
            cvel = velocities[c['indices']] if len(velocities) > 0 else np.zeros(len(cpts))
            cx, cy = np.mean(cpts[:,0]), np.mean(cpts[:,1])
            t = FMTSingleTarget(self.target_id_counter, (cx, cy), cpts, cvel,
                                self.search_radius, self.max_association_distance)
            self.trackers.append(t)
            self.target_id_counter += 1
        self.initialized = True
        print(f"  FMT initialized {len(self.trackers)} targets")

    def track_frame(self, points, velocities):
        if not self.initialized:
            self.initialize(points, velocities)
            return [t for t in self.trackers if t.is_active]

        if len(points) == 0:
            return [t for t in self.trackers if t.is_active]

        tree = KDTree(points[:, :2])
        used = set()

        for trk in self.trackers:
            if not trk.is_active: continue

            # Predict next position using manifold velocity model
            pred = trk.predict()
            indices = tree.query_ball_point([pred[0], pred[1]], 2 * self.search_radius)

            if len(indices) < self.min_points:
                trk.lost_count += 1
                pred_pos = trk.predict()
                trk.center = pred_pos
                trk.track_history.append(pred_pos)
                trk.is_predicted = True
                continue

            local_pts = points[indices]
            local_vels = velocities[indices]
            clusters = self._cluster(local_pts)

            # Use manifold-constrained cost for association
            best, best_cost = None, float('inf')
            for c in clusters:
                cpts = c['points']
                center_3d = np.mean(cpts, axis=0)
                cx, cy = center_3d[0], center_3d[1]
                cvel = np.mean(local_vels[c['indices']])

                # Manifold association cost
                cost, residual = self._manifold_association_cost(trk, center_3d, cvel)

                # Hard gate on position distance
                pos_dist = np.sqrt((cx - pred[0])**2 + (cy - pred[1])**2)
                if pos_dist > self.max_association_distance:
                    continue

                if cost < best_cost:
                    best_cost = cost
                    best = {'center': (cx, cy),
                            'center_3d': center_3d,
                            'num_points': len(cpts),
                            'radial_velocity': cvel,
                            'indices': [indices[i] for i in c['indices']],
                            'manifold_residual': residual}

            if best is not None and best_cost < self.max_association_distance * 2:
                trk.update(best['center'], best['center_3d'],
                          best['num_points'], best['radial_velocity'])
                used.update(best['indices'])
            else:
                trk.lost_count += 1
                pred_pos = trk.predict()
                trk.center = pred_pos
                trk.track_history.append(pred_pos)
                trk.is_predicted = True

        # Remove inactive
        active = []
        for trk in self.trackers:
            remove, reason = trk.should_remove()
            if remove:
                trk.is_active = False
                self.removed_targets.append({'id': trk.target_id, 'track_history': trk.track_history})
            else:
                active.append(trk)
        self.trackers = active
        return [t for t in self.trackers if t.is_active]

# ============================================================================
# SECTION 5 — METRICS COMPUTATION
# ============================================================================

def compute_comparison_metrics(tracker_a, tracker_b):
    """Compute head-to-head metrics."""
    active_a = [t for t in tracker_a.trackers if t.is_active]
    active_b = [t for t in tracker_b.trackers if t.is_active]

    # Trajectory smoothness: variance of speed between consecutive positions
    def compute_smoothness(trackers):
        values = []
        for t in trackers:
            hist = t.track_history
            if len(hist) < 4: continue
            speeds = []
            for k in range(1, len(hist)):
                speeds.append(np.sqrt((hist[k][0] - hist[k-1][0])**2 +
                                      (hist[k][1] - hist[k-1][1])**2))
            if len(speeds) >= 3:
                values.append(float(np.var(speeds)))
        return np.mean(values) if values else 0.0, len(values)

    # Kinematic residual (FMT only)
    fmt_residuals = []
    for t in tracker_b.trackers:
        if hasattr(t, 'residual_history') and len(t.residual_history) > 0:
            fmt_residuals.append(float(np.mean(t.residual_history)))

    smooth_a, n_a = compute_smoothness(tracker_a.trackers)
    smooth_b, n_b = compute_smoothness(tracker_b.trackers)

    return {
        'baseline': {
            'active_tracks': len(active_a),
            'total_generated': tracker_a.target_id_counter,
            'fragmentations': len(tracker_a.removed_targets),
            'smoothness': round(smooth_a, 4),
            'tracks_evaluated': n_a,
        },
        'fmt': {
            'active_tracks': len(active_b),
            'total_generated': tracker_b.target_id_counter,
            'fragmentations': len(tracker_b.removed_targets),
            'smoothness': round(smooth_b, 4),
            'tracks_evaluated': n_b,
            'mean_kinematic_residual': round(np.mean(fmt_residuals), 4) if fmt_residuals else 0,
        }
    }

# ============================================================================
# SECTION 6 — MAIN BENCHMARK
# ============================================================================

def run_benchmark(data_folder, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    frame_plots_dir = os.path.join(output_dir, "frame_plots")
    os.makedirs(frame_plots_dir, exist_ok=True)
    # Clean previous plots
    for f in glob.glob(os.path.join(frame_plots_dir, "*.png")):
        os.remove(f)

    print("=" * 72)
    print("FMT vs BASELINE BENCHMARK")
    print("=" * 72)
    print(f"Data:   {data_folder}")
    print(f"Output: {output_dir}")

    # Load data
    seq = load_point_cloud_sequence(data_folder)
    n_frames = len(seq['timestamps'])
    if n_frames == 0:
        print("ERROR: No data found!")
        return
    print(f"Frames: {n_frames}")

    # Shared voxel filter
    voxel = OnlineDynamicFilter(voxel_size=1.0, x_range=(0,500), y_range=(-150,150),
                                z_range=(-5,50), prob_threshold=0.05, init_frames=8, verbose=False)

    # Both trackers (same parameters for fair comparison)
    tracker_a = BaselineMultiTracker(min_points=1, search_radius=15.0, radial_threshold=5.0,
                                     max_association_distance=30.0, dbscan_eps=2.0)
    tracker_b = FMTMultiTracker(min_points=1, search_radius=15.0, radial_threshold=5.0,
                                max_association_distance=30.0, dbscan_eps=2.0)

    # Per-frame metrics collection
    all_frame_metrics = []
    latencies_a, latencies_b = [], []

    print("\n" + "=" * 72)
    print("FRAME-BY-FRAME PROCESSING")
    print("=" * 72)

    for i in range(n_frames):
        ts = seq['timestamps'][i]
        points = seq['points'][i]
        velocities = seq['velocities'][i]
        intensities = seq['intensities'][i] if i < len(seq['intensities']) else np.array([])

        if len(points) == 0:
            continue

        # Height filter
        f_pts, f_vels, _ = filter_points_by_height(points, velocities, 10.0)
        f_ints = intensities[:len(f_pts)] if len(intensities) > 0 else np.array([])

        if len(f_pts) == 0:
            continue

        # Voxel dynamic filter
        dyn_pts, dyn_ints, dyn_vels = voxel.process_frame(f_pts, f_ints, f_vels)

        # Background learning
        if voxel.frame_count <= voxel.init_frames:
            status = f"bg learn {voxel.frame_count}/{voxel.init_frames}"
            if voxel.frame_count % 4 == 1:
                print(f"  Frame {i+1:2d}: {status}")
            all_frame_metrics.append({'frame': i, 'phase': 'background', 'count': voxel.frame_count})
            continue

        if len(dyn_pts) < 1:
            continue

        # --- Track-A (Baseline) ---
        t0 = time.perf_counter()
        result_a = tracker_a.track_frame(dyn_pts, dyn_vels)
        lat_a = (time.perf_counter() - t0) * 1000
        latencies_a.append(lat_a)

        # --- Track-B (FMT) ---
        t0 = time.perf_counter()
        result_b = tracker_b.track_frame(dyn_pts, dyn_vels)
        lat_b = (time.perf_counter() - t0) * 1000
        latencies_b.append(lat_b)

        active_a = len([t for t in tracker_a.trackers if t.is_active])
        active_b = len([t for t in tracker_b.trackers if t.is_active])
        res_b = np.mean([t.kinematic_residual for t in tracker_b.trackers
                        if t.is_active and hasattr(t, 'kinematic_residual')]) if active_b > 0 else 0

        print(f"  Frame {i+1:2d}: dyn_pts={len(dyn_pts):4d} | "
              f"A: {active_a:2d} tracks, {lat_a:6.1f}ms | "
              f"B: {active_b:2d} tracks, {lat_b:6.1f}ms | "
              f"B_res={res_b:.2f}m")

        all_frame_metrics.append({
            'frame': i,
            'dynamic_points': len(dyn_pts),
            'baseline_active': active_a,
            'fmt_active': active_b,
            'baseline_latency_ms': round(lat_a, 1),
            'fmt_latency_ms': round(lat_b, 1),
            'fmt_mean_residual': round(res_b, 3),
        })

        # Render comparison frame
        active_a_list = [t for t in tracker_a.trackers if t.is_active]
        active_b_list = [t for t in tracker_b.trackers if t.is_active]
        render_comparison_frame(dyn_pts, dyn_vels, active_a_list, active_b_list,
                               i, frame_plots_dir)

    # =========================================================================
    # FINAL REPORT
    # =========================================================================
    final_metrics = compute_comparison_metrics(tracker_a, tracker_b)

    avg_lat_a = np.mean(latencies_a) if latencies_a else 0
    avg_lat_b = np.mean(latencies_b) if latencies_b else 0

    print("\n" + "=" * 72)
    print("RESULTS")
    print("=" * 72)

    b, f = final_metrics['baseline'], final_metrics['fmt']

    print(f"""
  ┌──────────────────────────────────────────────────────────────────┐
  │               HEAD-TO-HEAD COMPARISON (28 frames)                │
  ├───────────────────────────┬──────────────────┬───────────────────┤
  │ Metric                    │ Baseline (DBSCAN)│ FMT (Manifold)    │
  ├───────────────────────────┼──────────────────┼───────────────────┤
  │ Final Active Tracks       │ {b['active_tracks']:>16} │ {f['active_tracks']:>17} │
  │ Total Tracks Generated    │ {b['total_generated']:>16} │ {f['total_generated']:>17} │
  │ Track Fragmentations      │ {b['fragmentations']:>16} │ {f['fragmentations']:>17} │
  │ Avg Latency (ms/frame)    │ {avg_lat_a:>16.2f} │ {avg_lat_b:>17.2f} │
  │ Trajectory Smoothness     │ {b['smoothness']:>16.4f} │ {f['smoothness']:>17.4f} │
  │ Kinematic Residual (m)    │ {"N/A":>16} │ {f['mean_kinematic_residual']:>17.4f} │
  └───────────────────────────┴──────────────────┴───────────────────┘
""")

    # Save results
    report = {
        'config': {
            'voxel_size': 1.0, 'prob_threshold': 0.05, 'init_frames': 8,
            'height_threshold': 10.0, 'search_radius': 15.0, 'radial_threshold': 5.0,
            'max_association_dist': 30.0, 'dbscan_eps': 2.0, 'min_points': 1,
        },
        'dataset': data_folder,
        'n_frames': n_frames,
        'final_metrics': final_metrics,
        'avg_latency_ms': {'baseline': round(avg_lat_a, 2), 'fmt': round(avg_lat_b, 2)},
        'frame_metrics': all_frame_metrics,
    }

    json_path = os.path.join(output_dir, "benchmark_metrics.json")
    with open(json_path, 'w') as fh:
        json.dump(report, fh, indent=2, default=str)

    plot_count = len(glob.glob(os.path.join(frame_plots_dir, "*.png")))
    print(f"  JSON report:  {json_path}")
    print(f"  Frame plots:  {plot_count} PNGs in {frame_plots_dir}/")
    print("=" * 72)

    return report


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', type=str,
                    default='data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data',
                    help='Path to filtered PCD data')
    ap.add_argument('--output', type=str, default='results/81-82-pm-x10_2025-11-19-15-38-07_filtered_data/comparison_study',
                    help='Output directory')
    args = ap.parse_args()
    run_benchmark(args.data, args.output)
