#!/usr/bin/env python3
"""
Manifold Enhancement Modules for FMCW LIDAR Drone Tracking
===========================================================

Module 1: GrassmannDynamicFilter — Incremental SVD background subtraction
Module 2: FMTTrajectorySmoother — Post-tracking Riemannian manifold smoothing

Both modules are designed as DROP-IN components:
  - GrassmannDynamicFilter has the same `process_frame()` interface as
    OnlineDynamicFilter, so it can replace or run in parallel.
  - FMTTrajectorySmoother is a pure post-processor: it takes finalized track
    histories and outputs smoothed trajectories WITHOUT modifying the tracker's
    internal state. No feedback loop.
"""

import numpy as np
from scipy.spatial import KDTree
from scipy.optimize import minimize
from collections import defaultdict

# ============================================================================
# MODULE 1: Grassmann Dynamic Filter (Incremental SVD Background Subtraction)
# ============================================================================

class GrassmannDynamicFilter:
    """
    Per-voxel local subspace background subtraction.

    PRINCIPLE (corrected):
      A global low-rank subspace in R³ is meaningless because:
        - R³ is only 3-dimensional → rank > 3 is degenerate
        - Static structures (ground, trees, buildings) are NOT a single plane or line
        - Different regions have different geometries

    SOLUTION: Per-voxel local model (Local Grassmann / per-voxel PCA):
      - Divide space into voxels (same grid as OnlineDynamicFilter)
      - Each voxel maintains a LOCAL rank-1 model: running mean μ_v ∈ R³
      - The residual ‖x - μ_v‖ measures how much a point deviates from
        the local static structure
      - Large residual → point is dynamic (drone moving through voxel)
      - Mean updated online via exponential moving average for static points

    This is mathematically a Grassmann manifold approach applied LOCALLY:
    each voxel's static point distribution is approximated by a rank-1
    subspace (the mean), and the Grassmann distance is the Euclidean
    residual from that mean.

    Advantages over global SVD:
      - Locality: different regions have different "background" positions
      - Interpretable: residual = distance from voxel center (meters)
      - Robust: per-voxel means naturally adapt to local geometry

    Parameters:
        voxel_size: float — voxel resolution (meters). Same as OnlineDynamicFilter.
        residual_threshold: float — points with ‖x - μ_voxel‖ > threshold
                              are classified as dynamic (meters).
        init_frames: int — number of frames for background initialization.
        ema_alpha: float (0-1) — exponential moving average rate for mean updates.
                   Higher = faster adaptation.
        verbose: bool
    """

    def __init__(self, voxel_size=1.0, residual_threshold=1.5, init_frames=10,
                 ema_alpha=0.3, x_range=(0,500), y_range=(-150,150),
                 z_range=(-5,50), verbose=False):
        self.voxel_size = voxel_size
        self.residual_threshold = residual_threshold
        self.init_frames = init_frames
        self.ema_alpha = ema_alpha
        self.verbose = verbose

        # ROI bounds (same as OnlineDynamicFilter)
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.z_min, self.z_max = z_range

        # Voxel grid dimensions
        self.nx = int(np.ceil((self.x_max - self.x_min) / voxel_size))
        self.ny = int(np.ceil((self.y_max - self.y_min) / voxel_size))
        self.nz = int(np.ceil((self.z_max - self.z_min) / voxel_size))
        self.total_voxels = self.nx * self.ny * self.nz

        if self.verbose:
            print(f"  Grassmann voxel grid: {self.nx}x{self.ny}x{self.nz} = {self.total_voxels} voxels")

        # Per-voxel state: mean position and count
        # Use dict for sparse storage (only occupied voxels)
        self.voxel_mean = {}     # lin_idx -> np.array(3,) running mean
        self.voxel_count = {}    # lin_idx -> int, number of static points seen
        self.frame_count = 0

    def _xyz_to_linidx(self, points):
        """Convert (N,3) coordinates to linear voxel indices."""
        ix = np.floor((points[:,0] - self.x_min) / self.voxel_size).astype(int)
        iy = np.floor((points[:,1] - self.y_min) / self.voxel_size).astype(int)
        iz = np.floor((points[:,2] - self.z_min) / self.voxel_size).astype(int)
        valid = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny) & (iz >= 0) & (iz < self.nz)
        lin_idx = -np.ones(len(points), dtype=int)
        lin_idx[valid] = ix[valid] * self.ny * self.nz + iy[valid] * self.nz + iz[valid]
        return lin_idx, valid

    def _compute_per_point_residuals(self, points, lin_idx, valid_mask):
        """
        Compute residual ‖x - μ_voxel‖ for each point.
        Points in voxels without a model get residual = 0 (treated as static/unknown).
        """
        N = len(points)
        residuals = np.zeros(N)

        for i in range(N):
            if not valid_mask[i]:
                continue
            idx = lin_idx[i]
            if idx in self.voxel_mean:
                residuals[i] = np.linalg.norm(points[i] - self.voxel_mean[idx])
            else:
                # Voxel has no model → unknown → treat as static (residual = 0)
                residuals[i] = 0.0

        return residuals

    def process_frame(self, points, intensities=None, velocities=None):
        """
        Process one frame. Same interface as OnlineDynamicFilter.process_frame().
        """
        if len(points) == 0:
            return np.array([]), None, None

        self.frame_count += 1
        lin_idx, valid_mask = self._xyz_to_linidx(points)

        if self.frame_count <= self.init_frames:
            # Build initial per-voxel models from all points
            for i in range(len(points)):
                if not valid_mask[i]:
                    continue
                idx = lin_idx[i]
                if idx not in self.voxel_mean:
                    self.voxel_mean[idx] = points[i].copy()
                    self.voxel_count[idx] = 1
                else:
                    # Running average
                    n = self.voxel_count[idx]
                    self.voxel_mean[idx] = (n * self.voxel_mean[idx] + points[i]) / (n + 1)
                    self.voxel_count[idx] = n + 1

            if self.verbose and self.frame_count == self.init_frames:
                print(f"  Grassmann: {len(self.voxel_mean)} occupied voxels initialized")
            # During init, return empty
            return np.array([]), None, None

        # Compute per-point residuals
        residuals = self._compute_per_point_residuals(points, lin_idx, valid_mask)

        # Dynamic = high residual (far from voxel mean)
        dynamic_mask = (residuals > self.residual_threshold) & valid_mask
        static_mask = (~dynamic_mask) & valid_mask

        # Apply neighborhood filtering: a dynamic point surrounded by static
        # points in neighboring voxels is likely noise, not a target
        dynamic_mask = self._apply_spatial_consistency(dynamic_mask, lin_idx, valid_mask, points)

        dyn_pts = points[dynamic_mask]
        dyn_int = intensities[dynamic_mask] if intensities is not None else None
        dyn_vel = velocities[dynamic_mask] if velocities is not None else None

        # Update per-voxel models with static points (EMA)
        static_indices = np.where(static_mask)[0]
        for i in static_indices:
            idx = lin_idx[i]
            if idx not in self.voxel_mean:
                self.voxel_mean[idx] = points[i].copy()
                self.voxel_count[idx] = 1
            else:
                # EMA update
                self.voxel_mean[idx] = ((1 - self.ema_alpha) * self.voxel_mean[idx] +
                                        self.ema_alpha * points[i])
                self.voxel_count[idx] += 1

        if self.verbose and self.frame_count > self.init_frames:
            print(f"  Frame {self.frame_count}: {len(dyn_pts)} dynamic / {len(points)} total "
                  f"(residual > {self.residual_threshold}m)")

        return dyn_pts, dyn_int, dyn_vel

    def _apply_spatial_consistency(self, dynamic_mask, lin_idx, valid_mask, points):
        """
        Require spatial consistency: a dynamic point should have at least
        min_neighbors other dynamic points in adjacent voxels.
        This filters isolated noise while preserving clustered drone points.
        """
        if np.sum(dynamic_mask) == 0:
            return dynamic_mask

        nynz = self.ny * self.nz
        # 27-neighbor offsets (including self)
        offsets = np.array([dx*nynz + dy*self.nz + dz
                           for dx in [-1,0,1] for dy in [-1,0,1] for dz in [-1,0,1]],
                           dtype=np.int64)

        dynamic_indices = np.where(dynamic_mask)[0]
        dyn_lin = lin_idx[dynamic_indices]

        refined = np.copy(dynamic_mask)
        for di, dl in zip(dynamic_indices, dyn_lin):
            if dl < 0:
                refined[di] = False
                continue
            neighbors = dl + offsets
            neighbors = neighbors[(neighbors >= 0) & (neighbors < self.total_voxels)]
            # Count dynamic neighbors
            dyn_set = set(dyn_lin[dyn_lin >= 0])
            n_dyn_neighbors = sum(1 for nb in neighbors if nb in dyn_set)
            if n_dyn_neighbors < 2:  # need at least 2 dynamic neighbors (drone has few points)
                refined[di] = False

        return refined

    def get_background_model(self):
        """Return per-voxel mean positions (for debugging)."""
        return dict(self.voxel_mean)


# ============================================================================
# MODULE 2: FMT Trajectory Smoother (Post-Tracking Manifold Optimization)
# ============================================================================

class FMTTrajectorySmoother:
    """
    Post-tracking Riemannian manifold trajectory smoother.

    DESIGN PRINCIPLE: This is a PURE POST-PROCESSOR.
      - It takes a COMPLETED track history as input.
      - It outputs a smoothed version of the trajectory.
      - It does NOT modify the tracker's internal state.
      - No feedback loop: smoothed positions are NOT fed back into tracking.

    When to call:
      - When a track is finalized (marked for removal)
      - At the end of the sequence (for all remaining tracks)
      - The smoothed trajectory is stored in tracker.smoothed_history
        and used ONLY for visualization and final reporting.

    Energy functional (optimized per-track):
      E(X) = E_data + α·E_kinematic + β·E_curvature + γ·E_prior

      where:
        E_data       = Σ_k ‖p_k - p_k^obs‖²          (data fidelity)
        E_kinematic  = Σ_k ‖(p_{k+1} - p_k)/dt - v_k‖² (kinematic consistency)
        E_curvature  = Σ_k ‖a_{k+1} - a_k‖²           (jerk minimization)
        E_prior      = Σ_k ‖w_k‖²                      (tangential velocity prior)

      with the FMCW manifold constraint:
        v_k = v_r_k · p̂_k + B(p̂_k) · w_k

    Drone-specific soft penalties (included in E_curvature + E_prior):
      - Acceleration bound: penalty when |a| > 8 m/s²
      - Curvature bound: penalty when κ > 0.6 /m
      - Speed bound: penalty when v < 1 or v > 25 m/s

    Parameters:
        alpha_kinematic: weight for kinematic consistency term (default 2.0)
        beta_curvature: weight for jerk/curvature smoothness (default 0.5)
        gamma_prior: weight for tangential velocity regularization (default 0.1)
        data_weight: weight for data fidelity (default 0.5)
        drone_max_accel: soft penalty threshold for acceleration (m/s²)
        drone_max_curvature: soft penalty threshold for curvature (1/m)
        drone_min_speed, drone_max_speed: speed bounds (m/s)
        max_iter: max L-BFGS-B iterations (default 100)
    """

    def __init__(self, alpha_kinematic=2.0, beta_curvature=0.5, gamma_prior=0.1,
                 data_weight=0.5, drone_max_accel=8.0, drone_max_curvature=0.6,
                 drone_min_speed=1.0, drone_max_speed=25.0, max_iter=100,
                 verbose=False):
        self.alpha_kinematic = alpha_kinematic
        self.beta_curvature = beta_curvature
        self.gamma_prior = gamma_prior
        self.data_weight = data_weight
        self.drone_max_accel = drone_max_accel
        self.drone_max_curvature = drone_max_curvature
        self.drone_min_speed = drone_min_speed
        self.drone_max_speed = drone_max_speed
        self.max_iter = max_iter
        self.verbose = verbose

    @staticmethod
    def _tangent_basis(p_hat):
        """Orthonormal basis for the tangent plane at p̂. Returns (3, 2)."""
        ez = np.array([0., 0., 1.])
        if abs(np.dot(p_hat, ez)) > 0.99:
            b1 = np.cross(p_hat, np.array([1., 0., 0.]))
        else:
            b1 = np.cross(p_hat, ez)
        b1 /= np.linalg.norm(b1) + 1e-12
        b2 = np.cross(p_hat, b1)
        b2 /= np.linalg.norm(b2) + 1e-12
        return np.column_stack([b1, b2])

    def smooth(self, positions_3d, radial_velocities):
        """
        Smooth a completed trajectory using manifold-constrained optimization.

        Args:
            positions_3d: (N, 3) array of observed 3D positions
            radial_velocities: (N,) array of measured radial velocities (FMCW)

        Returns:
            smoothed_positions: (N, 3) array of smoothed 3D positions
            metrics: dict with optimization statistics
        """
        N = len(positions_3d)
        if N < 4:
            # Too short to benefit from smoothing
            return np.array(positions_3d), {'status': 'too_short', 'N': N}

        pos_obs = np.asarray(positions_3d, dtype=np.float64)
        vr_obs = np.asarray(radial_velocities, dtype=np.float64)

        # Precompute tangent bases at observed positions
        B_cache = []
        p_hat_cache = []
        for k in range(N):
            p_hat = pos_obs[k] / (np.linalg.norm(pos_obs[k]) + 1e-6)
            p_hat_cache.append(p_hat)
            B_cache.append(self._tangent_basis(p_hat))

        # ── Initial guess ──
        # x = [p_0(3), ..., p_{N-1}(3), w_0(2), ..., w_{N-1}(2)]
        # Total: 3N + 2N = 5N variables
        n_vars = 5 * N
        x0 = np.zeros(n_vars)
        # Initialize positions from observations
        for k in range(N):
            x0[3*k:3*k+3] = pos_obs[k]
            x0[3*N + 2*k:3*N + 2*k+2] = np.zeros(2)  # w_init = 0

        # ── Energy function ──
        def energy_and_grad(x):
            E = 0.0
            grad = np.zeros_like(x)

            # Unpack
            p = x[:3*N].reshape(N, 3)
            w = x[3*N:].reshape(N, 2)
            grad_p = np.zeros((N, 3))
            grad_w = np.zeros((N, 2))

            for k in range(N):
                # ── E_data: weak anchor to observations ──
                diff = p[k] - pos_obs[k]
                E += self.data_weight * np.dot(diff, diff)
                grad_p[k] += 2.0 * self.data_weight * diff

                # ── E_prior: tangential velocity regularization ──
                E += self.gamma_prior * np.dot(w[k], w[k])
                grad_w[k] += 2.0 * self.gamma_prior * w[k]

                # ── E_kinematic: velocity consistency ──
                if k < N - 1:
                    # Reconstruct model velocity at frame k
                    p_hat_k = p[k] / (np.linalg.norm(p[k]) + 1e-6)
                    # Use cached B (approximation — ignore p_hat dependence on p)
                    B_k = B_cache[k]
                    v_k = vr_obs[k] * p_hat_k + B_k @ w[k]

                    dp = p[k+1] - p[k]
                    resid = dp - v_k    # (3,)
                    E += self.alpha_kinematic * np.dot(resid, resid)

                    # Gradients (simplified: ignore ∂p_hat/∂p)
                    grad_p[k]   += 2.0 * self.alpha_kinematic * (-resid)
                    grad_p[k+1] += 2.0 * self.alpha_kinematic * resid
                    grad_w[k]   += 2.0 * self.alpha_kinematic * (-B_k.T @ resid)

            for k in range(1, N - 1):
                # ── E_curvature: jerk minimization ──
                # a_k = v_k - v_{k-1}
                p_hat_k = p[k] / (np.linalg.norm(p[k]) + 1e-6)
                B_k = B_cache[k]
                v_k = vr_obs[k] * p_hat_k + B_k @ w[k]

                p_hat_km1 = p[k-1] / (np.linalg.norm(p[k-1]) + 1e-6)
                B_km1 = B_cache[k-1]
                v_km1 = vr_obs[k-1] * p_hat_km1 + B_km1 @ w[k-1]

                a_k = v_k - v_km1

                if k < N - 1:
                    p_hat_kp1 = p[k+1] / (np.linalg.norm(p[k+1]) + 1e-6)
                    B_kp1 = B_cache[k+1]
                    v_kp1 = vr_obs[k+1] * p_hat_kp1 + B_kp1 @ w[k+1]
                    a_kp1 = v_kp1 - v_k
                    jerk = a_kp1 - a_k

                    E += self.beta_curvature * np.dot(jerk, jerk)

                # ── Drone-specific soft penalties ──
                accel_mag = np.linalg.norm(a_k)
                if accel_mag > self.drone_max_accel:
                    excess = accel_mag - self.drone_max_accel
                    E += 5.0 * excess**2   # strong penalty for exceeding accel bound

                speed = np.linalg.norm(v_k)
                if speed < self.drone_min_speed and speed > 0.1:
                    E += 1.0 * (self.drone_min_speed - speed)**2
                if speed > self.drone_max_speed:
                    E += 5.0 * (speed - self.drone_max_speed)**2

                # Curvature penalty
                if k >= 2:
                    p0, p1, p2 = p[k-2], p[k-1], p[k]
                    v1, v2 = p1 - p0, p2 - p1
                    cross_norm = np.linalg.norm(np.cross(v1, v2))
                    curv = cross_norm / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-12) / \
                           (np.linalg.norm(v1) + 1e-12)
                    if curv > self.drone_max_curvature:
                        E += 10.0 * (curv - self.drone_max_curvature)**2

            # Flatten gradients
            grad[:3*N] = grad_p.ravel()
            grad[3*N:] = grad_w.ravel()

            return E, grad

        # ── Optimization ──
        try:
            E0, _ = energy_and_grad(x0)
            result = minimize(energy_and_grad, x0, method='L-BFGS-B',
                            jac=True, options={'maxiter': self.max_iter, 'ftol': 1e-6})
            x_opt = result.x if (result.success or result.fun < E0) else x0
            E_final = result.fun
            success = result.success
            n_iter = result.nit
        except Exception as e:
            if self.verbose:
                print(f"  FMT optimization failed: {e}, using raw trajectory")
            x_opt = x0
            E_final = 0.0
            success = False
            n_iter = 0

        # Extract smoothed positions
        smoothed = x_opt[:3*N].reshape(N, 3)

        # Compute quality metrics
        metrics = {
            'status': 'optimized' if success else 'fallback',
            'N': N,
            'energy_initial': float(E0),
            'energy_final': float(E_final),
            'iterations': int(n_iter),
            'mean_position_shift': float(np.mean(np.linalg.norm(smoothed - pos_obs, axis=1))),
            'max_position_shift': float(np.max(np.linalg.norm(smoothed - pos_obs, axis=1))),
        }

        if self.verbose:
            print(f"  FMT Smoother: N={N}, E: {E0:.1f}→{E_final:.1f}, "
                  f"iter={n_iter}, mean_shift={metrics['mean_position_shift']:.2f}m")

        return smoothed, metrics

    def process_trackers(self, trackers, only_finalized=True, quality_threshold=0.5):
        """
        Apply smoothing to a list of trackers and compute drone quality scores.

        Args:
            trackers: list of SingleTargetTracker objects
            only_finalized: if True, only smooth tracks marked for removal
                           or inactive. If False, smooth all.
            quality_threshold: tracks with quality score below this are
                              marked as likely non-drone in fmt_is_drone.

        Returns:
            n_smoothed: number of tracks that were smoothed
            quality_report: list of (tracker_id, quality_score, is_drone) tuples
        """
        n_smoothed = 0
        quality_report = []

        for trk in trackers:
            should_smooth = (not only_finalized) or \
                           (not trk.is_active) or \
                           (trk.marked_for_removal)

            if not should_smooth:
                continue

            # Gather position history (3D)
            if hasattr(trk, 'position_3d_history') and len(trk.position_3d_history) >= 4:
                pos_3d = np.array(trk.position_3d_history)
                vr = np.array(trk.velocity_history) if len(trk.velocity_history) >= len(pos_3d) \
                     else np.array(trk.velocity_history + [trk.velocity_history[-1]] * (len(pos_3d) - len(trk.velocity_history)))
                vr = vr[:len(pos_3d)]
            else:
                # Fall back to 2D history
                hist = np.array(trk.track_history)
                pos_3d = np.column_stack([hist[:,0], hist[:,1],
                                          np.full(len(hist), trk.center_3d[2])])
                vr = np.array(trk.velocity_history)
                if len(vr) < len(pos_3d):
                    vr = np.pad(vr, (0, len(pos_3d) - len(vr)), 'edge')

            if len(pos_3d) < 4:
                continue

            smoothed, metrics = self.smooth(pos_3d, vr)

            # Store smoothed positions WITHOUT modifying tracker's internal state
            trk.smoothed_history_3d = smoothed
            trk.smoothed_history_2d = [(s[0], s[1]) for s in smoothed]
            trk.fmt_metrics = metrics
            n_smoothed += 1

            # Compute drone quality score using smoothed trajectory
            quality = compute_drone_quality_score(trk, self)
            trk.fmt_quality_score = quality
            trk.fmt_is_drone = quality >= quality_threshold
            quality_report.append((trk.target_id, quality, trk.fmt_is_drone))

        return n_smoothed, quality_report

    def filter_by_drone_quality(self, trackers, quality_threshold=0.5):
        """
        Mark low-quality tracks as non-drone for final reporting.
        Does NOT modify tracker.is_active — only sets fmt_is_drone flag.

        Returns list of trackers that pass the quality threshold.
        """
        drone_tracks = []
        for trk in trackers:
            if hasattr(trk, 'fmt_quality_score'):
                trk.fmt_is_drone = trk.fmt_quality_score >= quality_threshold
            else:
                # No FMT data — compute quick quality from raw track
                quality = compute_drone_quality_score(trk, None)
                trk.fmt_quality_score = quality
                trk.fmt_is_drone = quality >= quality_threshold
            if trk.fmt_is_drone:
                drone_tracks.append(trk)
        return drone_tracks


# ============================================================================
# Utility: Combined pipeline helper
# ============================================================================

def compute_drone_quality_score(tracker, smoother=None):
    """
    Compute a 0-1 "drone-likeness" score for a tracked target.

    High score = more likely to be a real drone.
    Uses smoothed trajectory if available, otherwise raw track history.

    Components:
      - Trajectory smoothness (low curvature variance)
      - Speed consistency (within drone bounds)
      - Track persistence (length)
      - Kinematic residual (low = physically consistent)
    """
    score = 0.5  # neutral prior

    # Use smoothed positions if available
    if hasattr(tracker, 'smoothed_history_3d') and len(tracker.smoothed_history_3d) >= 3:
        pos = np.array(tracker.smoothed_history_3d)
    elif hasattr(tracker, 'position_3d_history') and len(tracker.position_3d_history) >= 3:
        pos = np.array(tracker.position_3d_history)
    else:
        hist = np.array(tracker.track_history)
        if len(hist) < 3:
            return 0.1
        pos = np.column_stack([hist[:,0], hist[:,1], np.zeros(len(hist))])

    # 1. Track persistence (longer = better)
    persistence = min(len(pos) / 30.0, 1.0)  # 30 frames = full score
    score += 0.2 * persistence

    # 2. Speed consistency
    speeds = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    if len(speeds) >= 2:
        speed_cv = np.std(speeds) / (np.mean(speeds) + 1e-6)
        speed_score = max(0, 1.0 - speed_cv)
        mean_speed = np.mean(speeds)
        if 1.0 <= mean_speed <= 25.0:
            speed_score += 0.2
        score += 0.2 * speed_score

    # 3. Curvature consistency (low variance = smooth)
    if len(pos) >= 3:
        curvatures = []
        for k in range(2, len(pos)):
            v1 = pos[k-1] - pos[k-2]
            v2 = pos[k] - pos[k-1]
            cross = np.linalg.norm(np.cross(v1, v2))
            curv = cross / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-12) / \
                   (np.linalg.norm(v1) + 1e-12)
            curvatures.append(curv)
        if curvatures:
            curv_cv = np.std(curvatures) / (np.mean(curvatures) + 1e-6)
            curv_score = max(0, 1.0 - curv_cv)
            score += 0.3 * curv_score

    # 4. FMT kinematic residual (if available)
    if hasattr(tracker, 'fmt_metrics'):
        res = tracker.fmt_metrics.get('mean_position_shift', 0)
        res_score = max(0, 1.0 - res / 5.0)  # < 5m shift = good
        score += 0.1 * res_score

    return min(max(score, 0.0), 1.0)
