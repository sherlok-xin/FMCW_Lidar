# Project Context

Last repository audit: **2026-09-20**  
Evidence scope: files present under `/home/xin/FMCW_LIDAR` at audit time.  
Primary recovered audits: [`FMCW_LiDAR_Project_Archaeology.md`](../FMCW_LiDAR_Project_Archaeology.md) and [`FMCW_LiDAR_Data_Archaeology.md`](../FMCW_LiDAR_Data_Archaeology.md).

## Evidence vocabulary

- **IMPLEMENTED** means source code exists.
- **TESTED** means a matching result artifact or a recorded reproduction exists.
- **VERIFIED RESULT** means a value is directly supported by an artifact.
- **PROPOSED** and **SPECULATIVE** are not results.
- **UNKNOWN** means the repository does not establish the fact.

This distinction is essential because the repository contains substantially more algorithm code than validated experimental evidence.

## Research objective

The project investigates detection and tracking of small unmanned aerial vehicles (UAVs) using FMCW LiDAR point clouds. The available code attempts to:

1. retain LiDAR position, intensity, and Doppler/radial-velocity information;
2. remove static background and spatially isolated clutter;
3. cluster remaining dynamic points into object candidates;
4. associate candidates over time into tracks; and
5. explore trajectory filtering or multi-target filters for more robust tracking.

The intended operational performance target, deployment environment, acceptable false-alarm rate, and required range are **UNKNOWN**. Folder names refer to nominal 100 m, 200 m, and 300 m flight scenarios, but those names are not calibrated per-frame ground truth.

## Physical sensing setup

What the repository establishes:

- The sensor produces FMCW LiDAR point clouds with spatial coordinates, intensity, and a per-point `velocity` field.
- ROS bag recordings include `/aqronos_cloud`; `get_lidar.py` republishes a five-field cloud on `/aqronos_cloud_81`.
- Some bags contain one rich point-cloud topic; some dual-radar bags contain two rich point-cloud topics.
- Recorded experiments include nominal cross-range flights, longitudinal/approaching motion, and round-trip flights at nominal speeds such as 5, 10, and 13 m/s.
- A spreadsheet, `bag/20260205/飞行轨迹坐标.xlsx`, contains GNSS-like coordinate strings for named flight sequences and device coordinates.

The following are **UNKNOWN** from repository evidence:

- LiDAR manufacturer/model, optical scan pattern, wavelength, angular resolution, range accuracy, Doppler ambiguity interval, and velocity-unit specification;
- exact sensor mounting pose and the authoritative sensor coordinate convention;
- LiDAR-to-GNSS time synchronization and spatial extrinsics;
- dual-sensor relative pose, topic-to-physical-device mapping, and clock synchronization;
- UAV models, dimensions, reflectivity, attitude, weather, visibility, and complete acquisition settings;
- whether all directory distance/speed labels reflect executed trajectories rather than planned trajectories.

The preprocessing code documents an input frame interpreted as x-forward, y-down, z-left and converts points to x-forward, y-left, z-up using:

```text
[x_out, y_out, z_out] = [x_in, z_in, -y_in]
```

This is an **implemented assumption**, not a verified calibration.

## FMCW-LiDAR data characteristics

### Point fields

Rich ROS point clouds observed by the repository audit contain 16 named fields:

```text
x, y, z, rgba, velocity, radius, theta, phi, intensity, ring,
velocity0, time, velocity_direction, acceleration,
acceleration_direction, surface_evenness
```

The five-field republished/cloud-export representation is:

```text
x, y, z, intensity, velocity
```

Existing PCD files use these headers:

- raw PCD: `x y z intensity velocity`
- filtered PCD: `x y z velocity intensity`

Software must access columns by field name. Assuming positional equivalence across raw and filtered files can interchange intensity and velocity.

### Scale and sparsity

Selected raw point clouds contain approximately 1,250,000 points per frame. After intensity and height filtering, only hundreds to thousands remain, and the final background/neighborhood stages often retain tens or fewer. The UAV is therefore represented by a sparse and intermittent subset of a very dense scan.

In manually guided pseudo-label regions:

- cross-range sequence: mean 8.32 candidate UAV points per frame after intensity and height filtering, with one zero-return frame;
- 300 m sequence: mean 3.96 candidate UAV points per frame, with 15 of 50 zero-return frames.

These are **VERIFIED RESULTS for selected pseudo-label regions**, not unbiased target recall.

### Velocity convention

For two reconstructed target trajectories, `-measured velocity` correlated with geometric range rate at 0.973 (cross-range) and 0.966 (300 m). This is evidence that the stored velocity sign is opposite increasing geometric range for those data and scripts. The formal sensor convention and its generality are **UNKNOWN**.

No simple universal range-to-radial-speed relationship was found in those two sequences. Cross-range motion frequently has near-zero radial velocity even when the target is moving.

### Timing

- The 100 m rich bag contained 46 frames at roughly 1.02 Hz according to the audit.
- PCD filenames encode timestamps, but several scripts truncate them to integer seconds.
- Many trackers assume `dt=1`.
- Some bag record times are irregular, and absolute PCD header dates can disagree with bag dates.

An authoritative clock, timestamp-conversion procedure, and inter-sensor synchronization are **UNKNOWN**.

## Task definition

The implemented task is currently:

> Given a temporal sequence of FMCW-LiDAR point clouds, identify dynamic point clusters and maintain object tracks using position and radial-velocity consistency.

The scientific target appears to be UAV detection and tracking, but the repository lacks synchronized point-level labels, object boxes/centers, persistent identity ground truth, negative-scene labels, and a frozen train/validation/test split. Therefore:

- “dynamic point” does not imply “UAV point”;
- “cluster” does not imply “UAV detection”;
- “active track” does not imply “correct target track”;
- pseudo-label trajectories are useful for diagnostics but are not independent ground truth.

## Current processing pipeline

The most complete offline rule-based path is implemented in `target_track_without_initialize_and_grid.py`. The reconstructed pipeline is:

```text
ROS bag / PCD sequence
        |
        v
five-field extraction or PCD loading
        |
        v
coordinate transform [x, z, -y]
        |
        v
intensity filter (commonly 10..250; some ROS variants use 20..250)
        |
        v
height filter (commonly z > 10 m in offline evaluation)
        |
        v
voxel occupancy background model
  - 1 m voxels
  - ROI x 0..500, y -150..150, z -5..50
  - first 8 frames initialize background
  - static probability threshold 0.05
        |
        v
27-neighbor static-density suppression
        |
        v
DBSCAN clustering (commonly eps=2 m, min_points=1)
        |
        v
candidate initialization / association
  - Euclidean position gate
  - radial-velocity consistency gate
  - lost/static/stability removal rules
        |
        v
track states, plots, and internal summary metrics
```

The exact preprocessing route from each ROS bag to the checked-in PCD directories is **UNKNOWN**: no complete bag-to-PCD export command, manifest, or provenance record was found.

## Algorithm and model architecture

### Rule-based baseline

The baseline is detect-then-track, with no learned detector. Its main components are thresholding, voxel occupancy, neighbor-count filtering, DBSCAN, and hand-written track management.

Common offline benchmark parameters are:

| Parameter | Value |
|---|---:|
| voxel size | 1 m |
| background initialization | first 8 frames |
| static probability threshold | 0.05 |
| height threshold | 10 m |
| search radius | 15 m |
| radial-velocity threshold | 5 m/s |
| maximum association distance | 30 m |
| DBSCAN `eps` | 2 m |
| DBSCAN `min_points` | 1 |

These are code/configuration facts, not optimized values.

### Manual and automatic initialization variants

- `target_track.py`: earlier tracker with manual region-of-interest initialization.
- `target_track_without_initialize.py`: removes manual initialization.
- `target_track_without_initialize_and_grid.py`: integrates voxel background filtering, pending candidates, stability scoring, and removal rules.
- `target_track_without_initialize_and_grid_ros.py`: Python ROS online variant.
- `src/target_track_without_initialize_and_grid_ros.cpp`: C++ ROS variant.

Defaults are inconsistent across variants. For example, the Python ROS path commonly uses search/radial thresholds 10 m/3 m/s, while the C++ path uses 10 m/0.5 m/s. Which defaults are authoritative is **UNKNOWN**.

### GM-PHD variants

- `target_track_gmphd_ros.py`: linear GM-PHD-style multi-target tracker.
- `target_track_ekf_gmphd_ros.py`: EKF GM-PHD-style tracker including radial-velocity observation handling.

Both are **IMPLEMENTED**. No matching result directory, log, or quantitative comparison was found, so successful operation and performance are **UNKNOWN**.

### “Grassmann” and FMT experiments

`src/manifold_modules.py` contains:

- `GrassmannDynamicFilter`: despite the name, the inspected implementation maintains a per-voxel running mean and uses Euclidean residuals plus neighborhood consistency. A learned Grassmann subspace, principal-angle metric, or geodesic update was not found.
- `FMTTrajectorySmoother`: a trajectory postprocessor.

`target_track_fmt.py` integrates these experimental components. `scripts/benchmark_fmt.py` separately compares a reconstructed baseline with an online FMT tracker using radial/tangential velocity decomposition.

The benchmark contains important confounds:

- the benchmark baseline is not behaviorally identical to the main offline tracker;
- it initializes on the first dynamic frame but does not add later births/pending candidates;
- empty candidate frames skip track aging;
- its intensity array is sliced rather than masked consistently, although intensity is not consumed downstream in that benchmark;
- the baseline and FMT paths use inconsistent radial-velocity sign treatment;
- “fragmentations” is implemented as the count of removed tracks, not a ground-truth fragmentation metric.

## Directory structure

```text
FMCW_LIDAR/
├── AGENTS.md                         persistent agent policy
├── ReadMe.md                         short original project overview
├── FMCW_LiDAR_Project_Archaeology.md technical repository audit
├── FMCW_LiDAR_Data_Archaeology.md    data and experimental audit
├── docs/                             durable project handoff documents
├── bag/                              ROS bags, archives, scene image, GNSS sheet
├── data/                             raw and filtered PCD sequences
├── results/                          benchmark and archaeology artifacts
├── scripts/                          benchmarks and reverse-analysis utilities
├── src/                              C++ ROS code and manifold/FMT modules
├── include/                          C++ headers
├── fmcw/                             local Python virtual environment
├── build/                            empty at audit time
├── CMakeLists.txt                    ROS1/catkin build definition
├── package.xml                       ROS1 package metadata
└── *.py                              preprocessing, filtering, and tracking entry points
```

Other repository-local metadata:

- `.vscode/` contains stale references to a `dynablox` workspace not present here.
- `.claude/` contains stale permissions referencing missing files and unrelated absolute paths.
- `.local/` contains desktop trash/GVFS metadata and unrelated discarded ML files; it is not considered project evidence.
- `.qoder/` is effectively empty.

## Important scripts and roles

| File | Role | Evidence status |
|---|---|---|
| `get_lidar.py` | Repackages `/aqronos_cloud` to five fields on `/aqronos_cloud_81` | IMPLEMENTED; no PCD export |
| `multi_filter_with_intensity.py` | Coordinate transform, intensity filtering, compressed PCD writing | IMPLEMENTED |
| `single_filter.py` | Single-cloud filtering utility | IMPLEMENTED |
| `static_and_dynamic.py` | Initial voxel static/dynamic separation | IMPLEMENTED |
| `static_and_dynamic_density.py` | Adds neighborhood-density filtering | IMPLEMENTED |
| `target_track.py` | Manual-initialization tracker | IMPLEMENTED |
| `target_track_without_initialize.py` | Automatic-initialization tracker | IMPLEMENTED |
| `target_track_without_initialize_and_grid.py` | Most complete offline rule baseline | IMPLEMENTED; benchmark-related artifacts exist |
| `target_track_without_initialize_and_grid_ros.py` | Python ROS online tracker | IMPLEMENTED; execution UNKNOWN |
| `src/target_track_without_initialize_and_grid_ros.cpp` | C++ ROS tracker | IMPLEMENTED; build/run UNKNOWN |
| `target_track_gmphd_ros.py` | GM-PHD ROS experiment | IMPLEMENTED; test evidence UNKNOWN |
| `target_track_ekf_gmphd_ros.py` | EKF GM-PHD ROS experiment | IMPLEMENTED; test evidence UNKNOWN |
| `src/manifold_modules.py` | Running-mean dynamic filter and FMT smoother | IMPLEMENTED; naming overstates Grassmann content |
| `target_track_fmt.py` | Experimental FMT/manifold integration | IMPLEMENTED; matching results UNKNOWN |
| `scripts/benchmark_fmt.py` | Baseline-versus-FMT offline benchmark | TESTED on five sequences |
| `scripts/data_archaeology.py` | Stage-count and point-cloud diagnostic analysis | TESTED |
| `scripts/uav_pseudo_label.py` | Manual/trajectory-guided pseudo-label construction | TESTED on 300 m and cross sequences |
| `scripts/compile_uav_evidence.py` | Aggregates pseudo-label evidence | TESTED |
| `scripts/scan_100m_expected_band.py` | Searches GNSS-informed range/height band | TESTED on 100 m sequence |
| `scripts/reverse_engineer_baseline_uav.py` | Reconstructs cross-range baseline target track | TESTED |
| `scripts/reverse_engineer_longitudinal_baseline.py` | Reconstructs longitudinal baseline target track | TESTED |
| `visualize_pcd.py` | PCD visualization utility | IMPLEMENTED |

## Datasets and data formats

### PCD inventory

At audit time, `data/` contained 674 PCD files across these sequences:

| Directory | Frames | Representation | Interpreted scenario |
|---|---:|---|---|
| `100m_v10m_s_2026-02-05-15-08-28_filtered_data` | 46 | filtered PCD | nominal 100 m, 10 m/s |
| `2025-10-23-15-15-25_filtered_data` | 343 | filtered PCD | scenario metadata UNKNOWN |
| `300m_v10_ms_2026-03-19-14-11-42` | 57 | raw PCD | nominal 300 m, 10 m/s |
| `300m_v10_ms_2026-03-19-14-11-42_filtered_data` | 50 | filtered PCD | nominal 300 m, 10 m/s |
| `81-82-pm-x10_2025-11-19-15-38-07` | 30 | raw PCD | dual-radar/longitudinal interpretation |
| `81-82-pm-x10_2025-11-19-15-38-07_filtered_data` | 28 | filtered PCD | same sequence |
| `81-pm-cross-x10_2025-11-19-15-57-13` | 60 | raw PCD | cross-range interpretation |
| `81-pm-cross-x10_2025-11-19-15-57-13_filtered_data` | 60 | filtered PCD | same sequence |

The reason raw and filtered frame counts differ in two sequences is **UNKNOWN**.

### Bag inventory

`bag/` occupied approximately 115 GB and held 21 bag/archive files at audit time. Major groups include:

- February 2026 nominal 100/200/300 m cross-range recordings at multiple speeds;
- nominal 300 m round-trip recordings;
- an extracted 100 m bag (about 4.6 GB) and an extracted 300 m bag (about 1.4 GB);
- dual-radar recordings (about 13 GB and 26.4 GB);
- `DronesBag` ZIP archives (about 5.8–19.3 GB each).

Exact checksums and a canonical raw-to-derived manifest are not currently recorded.

### Labels and splits

- No authoritative object annotations were found.
- No train/validation/test split was found.
- Pseudo-label CSVs exist under `results/data_archaeology` for selected 300 m and cross-range trajectories.
- The GNSS spreadsheet contains sparse waypoints, but alignment to LiDAR frames is unresolved.

## Known assumptions

1. The coordinate transform in `multi_filter_with_intensity.py` is correct.
2. `velocity` is radial velocity and is usable after a sign inversion in at least some code paths.
3. The first eight frames are suitable background-only initialization frames.
4. A voxel occupied during initialization is evidence of static background.
5. Sparse moving points remain after 27-neighbor suppression.
6. DBSCAN with `min_points=1` is acceptable for UAV candidates.
7. Constant-velocity prediction with `dt=1` is adequate.
8. Position and radial-velocity gates can associate candidates without calibrated uncertainty.
9. Filename order corresponds to time order.
10. Data directory labels identify the physical trajectory.

All ten are implementation or analysis assumptions. None is fully validated by a synchronized calibration/ground-truth dataset.

## Current evaluation metrics

The benchmark records internal tracker diagnostics:

- final active-track count;
- total generated-track count;
- removed-track count, labeled “fragmentations” in places;
- trajectory smoothness, implemented as a variance-derived internal statistic;
- per-frame latency;
- FMT residual error.

These metrics can detect behavioral changes but do not establish UAV detection/tracking accuracy. The repository does **not** currently support verified computation of precision/recall/F1, false-alarm rate, localization or velocity RMSE, MOTA/MOTP/HOTA/IDF1, identity switches, true fragmentation, or track completeness.

## Environment and reproducibility state

At the 2026-09-20 audit:

- local environment: `fmcw/bin/python`, Python 3.10.12;
- key installed versions: NumPy 2.2.6, SciPy 1.15.3, scikit-learn 1.7.2, Open3D 0.19.0, OpenCV 4.13.0.92, Matplotlib 3.10.9, PyYAML 6.0.3;
- `requirements.txt` listed package names but did not pin versions;
- 22 project Python files parsed successfully;
- `CMakeLists.txt` defines a ROS1/catkin C++11 target, but `roscore` and `catkin_make` were unavailable;
- the repository was not under Git control;
- no notebooks and no scientific run logs were found.

Reproducibility is therefore partial: checked-in result artifacts can be audited, but exact source/data lineage for older runs is not guaranteed.
