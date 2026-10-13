# FMCW visual diagnostic / ROS2 sanity check

Run date: 2026-10-13. Workspace: `/home/xin/FMCW_LIDAR`. Scope: temporary visualization and ROS transport only. These artifacts are diagnostic evidence, not a new method, calibration, point GT, or benchmark.

**Observed outcome:** The unchanged `get_lidar` callback preserves raw geometry and target/clutter returns on all 9 inspected 100 m frames. Provisional GNSS centres do not consistently coincide with existing closure anchors. An empty small GNSS region therefore does not certify physical UAV non-return. The cause of the remaining spatial, temporal, or association disagreement is **UNKNOWN**.

## Environment and source identity

- Initial `$ROS_DISTRO` was unset. After `source /opt/ros/humble/setup.bash`, it is `humble`.
- `ros2 --help`, `ros2 bag --help`, and `ros2 bag info/record/play` worked. ROS2 Python imports for `rclpy`, `sensor_msgs`, and `rosbag2_py` also worked.
- `rviz2` runs on the existing X11 display `:1`, with Xauthority `/run/user/1000/gdm/Xauthority`. The initial hardware-rendered viewport was black; temporary `LIBGL_ALWAYS_SOFTWARE=1` produced a visible cloud. OpenGL 4.5 software rendering was used. No package installation or permanent graphics change was made.
- Available ROS2 bag verbs: `convert`, `info`, `list`, `play`, `record`, and `reindex`. Installed usable storage: `sqlite3`; test plugins are also enumerated. ROS1 `rosbag`, `rospy`, `roscore`, and catkin tooling, as well as `ros1_bridge` and `rosbags-convert`, were unavailable. The available `rosbag2 convert` command does not establish ROS1 input support. ROS2 does not require `roscore`.
- The ZIP members for `100m_cross_5` and `100m_cross_10` were read directly and have magic `#ROSBAG V2.0\n`: ROS1 bag version 2.0, not ROS2 rosbag2. Existing standalone copies were opened read-only for indexed playback. No raw conversion was attempted. A temporary ROS2 raw publisher decoded the ROS1 source bytes without creating a converted bag.
- Both 100 m source topics are exactly `/aqronos_cloud_0`, with type `sensor_msgs/PointCloud2` (ROS2: `sensor_msgs/msg/PointCloud2`). Actual source field/header checks agree with fixed frame `aqronos_lidar`; no frame discrepancy was found. Source footer counts are 32 and 46 messages, respectively.
- Rich layout: height 8; width 156250; 1,250,000 slots; `point_step=80`; `row_step=12,500,000`; data length 100,000,000 bytes; zero row padding; little endian; `is_dense=true`. Fields: `x`, `y`, `z`, `rgba`, `velocity`, `radius`, `theta`, `phi`, `intensity`, `ring`, `velocity0`, `time`, `velocity_direction`, `acceleration`, `acceleration_direction`, and `surface_evenness`. Field names and offsets are preserved. Intensity is at offset 20 despite its later position in the declared field order. ROS1 `Header.seq` has no ROS2 Header counterpart and remains in explicit diagnostic identity JSON.
- Played source frames retain their exact original point bytes, fields, dimensions, `frame_id`, and header stamp. SHA-256 identities are recorded in raw-player JSONL logs. A representative raw ROS2 CDR serialization round trip matched the original data bytes. These are transport checks, not new calibration results.

## Frozen geometry and synchronization

The required phenotype, closure, and v0 reports, project state documents, current `get_lidar` source, `package.xml`, and `CMakeLists.txt` were inspected. Current markers use the completed phenotype `run_02/frames.csv` and its existing `region_features.csv`, with provenance to `results/calibration_closure/alignment.json`, `alignment_results.csv`, and unchanged observed silver anchors. The superseded root `frame_ground_truth.csv` is not used for markers.

Table axes are `[raw_x, raw_z, -raw_y]`. The inverse mapping to native RViz XYZ is `[analysis_x, -analysis_z, analysis_y]`. Raw and repacked clouds retain sensor XYZ. Plotting table XYZ directly in their frame would create a false diagnostic mismatch. No extrinsic or GNSS value was refitted.

The saved shared provisional pose is yaw 173.4498°, pitch -6.5149°, roll 2.2361°, and translation `[1.2056, -6.2698, 2.0066]` m. The exact matrix, translation, and already interpolated per-frame coordinates are read from frozen artifacts, rather than from rounded report values. Saved offsets are:

- `100m_cross_5`: 96.305750645698 s.
- `100m_cross_10`: 96.57586932182312 s.
- `200m_cross_5`: 122.31215786933898 s.
- `200m_cross_10`: 96.3393325805664 s.
- `300m_cross_10`: 129.75867342948914 s.

No yaw, pitch, roll, translation, time-offset, or interpolation search was performed.

Raw marker updates require both explicit bag-record/frame identity and delivery of the matching cloud. Processed replay retains the historical header seconds/nanoseconds; that exact pair identifies the original frame-table row and bag-record time. The 2022 header epoch is **never** used as GNSS query time. New processed recording times are 2026-10-13 receipt times that include processing latency, not 2026-02-05 acquisition timestamps. The helper performs no new GNSS interpolation or extrapolation. Marker stamp 0 is a display convention, not a synchronization clock.

The red sphere shows provisional GNSS; the green sphere shows the existing observed anchor. Red great-circle wireframes have radii 3 m and, optionally, 10 m. The GNSS line and observed-anchor fragments are separate; gaps and absent anchors are not interpolated. Helper QA verified inverse coordinates, exact sphere radii, absent-centre handling, and raw identity/cloud pairing.

## Visual evidence

Full raw PointCloud2 messages from both 100 m conditions were inspected first in RViz, alongside source-verified top/side crop figures. Far-range views followed. All 17 selected frames were actually played over ROS2, in groups of 5 + 4 + 3 + 2 + 3. Their diagnostics—including condition, frame index, exact bag-record time, GNSS/anchor coordinates, discrepancy, and raw 1/3/10 m or 1/3 m counts—are in [SELECTED_FRAMES.md](SELECTED_FRAMES.md), [selected_frames.csv](selected_frames.csv), and [selected_frames.jsonl](selected_frames.jsonl). The marker helper also prints each viewed frame as JSON.

- Normal 100 m examples: `100m_cross_5` frames 14/28/31 have GNSS–anchor distances 1.288/0.856/0.634 m, GNSS 3 m counts 39/45/36, and anchor 3 m counts 39/45/36. `100m_cross_10` frames 21/22 have distances 2.082/1.840 m and 41/27 returns in both 3 m regions.
- Empty nominal 3 m regions with positive observed-anchor support occur in `100m_cross_5` frames 12/16 and `100m_cross_10` frames 18/19. Their respective anchor 3 m counts are 37/48/25/46. Compact return support is visible around the green anchor, while the red GNSS centre misses it laterally. Larger 10 m regions can also include lower surrounding clutter; their points are not all target labels.
- Maximum discrepancy: `200m_cross_5`, frame 21, bag record `1770276618.331014269`. Analysis GNSS is `[209.15285461756005, -11.312744510262032, -4.676603331711904]` m; the anchor is `[212.23568115234374, -37.69139760335286, -3.7201686541239423]` m; distance is **26.575400763519816 m**. GNSS 1/3/10 m counts are 0/0/0; anchor 1/3 m counts are 15/15. The anchor-minus-GNSS difference, `[3.082827, -26.378653, 0.956435]` m, is dominated by the lateral/flight-track coordinate. This row is at the saved final GNSS endpoint; the `200m_cross_5` offset is historically boundary-limited. The side projection hides the lateral separation and can misleadingly place the anchor inside the projected 10 m disk. Read exact 3D counts, top view, and side view together.
- At `200m_cross_5` frames 3/10, the same frozen model yields much smaller discrepancies of 0.782/0.465 m. A constant additive displacement alone does not describe this frame-to-frame pattern. This does not establish which extrinsic, timing, interpolation, or association mechanism causes it.
- `200m_cross_10` frames 36/40 and `300m_cross_10` frames 0/4/9 have compact selected anchor-neighborhood support of 21/10 and 5/5/11 raw returns, respectively, within 3 m. The 300 m support remains sparse. Nearby far-range clutter can project close in one view while separating in the other. Stored anchor-neighborhood points are not independent point-identity GT.

Static figures are explicitly labeled as saved raw region-union crops, not complete raw clouds. Every selected frame's 16 native columns were verified against saved raw point bytes, and counts were checked against unchanged region artifacts. Full cloud context was displayed through raw ROS2 playback; cropped plots provide stable orthogonal views and readable close-ups. Viewport limits and the 0–80 intensity colormap do not select new research thresholds. RViz overview snapshots may use interactive oblique camera positions; use the static top/side plots for precise projection comparison.

Figures: [100 m cross 5](figures/100m_cross_5__contact_sheet.png), [100 m cross 10](figures/100m_cross_10__contact_sheet.png), [200 m cross 5](figures/200m_cross_5__contact_sheet.png), [200 m cross 10](figures/200m_cross_10__contact_sheet.png), [300 m cross 10](figures/300m_cross_10__contact_sheet.png), and [maximum-discrepancy detail](figures/200m_cross_5__frame_0021__top_side.png). Byte/count figure manifests are [static_diagnostics_manifest_100m.json](figures/static_diagnostics_manifest_100m.json) and [static_diagnostics_manifest_far.json](figures/static_diagnostics_manifest_far.json).

## Actual get_lidar behaviour and recorded comparison

Source `/home/xin/FMCW_LIDAR/get_lidar.py`, SHA-256 `710f1e73f12293acff8e9976803693b00b94ec72e5e1a2bfb7832de74f979fcf`, is ROS1. Its input is `/aqronos_cloud`; its output is `/aqronos_cloud_81`. The package and CMake definitions use ROS1 catkin; no launch file exists for this node. The February source requires explicit remapping: `/aqronos_cloud:=/aqronos_cloud_0`.

The temporary ROS2 adapter executes the unchanged original callback through narrow transport/API shims. It does not rewrite the research implementation or claim that the original full `rospy` node ran natively under ROS2. The callback reads all fields with `skip_nans=true`, retains `x`, `y`, `z`, `intensity`, and `velocity` as FLOAT32, and copies the header. It rejects a row containing NaN in any field, while retaining infinities and finite zero-XYZ placeholders. Synthetic checks cover these semantics, endian handling, row padding, negative zero, headers, and exact bytes.

**There is no coordinate transform, intensity/height/background filtering, ROI rejection, or voxel processing in `get_lidar`.** The separate PCD/filter/tracker sources must not be conflated with this node. Output has height 1, width N, `point_step=20`, `row_step=20N`, little endian, and `is_dense=false`. Eleven rich fields are discarded, including point time; positions are preserved.

| Condition | Original source count | Recorded diagnostic frames | Recorded topic | Independent raw-vs-output byte checks | Failures |
|---|---:|---|---|---:|---:|
| `100m_cross_5` | 32 | 12, 14, 16, 28, 31 | `/aqronos_cloud_81` | 5/5 | 0 |
| `100m_cross_10` | 46 | 18, 19, 21, 22 | `/aqronos_cloud_81` | 4/4 | 0 |

Recorded bags: [processed/100m_cross_5_checked](processed/100m_cross_5_checked) and [processed/100m_cross_10_checked](processed/100m_cross_10_checked). Their `.db3` sizes are 125,157,376 and 100,130,816 bytes, respectively; metadata and exact sizes are directly auditable. These are selected diagnostic slices, not full 32/46-frame recordings. Source frame indices are the original zero-based identities.

All 9 selected messages have 1,250,000 slots and zero any-field NaN rejections. Independent live and recorded-file verifiers establish exact five-field bytes, original header/frame identity, unchanged nonzero-return counts and zero placeholders, and all available frozen neighborhood counts. The processing node and raw player were stopped before both recorded bags were independently replayed in RViz. All 5 and 4 original frame identities reappeared in the processed marker logs.

The explicit raw–processed comparison is:

- Selected candidate returns: all selected raw neighborhood returns survive.
- Nearby clutter: none disappears through `get_lidar`; clutter remains.
- Coordinates: unchanged; XYZ bytes match.
- Apparent candidate returns removed: none on the 9 tested frames. The general source policy would remove a row containing NaN in any rich field.
- Closure-anchor agreement: the same compact neighborhood and 1/3 m counts persist, without new association or GT certification.
- Cleaner view: **not achieved by this source**. Bag payload is 4× smaller per slot because fields are repacked from 80 to 20 bytes, not because clutter is removed.

## Retained failures and limits

The first temporary adapter transport used BEST_EFFORT. All 4 raw `100m_cross_10` messages reached the RELIABLE independent verifier, but none reached that adapter. The first `processed/100m_cross_10_diag` bag has 0 messages and remains preserved. This is a transport failure, not sensor non-return. A RELIABLE input successfully retried the same callback; no processing threshold changed.

The initial hardware-GL black viewport is retained in an initial screenshot; software-GL rendering worked. Processed playback with read-ahead queue size 1 stalled and was stopped. Queue size 10 replayed successfully; tested final commands use 10.

Final causal conclusions remain **UNKNOWN**. Systematic physical extrinsic error, along-track time error, sparse GNSS interpolation error within a chord, incorrect anchor association, and actual per-frame UAV absence cannot be separated definitively by these selected visual proxies. Positive anchor support prevents treating an empty nominal region alone as proof that nearby returns are absent. This task supplies no independent labels, surveyed calibration, clock event, negative-state certificate, or method score.

## Reproduce and preserve scope

[WORKFLOW.md](WORKFLOW.md) contains exact terminal-by-terminal commands for raw playback, markers, the unchanged-callback adapter, recording, and independent replay, together with view controls, the velocity-colour option, and far-frame commands. Start only one source/player and one marker helper at a time. All helpers, configs, and artifacts remain under `/tmp/fmcw_rviz_check/`. No E18, label, calibration, historical-report, score, threshold, raw-bag, or repository-source change is part of this task. No commit was made. All task-owned ROS nodes have been stopped.

The final integrity audit verified 1,181 tracked files and all 318 frozen protocol inputs, with no changed or missing tracked files, no frozen-input mismatches, and the same pre-existing dirty working-tree status. The audit is recorded in `repo_integrity.json`; run provenance and helper hashes are recorded in `run_manifest.json`.
