# Exact terminal workflow

All commands run in `/home/xin/FMCW_LIDAR`. Helpers, configs and recordings are temporary. No ROS1 source bag was converted, edited or replaced. No system package was installed. ROS2 does not need roscore.

## Raw inspection: start Terminal 1, then Terminal 3, then Terminal 2

Terminal 1 — RViz, raw rich cloud, top view in the established analysis axes (native X/Z plane):

```bash
cd /home/xin/FMCW_LIDAR
source /tmp/fmcw_rviz_check/environment.sh
rviz2 -d /tmp/fmcw_rviz_check/raw_100m_cross_5_f12_top.rviz
```

Terminal 3 — frozen markers and frame diagnostics:

```bash
cd /home/xin/FMCW_LIDAR
source /tmp/fmcw_rviz_check/environment.sh
python3 /tmp/fmcw_rviz_check/markers.py --sensitivity-10m
```

Terminal 2 — original ROS1 bag read-only, published over ROS2 without a converted bag:

```bash
cd /home/xin/FMCW_LIDAR
source /tmp/fmcw_rviz_check/environment.sh
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 100m_cross_5 --frames 12,14,16,28,31 --rate .25 --hold
```

The raw player holds each selected frame. Press `n` in Terminal 2 to advance, Space to resume/pause and `q` to quit. Equivalent service controls from another sourced terminal:

```bash
ros2 service call /fmcw_diag/next std_srvs/srv/Trigger '{}'
ros2 service call /fmcw_diag/set_paused std_srvs/srv/SetBool '{data: false}'
ros2 service call /fmcw_diag/set_paused std_srvs/srv/SetBool '{data: true}'
```

To continuously play at .25x, omit `--hold`. Rates .2 through .5 are useful initially. Selected-frame playback retains the original time gaps between selected frames. Omit `--frames` for the full source sequence; that full-source workflow was not recorded in this task.

For side view use RViz File → Open Config → `/tmp/fmcw_rviz_check/raw_100m_cross_5_f12_side.rviz`. These configs change only the camera. These views follow the established analysis axes; they do not establish an independently surveyed gravity frame. Native sensor horizontal is X/Z and analysis height is -native Y. Standard native XY TopDownOrtho is therefore a side projection, not top-down in the established analysis axes. The top config uses Orbit yaw 4.712389, pitch approximately 0; side uses pitch approximately 1.5707. Restore the config or use the saved focal point if you move the camera. Use Focus Camera on the green/red sphere, then zoom until the compact returns are visible.

Default cloud settings: fixed frame `aqronos_lidar`, PointCloud2 `/aqronos_cloud_0`, Position Transformer `XYZ`, Color Transformer `Intensity`, Channel Name `intensity`, Style `Points`, Size 5 pixels (increase to 8 if needed), Decay Time 0. Display colour range 0–80 affects colours only; it does not reject points. Red centre is provisional projected GNSS; green centre is existing observed anchor; red great circles have radius 3 m and optional thin 10 m. Full GNSS trajectory and separately fragmented observed-anchor trajectory are drawn. Missing anchors remain missing.

Optional velocity colour: keep Color Transformer `Intensity`, change Channel Name to `velocity`; use display bounds -3 to +3, or open `/tmp/fmcw_rviz_check/raw_velocity.rviz`. This reads the existing scalar field, changes no data, and is not a velocity threshold. Intensity remains the default.

After understanding `100m_cross_5`, stop Terminal 2 and use:

```bash
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 100m_cross_10 --frames 18,19,21,22 --rate .25 --hold
```

Open `/tmp/fmcw_rviz_check/raw_100m_cross_10_f18_top.rviz` or the corresponding `_side.rviz` config. Terminal 3 automatically follows exact incoming raw identities across conditions.

Only after the 100 m views, inspect far cases one source at a time:

```bash
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 200m_cross_5 --frames 3,10,21 --rate .25 --hold
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 200m_cross_10 --frames 36,40 --rate .25 --hold
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 300m_cross_10 --frames 0,4,9 --rate .25 --hold
```

These three commands are alternatives, not simultaneous publishers. Corresponding focal configs include `raw_200m_cross_5_f21_top.rviz`, `raw_200m_cross_10_f36_top.rviz`, `raw_300m_cross_10_f04_top.rviz`, and their `_side.rviz` partners. ZIP-only sources must be streamed through earlier records; a delay before the first selected frame is normal. All table diagnostics can also be printed without playback:

```bash
python3 /tmp/fmcw_rviz_check/markers.py --condition 200m_cross_5 --print-frames 3,10,21
cat /tmp/fmcw_rviz_check/SELECTED_FRAMES.md
```

## Create a new processed recording

Stop raw playback first. Keep the raw RViz and raw marker node available. Start Terminal 4, then Terminal 5, wait for recording subscription, then restart Terminal 2.

Terminal 4 — execute unchanged original get_lidar callback through the isolated ROS2 adapter:

```bash
cd /home/xin/FMCW_LIDAR
source /tmp/fmcw_rviz_check/environment.sh
python3 /tmp/fmcw_rviz_check/get_lidar_ros2.py --ros-args -r /aqronos_cloud:=/aqronos_cloud_0
```

Terminal 5 — record only the actual source-resolved output topic into a new path:

```bash
cd /home/xin/FMCW_LIDAR
source /tmp/fmcw_rviz_check/environment.sh
ros2 bag record /aqronos_cloud_81 -o /tmp/fmcw_rviz_check/processed/user_100m_cross_5 --max-cache-size 100000000
```

`user_100m_cross_5` is deliberately a new path; rosbag refuses an existing output directory. Do not replace either checked bag. Wait for `Subscribed to topic '/aqronos_cloud_81'` before playing input.

Terminal 2:

```bash
python3 /tmp/fmcw_rviz_check/raw_play.py --condition 100m_cross_5 --frames 12,14,16,28,31 --rate .25
```

Wait for five processed-frame log lines in Terminal 4, then Ctrl-C Terminal 5 to flush the bag; stop Terminal 4 and Terminal 2. The original callback can take several seconds per 1.25-million-slot frame; output receipt time is not acquisition time.

For 100m_cross_10, repeat with Terminal 5 output `/tmp/fmcw_rviz_check/processed/user_100m_cross_10` and Terminal 2 `--condition 100m_cross_10 --frames 18,19,21,22 --rate .25`. Wait for four processed frames.

## Independently replay processed data

Stop the processing node and raw player before this stage. Stop the raw-marker process and replace it with the processed-marker command. The following uses the already recorded and verified bag; no new recording is needed.

Terminal 1:

```bash
source /tmp/fmcw_rviz_check/environment.sh
rviz2 -d /tmp/fmcw_rviz_check/processed_100m_cross_5_f12_top.rviz
```

Terminal 3:

```bash
source /tmp/fmcw_rviz_check/environment.sh
python3 /tmp/fmcw_rviz_check/markers.py --condition 100m_cross_5 --cloud-topic /aqronos_cloud_81 --sensitivity-10m
```

Terminal 2:

```bash
source /tmp/fmcw_rviz_check/environment.sh
ros2 bag play /tmp/fmcw_rviz_check/processed/100m_cross_5_checked --rate .5 --read-ahead-queue-size 10 --start-paused
```

Press right arrow to display one recorded frame; Space toggles playback; up/down arrows change playback rate by 10%. Side config is `/tmp/fmcw_rviz_check/processed_100m_cross_5_f12_side.rviz`. Playback queue size 10 was tested; queue size 1 stalled and was stopped. When replaying your new recording, replace the bag path with `/tmp/fmcw_rviz_check/processed/user_100m_cross_5`.

For the second already verified bag, stop Terminal 2 and Terminal 3, use the matching condition and camera:

```bash
rviz2 -d /tmp/fmcw_rviz_check/processed_100m_cross_10_f18_top.rviz
python3 /tmp/fmcw_rviz_check/markers.py --condition 100m_cross_10 --cloud-topic /aqronos_cloud_81 --sensitivity-10m
ros2 bag play /tmp/fmcw_rviz_check/processed/100m_cross_10_checked --rate .5 --read-ahead-queue-size 10 --start-paused
```

Run these in Terminals 1/3/2 respectively. The processed RViz topic is `/aqronos_cloud_81`, with the same verified frame and default Intensity/intensity colour settings. Colouring `velocity` is also available through `/tmp/fmcw_rviz_check/processed_velocity.rviz`.

Processed bag record times are new 2026-10-13 receipt times and include processing delays. Original 2026-02-05 bag/frame identity and frozen alignment govern diagnostics. Historical 2022 header sec/nsec is used only as an exact table key, never as GNSS UTC or a replacement acquisition clock.
