# odom_bridge

LiDAR-anchored localization, exploration, reactive avoidance and 3D mapping nodes for the VENTRA indoor stack.

| Item | Detail |
|---|---|
| Package type | `ament_python` |
| Target | ROS 2 Jazzy, PX4 with uXRCE-DDS |
| Upstream dependencies | `px4_msgs`, `rf2o_laser_odometry`, `slam_toolbox`, `nav2_bringup` (exploration only) |

---

## 1. Problem Statement

With optical flow as the only horizontal aiding source, EKF2 reset its horizontal position to approximately (0, 0) whenever flow quality dropped (low light, low texture, or the sensor out of range). This package replaces flow as the primary horizontal reference with a pose derived from 2D LiDAR scan matching and SLAM. That pose does not degrade with lighting or floor texture.

---

## 2. Localization Chain

```
 LiDAR /scan
     │
     ▼
 rf2o_laser_odometry        frame-to-frame scan matching
     │                      publishes TF odom → base_link and /odom_rf2o
     ▼
 slam_toolbox (online_async) scan-to-map matching and loop closure
     │                      uses odom → base_link as the motion prior
     │                      publishes TF map → odom
     ▼
 vision_odom_bridge         looks up map → base_link, converts ENU/FLU → FRD
     │
     ▼
 /fmu/in/vehicle_visual_odometry  →  EKF2 (EKF2_EV_CTRL)
```

`odom_bridge_node` runs alongside the chain. It converts PX4 odometry to `/odom` for visualization and logging, and publishes the static `base_link → laser` transform. It does not publish `odom → base_link`; that edge belongs to RF2O.

### 2.1 TF ownership

| Edge | Publisher | Type |
|---|---|---|
| `map → odom` | `slam_toolbox` | Dynamic |
| `odom → base_link` | `rf2o_laser_odometry` | Dynamic |
| `base_link → laser` | `odom_bridge_node` | Static (default z = 0.07 m) |
| `map → base_link_3d` | `pose_3d_node` | Dynamic, separate branch for 3D mapping |

A frame can have only one parent, and two publishers on one edge corrupt the tree. `odom_bridge_node` therefore defaults to `publish_dynamic_tf: false`. Set it to `true` only when RF2O is not running, for example to check the PX4-derived TF in RViz before the SLAM chain is available.

RF2O publishes its odometry on `/odom_rf2o`, not `/odom`, so that it does not collide with the topic published by `odom_bridge_node`. Topic names can coexist; only the TF edge is exclusive.

### 2.2 External-vision message

`vision_odom_bridge` reads the composed `map → base_link` transform from tf2 rather than a topic. This guarantees the published pose always includes the latest SLAM correction regardless of which node updated which edge last.

| Field | Value |
|---|---|
| Rate | 10 Hz (`publish_rate_hz`) |
| `pose_frame` | `POSE_FRAME_FRD`: the SLAM map is anchored at the start pose and has no north reference |
| Position variance | 0.02 m² per axis |
| Orientation variance | 0.01 rad² per axis |
| Velocity, angular velocity | NaN (not estimated; EKF2 fuses position and yaw only) |

Field names and enums in `px4_msgs/msg/VehicleOdometry` have changed between PX4 releases. Confirm them against the installed version with `ros2 interface show px4_msgs/msg/VehicleOdometry`.

---

## 3. Node Reference

| Executable | Subscribes | Publishes | Function |
|---|---|---|---|
| `odom_bridge_node` | `/fmu/out/vehicle_odometry` | `/odom`, static TF | PX4 NED/FRD to ROS ENU/FLU conversion, including velocity frame and covariance |
| `vision_odom_bridge` | TF `map → base_link` | `/fmu/in/vehicle_visual_odometry` | SLAM pose to EKF2 external vision |
| `pose_3d_node` | TF `map → base_link`, `/fmu/out/vehicle_odometry` | TF `map → base_link_3d`, `/drone/pose_3d`, `/drone/path_3d`, `/drone/marker_3d` | Combines SLAM x, y, yaw with PX4 z, roll, pitch |
| `scan_3d_mapper` | `/scan`, TF `map → base_link_3d` | `/map_3d` (PointCloud2) | 10 cm voxel map; services `~/save` (writes `.ply` to `~/maps`) and `~/clear` |
| `frontier_explorer` | `/map`, TF `map → base_link` | `navigate_to_pose` goals, `/explore/frontiers` | Frontier selection for Nav2; services `~/pause`, `~/resume`, `~/reset` |
| `reactive_avoidance` | `/scan`, `/cmd_vel`, `/offboard_velocity_cmd` | `/cmd_vel_safe`, `/offboard_velocity_cmd_safe`, `/avoidance/status` | Braking-limited velocity filter; service `~/enable` |

### 3.1 Reactive avoidance

PX4 Collision Prevention acts only on Position-mode stick input and has no effect on Offboard setpoints. Avoidance for Offboard flight must therefore be applied before the setpoint reaches PX4. `reactive_avoidance` sits between the command sources and the Offboard control node:

```
 Nav2 ──────── /cmd_vel ───────────────┐
                                       ├──► reactive_avoidance ──► /cmd_vel_safe
 web_teleop ── /offboard_velocity_cmd ─┘                       └──► /offboard_velocity_cmd_safe
                                                                          │
                                         Offboard control node ◄──────────┘
                                         (started with -r remaps)
```

For each candidate heading the node computes the free distance d along a corridor of width 2 × (`robot_radius` + `safety_margin`). The allowable speed is the largest v that satisfies

d − d_stop = v · t_r + v² / (2a)

that is, the vehicle coasts for the reaction time t_r (`reaction_time`, 0.4 s) and then decelerates at a (`max_decel`, 1.0 m/s²), stopping d_stop (`stop_distance`, 0.45 m) short of the obstacle. The result is capped at `max_speed`. Yaw rate is never limited. If no scan has arrived within `scan_timeout`, horizontal velocity is set to zero.

A command that is already within its limit passes unchanged. Otherwise the node searches within ±`max_steer_deg` for the heading that maximizes progress along the original direction, which produces sliding along walls rather than a stop.

The Offboard control node must subscribe to the filtered topics. The GCS launcher starts it with:

```bash
--ros-args -r /cmd_vel:=/cmd_vel_safe -r /offboard_velocity_cmd:=/offboard_velocity_cmd_safe
```

Without these remaps the filter is bypassed.

---

## 4. Installation

`rf2o_laser_odometry` is not packaged in apt for Jazzy. This workspace includes it under `src/`. To build it separately:

```bash
cd ~/px4_ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/px4_ros2_ws
colcon build --packages-select rf2o_laser_odometry odom_bridge
source install/setup.bash
```

---

## 5. Launch

| Method | Clock | Use |
|---|---|---|
| `run_cmds/slam.sh` | System | Hardware. Starts the four stages with fixed delays. Edit the `slam_params_file` path for the target machine. |
| GCS launcher, SLAM sequence | System | Hardware. Same order, with start/stop control from the browser. |
| `ros2 launch odom_bridge full_stack_launch.py` | Simulation (`use_sim_time: true`) | PX4 SITL with Gazebo. Delays of 0, 5, 8 and 11 s between stages. |

The stages are started in sequence so that each one has the upstream data it requires. If SLAM Toolbox starts before RF2O has published `odom → base_link` and `/scan` is flowing, it remains idle without any error.

SLAM Toolbox parameters are in [`config/slam_toolbox_params.yaml`](config/slam_toolbox_params.yaml): `odom_frame: odom`, `base_frame: base_link`, `map_frame: map`, `scan_topic: /scan`, `mode: mapping`, resolution 0.05 m.

---

## 6. EKF2 Configuration

Set in QGroundControl or with `param set` on the MAVLink console.

| Parameter | Setting | Reason |
|---|---|---|
| `EKF2_EV_CTRL` | HPOS + yaw (1 + 8 = 9). Add VPOS (2) only if SLAM height is wanted. | Enables external-vision fusion |
| `EKF2_EV_DELAY` | Measured pipeline latency; start at 50–100 ms and tune from the ULog | Incorrect delay makes fusion noisy or causes rejection |
| `EKF2_OF_CTRL` | Keep at 1 until vision fusion is verified; reduce or disable afterwards if flow still disturbs the estimate | Prevents flow from pulling the estimate |
| `EKF2_HGT_REF` | Barometer (0). Do not enable VPOS while another height reference is active. | Avoids two conflicting height sources |

Do not disable optical flow before verifying in the ULog that external-vision innovations remain small and that no new reset events occur under the conditions that previously caused them (dim lighting, plain walls and floors).

---

## 7. Verification

| Check | Command or source | Expected |
|---|---|---|
| Scan input | `ros2 topic hz /scan` | ≈ 10 Hz |
| TF tree | `ros2 run tf2_tools view_frames` | `map → odom → base_link → laser`; one publisher per edge |
| Vision output | `ros2 topic hz /fmu/in/vehicle_visual_odometry` | ≈ 10 Hz |
| EKF2 acceptance | ULog `estimator_innovations`, `estimator_status_flags` | External-vision position fusion active; innovations small |
| Reset behaviour | ULog `vehicle_odometry.reset_counter` | Does not increment over low-texture or dim areas |
