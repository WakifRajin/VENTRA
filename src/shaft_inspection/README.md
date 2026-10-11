# shaft_inspection

Autonomous inspection of vertical shafts (mine shafts, ventilation shafts, wells) with a GPS-denied PX4 multicopter. The vehicle descends the shaft while holding the bore centre, turns around above the floor, climbs back out, and produces a 3D point cloud and a radius-versus-depth profile. The only sensors used are a 360° 2D LiDAR and a downward ToF rangefinder with optical flow. No GNSS, SLAM or telemetry link is required during the inspection.

| Item | Detail |
|---|---|
| Package type | `ament_python` |
| Simulation | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4-Autopilot main (v1.18 alpha) |
| Vehicle target | Pixhawk 6C with PX4 v1.14.x, Raspberry Pi 5 |
| Hardware procedure | [docs/HARDWARE_GUIDE.md](docs/HARDWARE_GUIDE.md) |

This document covers the design, the simulation and the parameter reference. The hardware build and test procedure is in the hardware guide.

---

## Contents

1. [Verification status](#1-verification-status)
2. [Concept of operation](#2-concept-of-operation)
3. [Target hardware](#3-target-hardware)
4. [System architecture](#4-system-architecture)
5. [Design description](#5-design-description)
6. [Simulation setup](#6-simulation-setup)
7. [Flying a mission in simulation](#7-flying-a-mission-in-simulation)
8. [Web dashboard](#8-web-dashboard)
9. [Outputs](#9-outputs)
10. [Parameter reference](#10-parameter-reference)
11. [Troubleshooting](#11-troubleshooting)
12. [Known limitations](#12-known-limitations)
13. [Repository map](#13-repository-map)

---

## 1. Verification Status

| Item | Status | Evidence |
|---|---|---|
| Fully autonomous mission (`auto_launch`): takeoff, centre, descend 24.8 m, turn around 0.5 m above the floor, climb, return to launch, disarm | **Passed in SITL** | End-to-end flight, no wall contact |
| Bore-centre estimator on circular, rectangular, elliptical, D-shaped and rough sections | **Passed** | 1–5 cm agreement with the world geometry; ≈ 3 ms per scan |
| Pilot handover (`pilot_handover`): manual flight, switch to Offboard, mission runs, pilot resumes control | **Partially verified** | Handover gating verified; complete handover flight not yet confirmed |
| Physical vehicle | **Not flown** | Parameters and message definitions checked against PX4 v1.14.4 source; staged test plan in the hardware guide |

---

## 2. Concept of Operation

```
      Pilot positions the vehicle over the mouth (Position mode, flow-aided)
                    │  switch to OFFBOARD
                    ▼
   ┌──────────── collar ─────────────┐
   │   CENTER  : servo to bore axis  │
   │   DESCEND : sink while centred  │   LiDAR (360°, horizontal) measures the walls
   │     │                           │   → bore centre → horizontal position
   │     ▼                           │
   │   brake below 2.5 m             │
   │   turn around 0.5 m above floor │   ToF (downward) measures the floor
   │     │                           │
   │   ASCEND  : climb while centred │
   │   EXIT    : stop at handover    │
   └─────────────────────────────────┘
                    │  pilot moves the sticks → Position mode
                    ▼
            Pilot flies to the landing point
```

### 2.1 Why SLAM is not used

A vertical bore provides almost no geometric constraint along its own axis. 2D scan matching (RF2O, SLAM Toolbox) cannot observe vertical motion and interprets changes in wall shape as lateral motion, which would drive the vehicle into the wall. 3D LiDAR-inertial odometry degenerates along the axis for the same reason. The estimation problem is therefore decomposed:

| Quantity | Source |
|---|---|
| Horizontal position | Bore centre measured directly in every LiDAR scan. Absolute and non-drifting; fused by EKF2 as external vision. |
| Depth | EKF2 height (barometer reference); ToF used for terrain |
| Floor detection | ToF rangefinder |
| Wall avoidance | Centring, repulsion and velocity clamp, all from the LiDAR |
| 3D map | Scans stacked at known depth (mapping with known pose) |

---

## 3. Target Hardware

| Part | Model | Function |
|---|---|---|
| Flight controller | Holybro Pixhawk 6C, PX4 v1.14.x | Attitude control, EKF2, position control, failsafes |
| Companion computer | Raspberry Pi 5, 8 GB | All ROS 2 nodes |
| 2D LiDAR | Slamtec RPLIDAR C1: 360°, 0.05–12 m, 10 Hz, ≈ 500 points per scan, ±30 mm | Walls, bore centre, obstacles, map |
| Downward sensor | Holybro H-Flow (DroneCAN): PAA3905 flow (42° FOV, 7.4 rad/s) + Broadcom AFBR-S50 ToF (0.08–30 m typical) | Floor distance, position hold near a surface |
| Airframe | x500-class quadrotor, ≈ 0.5 m tip to tip | |

The simulation models these sensors (§6.3).

---

## 4. System Architecture

### 4.1 Nodes

```
 RPLIDAR C1 ──/scan──► shaft_perception ──/fmu/in/vehicle_visual_odometry──► PX4 EKF2
                           │  /shaft/offset, /shaft/clearance,                  ▲
                           │  /shaft/sector_min, /shaft/repulsion, /shaft/valid │
                           ▼                                                    │
 PX4 ──status, local position, attitude──► shaft_mission ──offboard_control_mode, trajectory_setpoint,
                                               │            vehicle_command──────► PX4
                                               │  /shaft/state, /shaft/depth, /shaft/cmd_vel_ned, /shaft/budget
                                               ▼
                    shaft_mapper ──► ~/shaft_maps/*.pcd, *.csv, /shaft/cloud
                    shaft_dashboard ──► http://<host>:<port>  (read-only)
```

| Node | Function |
|---|---|
| `shaft_perception` | De-tilts each scan, finds the bore centre, computes clearances and repulsion, publishes the horizontal position fix to EKF2 |
| `shaft_mission` | State machine, control and obstacle avoidance. The only node that commands PX4. |
| `shaft_mapper` | Accumulates the 3D point cloud and the depth profile |
| `shaft_dashboard` | Web dashboard. Subscribes only; never commands. |
| `shaft_preflight` | Go/no-go check (command-line tool) |
| `shaft_mount_check` | Measures the LiDAR mounting yaw offset (command-line tool) |

### 4.2 Topics

| Topic | Type | Content |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR driver (SITL: Gazebo bridge) to perception, mapper and dashboard |
| `/down_range` | `sensor_msgs/LaserScan` (1 beam) | SITL only: Gazebo bridge to mission (`range_source: laserscan`) |
| `/shaft/offset` | `geometry_msgs/Vector3Stamped` | Vehicle displacement from the bore centre, level body frame (x forward, y left) |
| `/shaft/clearance` | `std_msgs/Float32` | Radius of the largest wall-free circle at the centre, m |
| `/shaft/valid` | `std_msgs/Bool` | Bore fix usable for this scan |
| `/shaft/roundness`, `/shaft/radius` | `std_msgs/Float32` | Shape descriptors; radius published only when the section is round |
| `/shaft/sector_min` | `std_msgs/Float32MultiArray` | Nearest return in 36 sectors of 10°, level body frame |
| `/shaft/repulsion` | `geometry_msgs/Vector3Stamped` | Push-away velocity from close returns |
| `/shaft/state` | `std_msgs/String` | Mission state name |
| `/shaft/depth` | `std_msgs/Float32` | Depth below the handover (datum) point, m |
| `/shaft/cmd_vel_ned` | `geometry_msgs/Vector3Stamped` | Velocity commanded by the mission (also published in dry run) |
| `/shaft/budget` | `std_msgs/Float32MultiArray` | Return-budget telemetry: battery available, battery required, time used, time limit, climb time (−1 where unavailable) |
| `/shaft/cloud` | `sensor_msgs/PointCloud2` | Accumulated map, frame `shaft` |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | Bore-centre fix to EKF2 |
| `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` | `px4_msgs` | Mission to PX4 |
| `/fmu/out/vehicle_status`, `vehicle_local_position`, `vehicle_attitude`, `manual_control_setpoint`, `failsafe_flags` | `px4_msgs` | PX4 to nodes |

PX4 releases after v1.14 append a version suffix to some topic names (`/fmu/out/vehicle_status_v4`, `vehicle_local_position_v1`). All nodes resolve the live name at startup (`px4_topics.resolve`), so the same code runs against v1.14 (unversioned) and newer PX4.

---

## 5. Design Description

### 5.1 Perception: bore-centre estimation

For each scan (`shaft_perception_node.py`, `scan_geometry.py`):

1. **Mounting correction.** `lidar_yaw_offset_deg` rotates the scan so that 0° is the vehicle nose; `lidar_upside_down` mirrors it. An incorrect value rotates every centring command and causes the vehicle to spiral into the wall.
2. **Self filter.** Returns closer than `self_filter_radius` (0.25 m) come from the airframe and are discarded.
3. **De-tilt.** Each beam is rotated by the vehicle roll and pitch into a level frame. When the vehicle tilts, the scan plane tilts, a round shaft appears elliptical and its apparent centre shifts. Without de-tilting, the controller chases a false offset. PX4 attitude is FRD and the scan is FLU, so pitch changes sign between them.
4. **Coverage gate.** The scan is binned into 180 bearings. If fewer than 85 % contain a return, the fix is rejected rather than estimated, because a large gap biases the result towards "centred".
5. **Centre as the point of maximum clearance.** The free space inside the scan is rasterized at 4 cm and a Euclidean distance transform gives each cell's distance to the nearest wall. The maximum is the safest point and is valid for round, square, D-shaped and irregular sections. Two conditions apply:
   - Candidates must lie inside the scan polygon. A point outside the wall is also far from the wall.
   - For elongated sections the maximum is a line rather than a point (every point on a rectangle's long centreline ties). Near-ties are resolved with the area centroid of the cross-section.
6. **Outputs.** Offset from the centre, clearance, per-sector nearest returns, a repulsion vector, and a roundness score. A circle-fit radius is reported only when the section is round.

### 5.2 Position fix to PX4

The offset from the bore centre is rotated into NED using the current heading, added to a shaft-axis position anchored once at startup, and published as `VehicleOdometry` with horizontal position only (`EKF2_EV_CTRL = 1`):

| Field | Value | Reason |
|---|---|---|
| Position x, y | Bore-relative position in NED | Horizontal fix |
| Position z | EKF2's own height | EKF2 discards the whole sample if any position component is NaN |
| Attitude | NaN | A symmetric bore provides no heading |
| Position variance | 0.09 m² | Conservative; the fix is only as absolute as the bore is prismatic |
| Timestamp | System clock | The uXRCE-DDS client synchronizes to the host clock. Sim-time stamps cause PX4 to reject every sample while the topics appear connected. |

Where the cross-section changes shape with depth, the computed centre moves; the variance accounts for this.

### 5.3 Height, terrain and the collar edge

A downward rangefinder measures the distance to whatever is below the vehicle, not the vehicle's height. The two are equal only over flat ground. EKF2 therefore maintains two separate states:

```
   vehicle height  h   (above a fixed reference: the EKF2 origin)
   terrain height  t   (height of the ground below the vehicle, same reference)
   rangefinder reading  r = h − t
```

| Quantity | Source | Used for |
|---|---|---|
| Vehicle height `h` | EKF2: IMU with barometer reference (`EKF2_HGT_REF = 0`) | Height hold, depth, return to the datum |
| Terrain height `t` | EKF2 from the ToF (`EKF2_RNG_CTRL = 1`, conditional) | Flow scaling, low-altitude aiding, land detection |
| Raw floor distance `r` | ToF | Turnaround decision only |

**Crossing the collar edge.** Hovering 1 m above the ledge, r = 1 m. After moving laterally over the shaft, the next sample reads r = 26 m. EKF2 must explain the 25 m step: either the vehicle climbed 25 m in 50 ms, or the ground dropped away. The IMU and barometer recorded no climb, so the measurement is rejected as a height change and EKF2 resets the terrain estimate (t decreases by 25 m) while h is unchanged. The position controller holds h and the vehicle continues level.

If the ToF were the height reference (`EKF2_HGT_REF = 2`), EKF2 would force h to follow r. The vehicle would believe it had climbed to 26 m and descend rapidly to correct. For this reason the rangefinder is never used as the height reference.

`EKF2_RNG_CTRL = 1` (conditional) allows the ToF to aid height only while the vehicle is slow (< `EKF2_RNG_A_VMAX`, 1 m/s) and low (< `EKF2_RNG_A_HMAX`, 3 m), such as during takeoff and landing where rotor wash disturbs the barometer. Over the shaft r exceeds 3 m and the aiding is disabled automatically.

**Short-term height** is provided by the integrated IMU accelerometer, which is smooth and responsive but drifts. The barometer corrects the drift continuously. Neither depends on the surface below the vehicle.

**Barometer suitability:**

| Condition | Barometer behaviour | Consequence |
|---|---|---|
| Hover in still air | Good; 0.1–0.3 m noise | Acceptable |
| Close to the ground or walls | Disturbed by rotor wash (ground effect; sign depends on the airframe) | Covered by the conditional ToF aiding during takeoff and landing |
| Inside a narrow shaft | Recirculating wash causes pressure disturbances of several decimetres | The vehicle may oscillate vertically; tolerable at the low speeds used |
| 25 m descent | Real, slowly changing pressure (≈ 0.12 hPa/m) | Measured well; this is the depth signal |
| Draughts or forced ventilation | Slow pressure offsets | Slow depth bias of decimetres; ToF turnaround unaffected |
| Temperature change underground | Slow drift | As above |

The barometer is accurate for slow, large height changes (depth) and poor for fast, small changes near surfaces. The IMU and the conditional ToF aiding cover the barometer's weak regime. The one decision that requires precision, the turnaround 0.5 m above the floor, uses the ToF directly, so a barometer error of a few decimetres cannot drive the vehicle into the floor.

Depth is h relative to the handover point. A floor beyond the ToF range is acceptable: the vehicle descends on barometric height until the floor enters ToF range.

**Barometer installation:** cover the barometer with open-cell foam and shield it from propeller wash. This matters more in a shaft than in open air.

### 5.4 Obstacle avoidance

PX4 Collision Prevention (`CP_DIST`) acts only on Position-mode stick input and has no effect on Offboard setpoints. It is enabled to protect manual flight, and the mission adds its own layers:

| Layer | Function | Protects against |
|---|---|---|
| 1. Centring | PD servo to the bore centre (`kp_lateral`, `kd_lateral`) | Approaching the walls at all |
| 2. Repulsion | Push away from any return within 1 m | Ledges, pipes and cables ignored by the centre estimate |
| 3. Velocity clamp | No commanded speed toward a sector in which the vehicle cannot stop before `stop_dist` (0.65 m) | All remaining cases; applied last, to the final vector |

Additional gates:

- Descent is permitted only while centred (`center_tol` 0.12 m, with hysteresis) and with clearance ≥ `descent_min_clearance` (0.60 m).
- Stale scan data (> 0.5 s) stops lateral motion and descent.
- A return inside `critical_dist` (0.45 m) triggers ABORT.

All distances are measured from the LiDAR centre. The propeller tips of an x500 extend to ≈ 0.38 m.

### 5.5 Mission state machine

```
auto_launch (SITL):
WAIT → ARM → TAKEOFF → CENTER → DESCEND → ASCEND → EXIT → RETURN → TOUCHDOWN → LAND → DONE

pilot_handover (vehicle, and manual testing in SITL):
WAIT ──(armed by pilot, in OFFBOARD, bore fix valid)──► CENTER → DESCEND → ASCEND → EXIT → DONE
                                                          ▲                                  │
                             any state: pilot leaves OFFBOARD (sticks or switch) ──► DONE ◄──┘
```

| State | Behaviour |
|---|---|
| `WAIT` | Streams zero-velocity setpoints so that PX4 accepts Offboard. In `pilot_handover` the node never arms the vehicle. |
| `CENTER` | Centres; holds for 1.5 s; records the depth datum |
| `DESCEND` | Descends at 0.35 m/s while gated; brakes from 2.5 m above the floor; turns around at 0.5 m. The vehicle does not land in the shaft (dust, debris, propeller strike). |
| `ASCEND` | Climbs at 0.5 m/s while centring |
| `EXIT` | Climbs to the datum height. In `pilot_handover` the next state is `DONE` and the vehicle hovers for the pilot. |
| `RETURN`, `TOUCHDOWN`, `LAND` | `auto_launch` only: fly to the recorded launch point, descend, land and disarm. PX4 Land mode is never used over the bore because it descends vertically. |
| `ABORT` | Entered on a return inside `critical_dist`, mission timeout, or a failed takeoff. Climbs, then exits or returns. |

### 5.6 Return budget

The shaft depth does not need to be known in advance. During `CENTER` and `DESCEND` the mission checks continuously whether it must start climbing, and switches to `ASCEND` when the first of these conditions is met:

| Condition | Rule | Parameters |
|---|---|---|
| Battery | Remaining charge ≤ reserve + drain rate × required climb time | `battery_reserve`, `drain_window_s` |
| Flight time | Time since arming + required climb time ≥ budget | `flight_time_budget_s` |
| Depth | Depth ≥ limit | `max_depth` |

Required climb time = (depth / `ascend_speed` + `exit_buffer_s`) × `return_margin`. The drain rate is a least-squares fit over the last `drain_window_s` of battery samples.

Reaching a budget limit is a planned turnaround, not an abort. PX4 v1.14 does not export `battery_status` over uXRCE-DDS, so on the vehicle only the flight-time and depth conditions are active (see the hardware guide, §5.6). If all three are disabled, descent is limited only by detecting the floor; the node logs a warning in this case.

### 5.7 Mapping

Each scan is a horizontal slice at a known depth. The mapper stores one slice per 5 cm of depth and produces two outputs: the point cloud, and a radius profile (clearance, mean and maximum radius, roundness versus depth). In a symmetric bore the heading drifts and smears the cloud azimuthally. The profile is unaffected and is the primary metric output.

---

## 6. Simulation Setup

### 6.1 Installation

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic and PX4-Autopilot main (v1.18 alpha).

```bash
# 1. ROS workspace (this package is part of the VENTRA repository)
git clone https://github.com/WakifRajin/VENTRA.git ~/px4_ros2_ws
cd ~/px4_ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to shaft_inspection
mkdir -p run_cmds && cp src/shaft_inspection/run_cmds/*.sh run_cmds/

# 2. PX4 additions: airframe, vehicle model, world
S=~/px4_ros2_ws/src/shaft_inspection/px4
cd ~/PX4-Autopilot
cp $S/airframes/4023_gz_x500_shaft ROMFS/px4fmu_common/init.d-posix/airframes/
sed -i 's/^\t4022_gz_x500_gps_denied$/&\n\t4023_gz_x500_shaft/; t; s/^\t4001_gz_x500$/&\n\t4023_gz_x500_shaft/' \
    ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt   # or add the entry manually
cp -r $S/models/x500_shaft Tools/simulation/gz/models/
cp $S/worlds/vshaft.sdf Tools/simulation/gz/worlds/

# 3. Only if the PX4 tree contains commit 162cce35d8 (see §6.4)
git apply $S/patches/revert-lockstep-162cce35d8.patch

make px4_sitl_default
```

`px4_msgs` must match the PX4 version in use.

Additional dependencies: Micro XRCE-DDS Agent (`MicroXRCEAgent` on the `PATH`), `ros-jazzy-ros-gz-bridge`, `python3-scipy`, `pip install websockets`, and QGroundControl at `~/Downloads/QGroundControl-x86_64.AppImage`.

After changing code, the airframe, the model or the world, rebuild with:

```bash
cd ~/PX4-Autopilot && make px4_sitl_default
cd ~/px4_ros2_ws && colcon build --packages-select shaft_inspection --symlink-install
```

### 6.2 Starting the simulation

Full environment (Gazebo GUI, QGroundControl, RViz, dashboard, manual handover):

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
GUI=1 QGC=1 RVIZ=1 MODE=pilot_handover ./run_cmds/shaft_sim.sh
```

Open `http://localhost:8080` for the dashboard. Startup takes about one minute. The system is ready when the dashboard shows all five links green and *Mission: WAIT*.

Other configurations:

```bash
./run_cmds/shaft_sim.sh                         # headless, fully autonomous mission
GUI=1 RVIZ=1 ./run_cmds/shaft_sim.sh            # observe the autonomous mission
GUI=1 QGC=1 RVIZ=1 GDB=1 MODE=pilot_handover ./run_cmds/shaft_sim.sh   # PX4 under gdb
```

| Variable | Default | Effect |
|---|---|---|
| `MODE` | `auto_launch` | `auto_launch` or `pilot_handover` |
| `GUI` | 0 | Gazebo GUI |
| `RVIZ` | 0 | RViz with the map and live scan |
| `QGC` | 0 | Start QGroundControl (reused if running; never terminated by the script) |
| `GDB` | 0 | Run PX4 under gdb; the backtrace is written to the log |
| `HEADLESS` | 1 | PX4's own Gazebo rendering |

The script terminates stale simulation processes, then starts the XRCE agent, PX4 SITL (`gz_x500_shaft` in world `vshaft`), the optional GUIs and the ROS stack. Logs are written to `log/shaft_<timestamp>/` (`px4.log`, `stack.log`, `agent.log`).

**Stopping.** Press Ctrl+C in the terminal running the script. This stops PX4, Gazebo, RViz, the agent and all nodes. QGroundControl is left running so that joystick settings are saved when it is closed from its own window.

If processes remain after an unclean exit, the next launch removes them. To remove them manually, run each line separately:

```bash
pkill -9 -f "px4_sitl_default/bin/px4"
pkill -9 -f "gz sim"
pkill -9 -x MicroXRCEAgent
pkill -9 -f "lib/ros_gz_bridge/parameter_bridge"
pkill -9 -f "lib/shaft_inspection/shaft_"
pkill -9 -x rviz2
```

Avoid patterns such as `pkill -f shaft_sim` from a shell whose own command line contains that text: `pkill -f` matches and terminates the calling shell.

### 6.3 Simulated environment

**World `vshaft`** (`~/PX4-Autopilot/Tools/simulation/gz/worlds/vshaft.sdf`). The cross-section changes with depth by design:

| Depth | Section |
|---|---|
| +3.0 to 0.0 m | Headframe chamber, R = 3.2 m; launch ledge at z = 0 |
| 0 to −5 m | Circular, R = 1.50 m |
| −5 to −11 m | Rectangular, 3.0 × 2.4 m |
| −11 to −17 m | Rough-cut, R ≈ 1.45 ± 0.3 m, with a ledge at −14.5 m |
| −17 to −25 m | D-shaped, R = 1.38 m with one flat wall |

Two service pipes run down the upper sections. The vehicle spawns on the ledge 2.2 m from the shaft axis.

**Model `x500_shaft`** (`Tools/simulation/gz/models/x500_shaft/`):

| Sensor | Simulation |
|---|---|
| RPLIDAR C1 | `gpu_lidar`, 360°, 500 samples, 10 Hz, 0.05–12 m, σ = 15 mm |
| H-Flow ToF | Single-ray `gpu_lidar`, downward, 20 Hz, 0.08–30 m, σ = 20 mm |
| H-Flow optical flow | PX4 optical-flow camera model (42° FOV, 7.4 rad/s) |

**Airframe `4023_gz_x500_shaft`** sets the SITL PX4 configuration: GPS off, external vision horizontal position only, barometer height reference, conditional range aiding, flow limited to 5 m, Collision Prevention, confined-space speed limits and joystick input. Simulation-only settings: magnetometer arming checks and supply check disabled.

### 6.4 Changes to the local PX4 tree

| Change | Reason | Revert |
|---|---|---|
| Airframe `4023_gz_x500_shaft` and its CMake entry | Simulation vehicle | Delete the file and entry |
| `x500_shaft` model and `vshaft` world | Simulation | Delete |
| Revert of commit `162cce35d8` in the lockstep scheduler (uncommitted) | Defect in PX4 main: wake-ups are broadcast after releasing a lock onto stack-local condition variables, causing `Fatal glibc error: pthread_mutex_lock … __owner == 0` or a silent freeze. Not present in any release, including v1.14; NuttX (Pixhawk) unaffected. | `git -C ~/PX4-Autopilot checkout -- platforms/posix/src/px4/common/lockstep_scheduler` |

---

## 7. Flying a Mission in Simulation

### 7.1 Autonomous mode (`MODE=auto_launch`)

Start the script and observe. The node arms, takes off, flies the mission, returns to the ledge and disarms.

### 7.2 Pilot handover mode (`MODE=pilot_handover`)

**QGroundControl joystick setup (once):** Vehicle Setup → Joystick → enable, select the controller, calibrate (keep "centre stick is zero throttle"), and assign buttons. Suggested assignments: Position, Offboard, Arm, Disarm. Settings persist because the run script never terminates QGroundControl.

**Procedure:**

1. Select **Position** mode. Do not arm in Hold: Hold, Mission, Return, Takeoff and Land require a GPS position, and PX4 refuses with "resolve system health failures".
2. Arm and climb off the ledge.
3. Fly over the centre of the shaft and hover less than about 3 m above the ledge (the top of the chamber). Above that height the LiDAR sees no walls, horizontal position is lost and PX4 falls back to Altitude mode. The dashboard must show *bore fix: valid*.
4. Switch to **Offboard**. *In control* changes to **MISSION**.
5. The mission centres, descends, turns around, climbs to the handover height and stops in `DONE`.
6. Move the sticks. PX4 switches to Position mode (`MAN_OVERRIDE_SPD`). This also aborts the mission at any time.
7. Fly back over the ledge and land. Hold the throttle stick fully down for about 3 s after touchdown. PX4 declares "landed" only while descent is commanded; a centred stick commands height hold and disarming is refused.

---

## 8. Web Dashboard

Read-only; it cannot command the vehicle. Started by the launch file (`dashboard:=false` to disable).

| Context | Address |
|---|---|
| SITL (`shaft_bringup.launch.py`) | `http://localhost:8080` |
| Vehicle (`shaft_real.yaml`) | `http://<pi-address>:8081`; port 8080 is used by the GCS launcher, which embeds the dashboard |

| Panel | Content |
|---|---|
| In control | PILOT / MISSION / FAILSAFE / ON GROUND, with a warning when arming is attempted in a GPS-dependent mode |
| PX4 mode, arming, mission | Current mode (and the pilot-selected mode during failsafes), arming state, mission state |
| Links | PX4, LiDAR, perception, mission node, joystick. Green with rate, red with age. |
| Mission strip | State progression |
| Bore | Top-down LiDAR returns, bore centre, clearance circle, nearest wall, commanded velocity |
| Depth and floor | Depth, ToF floor distance, clearance over time |
| Estimator | EKF2 horizontal and height validity, ToF-backed floor distance, position, velocity, attitude |
| Failsafe flags | Active flags |
| Pilot input | Both sticks, 16 buttons, stick-movement takeover indicator |
| Timeline | Mode changes (including pilot takeover and handover to the mission), mission transitions, warnings |

---

## 9. Outputs

Written to `~/shaft_maps/` (`out_dir`) when the mission reaches `DONE`, and on shutdown:

| File | Content |
|---|---|
| `shaft_cloud_<time>.pcd` | Point cloud; frame on the shaft axis, z = −depth |
| `shaft_profile_<time>.csv` | Columns `depth_m, clearance_m, r_mean_m, r_max_m, roundness` |

PX4 flight logs (ULog) are in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`.

---

## 10. Parameter Reference

Values are those in effect in SITL: `config/shaft.yaml` where set, otherwise the code default (marked †). Vehicle values are in `shaft_real.yaml` at the workspace root and are described in the hardware guide §8.

### 10.1 `shaft_perception`

| Parameter | Value | Description |
|---|---|---|
| `min_coverage` | 0.85 | Fraction of bearings that must contain a return |
| `search_window` | 6.0 m | Upper bound on the raster size |
| `min_clearance_valid` / `max_clearance_valid` | 0.35 / 8.0 m | Plausibility limits |
| `self_filter_radius` | 0.25 m | Discard returns from the airframe |
| `lidar_yaw_offset_deg` † | 0 | Scan zero relative to the vehicle nose (measure on hardware) |
| `lidar_upside_down` † | false | Mirrored mounting |
| `repulse_influence` / `repulse_gain` / `repulse_max` | 1.0 m / 0.35 / 0.6 m/s | Repulsion field |
| `roundness_for_radius` | 0.93 | Report a circle radius only above this roundness |
| `anchor_samples` | 30 | Scans used to anchor the shaft axis |
| `publish_ev` | true | Send the fix to EKF2 |

### 10.2 `shaft_mission`

| Parameter | Value | Description |
|---|---|---|
| `start_mode` | Launch argument | `auto_launch` or `pilot_handover`. Intentionally not set in `shaft.yaml`. |
| `range_source` † | `laserscan` | SITL: `/down_range`. Vehicle: `local_position` (EKF2 `dist_bottom`, accepted only when ToF-backed). |
| `dry_run` † | false | Compute everything, publish nothing to PX4 |
| `control_hz` | 20 Hz | Control loop rate |
| `kp_lateral` / `kd_lateral` | 1.2 / 0.45 | Centring PD gains |
| `max_lateral_speed` | 0.6 m/s | |
| `center_tol` / `center_settle_speed` / `center_hold_s` | 0.12 m / 0.15 m/s / 1.5 s | Definition of "centred" |
| `descend_speed` / `ascend_speed` | 0.35 / 0.5 m/s | |
| `kp_vertical` / `max_vertical_speed` | 0.6 / 0.8 m/s | Vertical position control |
| `slowdown_range` / `bottom_range` / `bottom_confirm_n` | 2.5 m / 0.5 m / 3 | Floor approach and turnaround |
| `max_depth` | 0 (off) | Turnaround depth; a planned turnaround, not an abort |
| `mission_timeout` | 0 (off) | Hard ABORT after this time; last resort only |
| `battery_reserve` / `return_margin` | 0.30 / 1.5 | Return budget (§5.6) |
| `exit_buffer_s` / `drain_window_s` | 60 s / 60 s | Return budget (§5.6) |
| `flight_time_budget_s` | 0 (off) | Usable endurance from arming; used when no battery data is available |
| `stop_dist` / `critical_dist` / `descent_min_clearance` | 0.65 / 0.45 / 0.60 m | Avoidance, measured from the LiDAR centre |
| `brake_delay` / `brake_decel` | 0.4 s / 1.0 m/s² | Velocity-clamp braking model |
| `scan_timeout` | 0.5 s | Stale data stops motion |
| `use_repulsion` | true | Enable layer 2 |
| `auto_descend` | true | false = hover-and-centre test only |
| `takeoff_clearance` | 1.0 m | `auto_launch` only |
| `return_tol`, `touchdown_speed`, `touchdown_range` † | 0.12 m, 0.3 m/s, 0.30 m | `auto_launch` only |

### 10.3 Launch arguments (`shaft_bringup.launch.py`)

| Argument | Default |
|---|---|
| `start_mode` | `auto_launch` |
| `dashboard` | `true` |
| `autostart` | `true` |

This launch file is for simulation only; it starts the Gazebo bridges. The vehicle is started as described in the hardware guide.

---

## 11. Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| QGroundControl: *no global position estimate*, arming refused | Arming attempted in Hold (GPS-dependent mode) | Switch to Position, then arm |
| Vehicle does not disarm after landing | Land detector requires commanded descent | Hold throttle fully down for about 3 s after touchdown |
| Mission remains in `WAIT` in handover mode | Not armed, not in Offboard, or no bore fix | Check the dashboard; hover lower over the shaft |
| *Offboard engaged but no bore fix, holding* | Vehicle above the chamber or not over the shaft | Descend below about 3 m and move over the shaft |
| Vehicle drifts while hovering high in the chamber | No walls within LiDAR range, so no horizontal position | Remain low, near the shaft mouth |
| Joystick assignments lost | QGroundControl terminated before saving | Close QGroundControl from its own window |
| `Fatal glibc error: pthread_mutex_lock … __owner == 0`, or PX4 freezes in flight | PX4 main lockstep defect | Apply the revert in §6.4; re-check after updating PX4 |
| `gazebo already running world: default` | Stale Gazebo server | The run script clears it; otherwise `pkill -9 -f "gz sim"` |
| PX4 topics exist but no data reaches a node | Versioned topic names, or DDS discovery race | Nodes resolve names and wait up to 5 s; restart the node if PX4 started much later |
| EKF2 ignores the vision fix (`xy_valid` false) | NaN in the message, incorrect timestamps, or `EKF2_EV_CTRL` | Handled in code; on the vehicle, check the parameters |
| Launch argument `start_mode` ignored | A node-specific YAML key takes precedence over launch parameters | Keep `start_mode` out of `shaft.yaml` |
| Mission aborts on the ground with a small clearance | Returns from the airframe | Increase `self_filter_radius` |

---

## 12. Known Limitations

- **Heading underground.** A symmetric bore provides no heading, and magnetometers are unreliable near steel and ore. Heading drift smears the point cloud azimuthally. The profile CSV is unaffected.
- **Companion-computer failure at depth.** Without the LiDAR fix PX4 has no horizontal position (optical flow is unusable at depth). Loss of Offboard falls back to Altitude mode; there is no safe autonomous recovery. Use a tether for early flights and run the nodes as auto-restarting services.
- **Downwash.** Reflection of rotor wash from the shaft walls is not simulated. Keep bores at least 2 m in diameter (about 4 × the x500 span; 1.5 m absolute minimum).
- **ToF range on dark, wet rock** will be well below the 30 m rating.
- **Firmware difference.** SITL runs PX4 main; the vehicle runs v1.14. Estimator, commander and failsafe behaviour may differ in detail.
- **Land detector.** In SITL the autonomous landing required the force-disarm fallback. Verify landing and disarm on hardware.
- **Pilot handover** has not yet been confirmed in a complete flight.

---

## 13. Repository Map

In the repository the PX4 additions are under `px4/` (airframe, model, world, lockstep patch) and the scripts under `run_cmds/`. After installation they are located as follows:

```
px4_ros2_ws/
├── shaft_real.yaml                 vehicle parameters
├── run_cmds/
│   ├── shaft_sim.sh                one-command simulation
│   └── px4_shaft_gdb.sh            PX4 under gdb (GDB=1)
└── src/shaft_inspection/
    ├── shaft_inspection/
    │   ├── scan_geometry.py        de-tilt, bore centre (EDT), sectors, repulsion, clamp
    │   ├── shaft_perception_node.py
    │   ├── shaft_mission_node.py
    │   ├── shaft_mapper_node.py
    │   ├── shaft_dashboard_node.py
    │   ├── preflight_check.py      shaft_preflight
    │   ├── lidar_mount_check.py    shaft_mount_check
    │   └── px4_topics.py           QoS, versioned topic names, PX4 timestamps
    ├── launch/shaft_bringup.launch.py   simulation stack
    ├── config/shaft.yaml, shaft.rviz    simulation parameters, RViz layout
    ├── web/dashboard.html
    ├── deploy/px4_v1.14_shaft.params            QGroundControl import (vehicle)
    ├── deploy/px4_v1.14_shaft_params_explained.txt
    └── docs/HARDWARE_GUIDE.md

~/PX4-Autopilot/  (local additions)
├── ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_x500_shaft
├── Tools/simulation/gz/models/x500_shaft/
└── Tools/simulation/gz/worlds/vshaft.sdf
```
