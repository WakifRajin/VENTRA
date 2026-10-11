# shaft_inspection: Hardware Implementation Guide

Build, configuration, test and operation of `shaft_inspection` on the physical vehicle.

| Item | Detail |
|---|---|
| Configuration | Pixhawk 6C, PX4 v1.14.x, Raspberry Pi 5 (8 GB), RPLIDAR C1, Holybro H-Flow, x500/S500-class quadrotor |
| Prerequisite reading | [README](../README.md): design and rationale. This guide covers procedure, in the order it should be carried out. |
| Status | Flown end-to-end in simulation only. Parameters and message definitions checked against PX4 v1.14.4 source. Not yet flown on hardware. |

> **Safety.** Carry out the stages in order and do not skip the bench tests. Use a tether and propeller guards until the vehicle has proven itself. Follow local regulations and site safety rules for underground work. Items marked **[verify]** could not be confirmed without the physical hardware.

---

## Contents

1. [Bill of materials](#1-bill-of-materials)
2. [Differences between simulation and vehicle](#2-differences-between-simulation-and-vehicle)
3. [Mechanical build](#3-mechanical-build)
4. [Electrical wiring](#4-electrical-wiring)
5. [Flight controller: firmware and parameters](#5-flight-controller-firmware-and-parameters)
6. [Holybro H-Flow configuration](#6-holybro-h-flow-configuration)
7. [Raspberry Pi 5: operating system and software](#7-raspberry-pi-5-operating-system-and-software)
8. [Stack configuration for the vehicle](#8-stack-configuration-for-the-vehicle)
9. [Running the stack](#9-running-the-stack)
10. [Calibration and bench tests (propellers removed)](#10-calibration-and-bench-tests-propellers-removed)
11. [Staged flight testing](#11-staged-flight-testing)
12. [Tuning from logs](#12-tuning-from-logs)
13. [Operating procedure and checklists](#13-operating-procedure-and-checklists)
14. [Troubleshooting](#14-troubleshooting)
15. [Open risks before unattended underground use](#15-open-risks-before-unattended-underground-use)

---

## 1. Bill of Materials

| Item | Notes |
|---|---|
| x500/S500-class frame, motors, ESCs, propellers | ≈ 0.5 m tip to tip. Requires payload margin for the Pi 5, LiDAR and cooling (≈ 250–300 g). |
| Holybro Pixhawk 6C | With power module (PM) |
| Raspberry Pi 5, 8 GB | With the official active cooler. Mandatory: the Pi throttles in an enclosed frame without it. |
| Pi storage | 64 GB or larger A2 microSD, or NVMe HAT and SSD (faster, more tolerant of vibration) |
| RPLIDAR C1 | Includes USB-UART adapter board |
| Holybro H-Flow (DroneCAN) | With JST-GH 4-pin CAN cable |
| Compute power supply | 5 V / 5 A BEC rated for the battery voltage (recommended), or a 5 V USB power bank (as on the VENTRA prototype) |
| JST-GH 6-pin cable and female Dupont jumpers | Pixhawk TELEM2 to Pi GPIO UART |
| Short USB cable | LiDAR adapter to Pi (match the adapter connector) |
| RC transmitter and receiver | Primary pilot link on the vehicle (§2) |
| Propeller guards or full cage | Strongly recommended in any confined space |
| Tether (tension line and reel) | For all early shaft flights |
| Downward LED light | H-Flow optical flow requires > 5 lux; the ToF does not |
| Open-cell foam | Barometer cover |
| Vibration-damping mounts | For the Pi and LiDAR |
| Laptop with QGroundControl | Setup, parameters, logs |
| Wi-Fi access to the Pi | SSH and dashboard; not required in flight |

---

## 2. Differences Between Simulation and Vehicle

| Aspect | Simulation | Vehicle |
|---|---|---|
| PX4 version | main (1.18 alpha) | **v1.14.x** |
| `px4_msgs` | main (the workspace carries 1.17.0) | **`release/1.14` branch**; must match the firmware |
| Topic names | Some versioned (`vehicle_status_v4`) | Unversioned (resolved automatically) |
| PX4 to ROS link | UDP to a local agent | Serial: Pixhawk TELEM2 to Pi UART, 921 600 baud |
| LiDAR data | Gazebo bridge | `sllidar_ros2` driver over USB |
| Floor distance in ROS | Gazebo bridge `/down_range` | EKF2 `dist_bottom` (`range_source: local_position`). v1.14 does not export the rangefinder over DDS. |
| Flow and ToF into PX4 | Gazebo plugin | DroneCAN from the H-Flow |
| Battery state in ROS | Available | Not exported by v1.14 over DDS (§5.6) |
| Pilot input | Gamepad through QGroundControl (MAVLink) | RC transmitter (`COM_RC_IN_MODE 0`) |
| Mission start | `start_mode:=auto_launch` or `pilot_handover` | `pilot_handover` only |
| Stick takeover from Offboard | Always enabled (newer PX4) | Requires `COM_RC_OVERRIDE = 3` (v1.14 default covers Auto modes only) |
| `use_sim_time` | true | false |
| Lockstep defect | Present in PX4 main (reverted locally) | Not present (v1.14, NuttX) |

**RC transmitter rather than a gamepad.** A gamepad reaches PX4 through QGroundControl and a telemetry radio. That path adds latency and is lost as soon as the vehicle is underground. An RC receiver on the vehicle provides direct, low-latency control for takeoff, handover, takeover and landing while in range. RC is usually also lost underground, which is why the mission is autonomous and `COM_RCL_EXCEPT` keeps Offboard active without RC.

---

## 3. Mechanical Build

### 3.1 Coordinate convention

The PX4 body frame is FRD: x forward, y right, z down, origin at the centre of gravity (CoG). Locate the CoG with the full payload and battery installed by balancing the frame along each axis. Use this frame for every offset measurement.

### 3.2 RPLIDAR C1 (top)

- **Scan plane above all other parts.** The laser sweeps a horizontal plane at the optical centre. Propellers, masts, antennas and cables must be below that plane, or they appear as fixed "walls" a few centimetres away. Raise the LiDAR on a standoff if necessary.
- **Unobstructed 360° view.** A blocked sector reduces coverage and can drop the bore fix below the 85 % coverage gate.
- **On or near the vertical axis through the CoG.** An offset is measurable and correctable (`EKF2_EV_POS_*`, §5.4), but should be small.
- **Level and rigid**, on vibration dampers. Tilt of the mount relative to the airframe is not compensated.
- **Cable routed downward** immediately; never across the scan plane.
- **Record the housing orientation** (for example, "cable toward the rear"). The software requires the angle between the LiDAR 0° direction and the nose; it is measured in §10.3 rather than taken from the marking.

### 3.3 Holybro H-Flow (underside)

- Facing straight down with a clear cone of at least 45°. The flow sensor field of view is 42°; the ToF is 12.4° × 6.2°. Landing gear, battery straps and wiring must remain outside the cone.
- **Orientation:** forward mark toward the nose. Check the exact marking and any orientation parameter against Holybro's H-Flow documentation for the unit **[verify]**.
- Mount rigidly; optical flow is sensitive to vibration.
- Keep the sensor away from landing-gear contact points so that debris does not strike the lens.

### 3.4 Raspberry Pi 5

- On dampers, with an unobstructed air path for the active cooler. CPU load is sustained; without airflow the Pi throttles within minutes.
- Away from the magnetometer and any GPS antenna (switching noise).

### 3.5 Barometer

The Pixhawk 6C barometer is internal. Cover the case vents with open-cell foam and shield the flight controller from direct propeller wash. Recirculating wash in a shaft makes this more important than in open air.

### 3.6 Offset record

Record these values; they are entered in §5.4.

| Offset (FRD from CoG, m) | x (forward) | y (right) | z (down) |
|---|---|---|---|
| H-Flow optical centre → `EKF2_OF_POS_*` | | | |
| H-Flow ToF (same unit) → `EKF2_RNG_POS_*` | | | |
| LiDAR optical centre → `EKF2_EV_POS_*` | | | |

---

## 4. Electrical Wiring

### 4.1 Overview

```
 Battery ──► Pixhawk power module ──► Pixhawk 6C (POWER1)
    │                                   ├── TELEM2 ──(TX/RX/GND only)──► Pi 5 GPIO UART
    │                                   ├── CAN1   ─────────────────────► H-Flow
    │                                   ├── RC IN  ─────────────────────► RC receiver
    │                                   ├── TELEM1 ─────────────────────► telemetry radio (optional)
    │                                   └── GPS1   ─────────────────────► GPS/compass mast (optional)
    │
    └──► 5 V / 5 A BEC ──► Raspberry Pi 5 (USB-C, or 5 V/GND header pins)
                               └── USB ──► RPLIDAR C1 adapter
```

### 4.2 Pixhawk TELEM2 to Raspberry Pi 5 UART

Pixhawk telemetry connectors are JST-GH 6-pin. Standard Pixhawk pinout, pin 1 being the marked wire. Check against the Holybro 6C pinout sheet before connecting **[verify]**.

| TELEM2 pin | Signal | Pi 5 header pin |
|---|---|---|
| 1 | +5 V | **Not connected** |
| 2 | TX (from Pixhawk) | Pin 10, GPIO15 / RXD |
| 3 | RX (to Pixhawk) | Pin 8, GPIO14 / TXD |
| 4 | CTS | Not connected |
| 5 | RTS | Not connected |
| 6 | GND | Pin 6, GND |

- TX connects to RX and RX to TX.
- **Do not connect the Pixhawk 5 V to the Pi.** The Pi has its own supply; two supplies connected through the header can damage either board.
- Both sides use 3.3 V logic; no level shifter is required.
- The common ground through pin 6 is required.

### 4.3 H-Flow to CAN1

Connect the H-Flow DroneCAN cable to CAN1. The CAN bus powers the sensor. If it is the only node on the bus, confirm bus termination as described in Holybro's documentation **[verify]**.

### 4.4 RPLIDAR C1 to Pi USB

Connect the LiDAR USB-UART adapter to a Pi USB port. The LiDAR is powered from USB. Use a short, strain-relieved cable.

### 4.5 Pi 5 power

The Pi 5 requires 5 V at up to 5 A.

| Option | Notes |
|---|---|
| BEC with USB-C PD output negotiating 5 A | Preferred |
| 5 A BEC to header pins 2/4 (5 V) and 6 (GND) | The Pi cannot negotiate current this way and limits USB port current by default. Add `usb_max_current_enable=1` to `/boot/firmware/config.txt` so the LiDAR receives enough current **[verify]** the setting for the OS image. |
| 5 V USB power bank | Used on the VENTRA prototype. Isolates the compute domain from the flight battery at the cost of mass and endurance. |

Do not power the Pi from the Pixhawk. Check for under-voltage under representative load (§10.1).

---

## 5. Flight Controller: Firmware and Parameters

### 5.1 Firmware and basic setup

1. QGroundControl → **Vehicle Setup → Firmware**. Connect the Pixhawk by USB, select **PX4 Pro** and the latest **v1.14.x** release.
2. **Airframe:** select the matching frame (for example, Holybro X500 V2, or the generic quadrotor X). Reboot.
3. **Sensors:** calibrate compass, gyroscope, accelerometer and level horizon with the Pi, LiDAR and H-Flow installed. They shift the CoG and add magnetic interference.
4. **Radio:** calibrate the RC transmitter.
5. **Flight modes** (one 3-position and one 2-position switch are sufficient):

   | Switch position | Mode |
   |---|---|
   | Mode switch 1 | Altitude (fallback; requires no position) |
   | Mode switch 2 | Position (normal manual flight) |
   | Mode switch 3 | Offboard (starts the inspection) |
   | Kill switch | Kill (motors off; last resort) |

6. **Power:** calibrate the power module voltage and current.
7. **Motors:** confirm rotation direction and order with propellers removed.

### 5.2 Serial link to the Pi (uXRCE-DDS)

| Parameter | Value |
|---|---|
| `UXRCE_DDS_CFG` | TELEM 2 |
| `SER_TEL2_BAUD` | 921600 8N1 (appears after setting the above and rebooting) |

Reboot. A telemetry radio, if used, remains on TELEM1 (MAVLink).

### 5.3 Shaft parameter set

`deploy/px4_v1.14_shaft.params` contains 48 parameters with types checked against the v1.14.4 source. Load it with **Parameters → Tools → Load from file**, then reboot. Each parameter is explained in `deploy/px4_v1.14_shaft_params_explained.txt`.

Parameters that differ from the v1.14 defaults and must be changed (all included in the file):

| Parameter | v1.14 default | Required | Reason |
|---|---|---|---|
| `EKF2_EV_CTRL` | 15 | **1** | Fuse horizontal position only from the bore fix (no yaw, no height) |
| `EKF2_HGT_REF` | 1 (GPS) | **0** | Barometer height reference (README §5.3) |
| `EKF2_GPS_CTRL` | 7 | **0** | No GPS |
| `EKF2_OF_CTRL` | 0 | **1** | H-Flow optical flow near surfaces |
| `COM_OBL_RC_ACT` | 0 | **1** | Offboard loss → Altitude (Land would descend into the shaft) |
| `COM_RC_OVERRIDE` | 1 | **3** | Stick input also overrides Offboard |
| `COM_RCL_EXCEPT` | 0 | **4** | Ignore RC loss while in Offboard (no RC underground) |
| `UAVCAN_ENABLE` | 0 | **2** | DroneCAN sensors (H-Flow) |

### 5.4 Parameters set from measurements

| Parameter | Source |
|---|---|
| `EKF2_OF_POS_X/Y/Z` | H-Flow optical-centre offset (§3.6) |
| `EKF2_RNG_POS_X/Y/Z` | H-Flow ToF offset (§3.6) |
| `EKF2_EV_POS_X/Y/Z` | LiDAR optical-centre offset (§3.6) |
| `MPC_THR_HOVER` | Hover log (§12) |
| `EKF2_EV_DELAY` | Start at 50 ms; tune from logs (§12) |

### 5.5 Site-dependent parameters

| Parameter | Guidance |
|---|---|
| `SYS_HAS_GPS` | 0 without GPS; 1 if a GPS mast is fitted for surface work |
| `EKF2_MAG_TYPE` | 0 (automatic) at the surface. Near steel or ore the compass is unreliable (README §12). |
| `COM_RC_IN_MODE` | 0 (RC transmitter only) on the vehicle. SITL uses 1 (joystick). |
| `SDLOG_MODE` | 1 (from boot until disarm), so the log includes the period before arming |
| `COM_DISARM_LAND` | Default 2 s |
| `CP_DIST` | 0.6 m; protects manual flight in Position mode |
| Battery failsafes (`BAT_*`, `COM_LOW_BAT_ACT`) | Set for the pack in use. The climb out requires more energy than the descent. |

### 5.6 Battery data for the return budget

PX4 v1.14 does not publish `battery_status` over uXRCE-DDS. Without it, the battery condition of the mission's return budget (README §5.6) is inactive and only `flight_time_budget_s` and `max_depth` can force a turnaround. Two options:

- **Configuration only (default):** measure usable hover endurance from arming to landing, and set `flight_time_budget_s` and `max_depth` in `shaft_real.yaml`.
- **Firmware change:** add `battery_status` to the published topics in `src/modules/uxrce_dds_client/dds_topics.yaml` and rebuild the v1.14 firmware **[verify]**.

---

## 6. Holybro H-Flow Configuration

1. With `UAVCAN_ENABLE = 2`, `UAVCAN_SUB_FLOW = 1` and `UAVCAN_SUB_RNG = 1` loaded, reboot the Pixhawk.
2. Confirm the H-Flow appears as a DroneCAN node in QGroundControl. Apply firmware updates and node-side settings, such as mounting orientation, as described in Holybro's documentation **[verify]**.
3. **Rangefinder check:** Analyze Tools → MAVLink Inspector → `DISTANCE_SENSOR`. Lift the vehicle by hand; `current_distance` must track height smoothly from about 0.1 m to at least 2 m.
4. **Flow direction check:** `OPTICAL_FLOW_RAD`. Over a textured floor, move the vehicle forward by hand, then to the right, and note the sign of the integrated flow. Rotate the vehicle and repeat. Correct any swapped or inverted axes before flight; incorrect flow axes cause Position mode to diverge.
5. Confirm that `EKF2_MIN_RNG` (0.08 m) is below the reading with the vehicle on the ground.

---

## 7. Raspberry Pi 5: Operating System and Software

### 7.1 Operating system

Flash Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi using Raspberry Pi Imager, with SSH and Wi-Fi enabled in the imager settings. After first boot:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git python3-pip build-essential cmake
sudo usermod -aG dialout $USER          # serial port access; log out and back in
```

**Enable the GPIO UART** (GPIO14/15) by adding to `/boot/firmware/config.txt`:

```
dtparam=uart0=on
```

Remove any serial login console on that UART (`console=serial0,...` or `console=ttyAMA0,...` in `/boot/firmware/cmdline.txt`), reboot, and identify the device:

```bash
ls -l /dev/ttyAMA* /dev/serial*
```

On a Pi 5 the header UART is normally `/dev/ttyAMA0`; the separate debug connector is `ttyAMA10`. Confirm on the installed image **[verify]**. This guide uses `/dev/ttyAMA0`.

**CPU governor:**

```bash
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

### 7.2 ROS 2 Jazzy

Install ROS 2 Jazzy from the official Debian packages for Ubuntu (arm64 is supported). The base install is sufficient on the vehicle:

```bash
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions \
                    python3-rosdep python3-numpy python3-scipy
pip3 install --user --break-system-packages websockets     # dashboard
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 7.3 Micro XRCE-DDS Agent

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig /usr/local/lib/
```

Test with the Pixhawk connected and powered:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Within a few seconds the agent logs session creation and topic registration.

### 7.4 ROS workspace

```bash
git clone https://github.com/WakifRajin/VENTRA.git ~/px4_ros2_ws
cd ~/px4_ros2_ws/src
rm -rf px4_msgs
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git      # must match v1.14 firmware
git clone https://github.com/Slamtec/sllidar_ros2.git              # RPLIDAR C1 driver
cd ~/px4_ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2      # px4_msgs takes a long time on a Pi
echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
```

The repository carries `px4_msgs` 1.17.0 for simulation; it is replaced above. A `px4_msgs` built from the wrong branch compiles without error and fails at runtime with misaligned fields or unmatched topics. If PX4 topics appear but their values are wrong, check this first.

### 7.5 Persistent LiDAR device name

The C1 adapter is a USB-serial device and its `/dev/ttyUSB*` number can change between boots. Read its identifiers with:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
```

Create `/etc/udev/rules.d/99-rplidar.rules`:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="<vendor>", ATTRS{idProduct}=="<product>", MODE="0666", SYMLINK+="rplidar"
```

Reload with `sudo udevadm control --reload-rules && sudo udevadm trigger`, reconnect the LiDAR, and use `/dev/rplidar` from then on.

### 7.6 LiDAR driver check

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar
ros2 topic hz /scan        # expect ≈ 10 Hz
```

C1 defaults in that launch file: 460 800 baud, `frame_id: laser`, `scan_mode: Standard`, `angle_compensate: true`.

---

## 8. Stack Configuration for the Vehicle

The simulation launch file starts Gazebo bridges and is not used on the vehicle. Vehicle parameters are in `shaft_real.yaml` at the workspace root (`~/px4_ros2_ws/shaft_real.yaml`). It sets `use_sim_time: false`, `start_mode: pilot_handover` and `range_source: local_position`, and reduces speeds for the first flights. `config/shaft.yaml` is the simulation configuration and must never be used on the vehicle.

`start_mode` is set in the YAML on the vehicle because no launch file is used; the YAML is the single source.

### 8.1 Values that differ from simulation

| Parameter | Simulation | Vehicle | Reason |
|---|---|---|---|
| `max_lateral_speed` | 0.6 m/s | 0.4 m/s | Conservative for first flights |
| `descend_speed` / `ascend_speed` | 0.35 / 0.5 m/s | 0.25 / 0.4 m/s | Conservative for first flights |
| `bottom_range` | 0.5 m | 0.8 m | Allows for late floor detection |
| `max_depth` | 0 (off) | 10.0 m | **SET** to the known shaft depth plus margin |
| `mission_timeout` | 0 (off) | 300 s | **SET** from measured endurance |
| `flight_time_budget_s` | 0 (off) | 0 | **SET** to measured usable endurance (§5.6) |
| `shaft_dashboard.port` | 8080 | 8081 | Port 8080 is used by the GCS launcher |
| `shaft_mapper.out_dir` | `~/shaft_maps` | `/home/abdazwd/shaft_maps` | **SET** for the user account |

### 8.2 Values to measure on the airframe

| Parameter | Nodes | Procedure |
|---|---|---|
| `lidar_yaw_offset_deg` | perception, mapper | §10.3 |
| `lidar_upside_down` | perception, mapper | §10.3 |
| `self_filter_radius` | perception, mapper | §10.4 |

Keep the perception and mapper values identical.

**Avoidance distances.** The defaults (`stop_dist` 0.65 m, `critical_dist` 0.45 m, `descent_min_clearance` 0.60 m) assume propeller tips about 0.38 m from the LiDAR vertical axis. If the reach is larger (propeller guards, larger propellers), increase all three by the difference.

---

## 9. Running the Stack

### 9.1 Manual start (bench work)

One terminal per process, or `tmux`:

```bash
P=~/px4_ros2_ws/shaft_real.yaml
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar
ros2 run shaft_inspection shaft_perception --ros-args --params-file $P
ros2 run shaft_inspection shaft_mission    --ros-args --params-file $P
ros2 run shaft_inspection shaft_mapper     --ros-args --params-file $P
ros2 run shaft_inspection shaft_dashboard  --ros-args --params-file $P
```

Dashboard: `http://<pi-address>:8081` from a laptop or phone on the same network.

### 9.2 GCS launcher

`python3 ~/px4_ros2_ws/drone_gcs.py`, then start the **Shaft** group at `http://<pi-address>:8080`. The launcher starts perception, mapper, dashboard and mission in that order, embeds the dashboard, and stops `vision_odom_bridge` and the Offboard control node automatically because they publish to the same PX4 topics.

### 9.3 Automatic start at boot (flight)

systemd restarts a failed node within one second. This matters because a failed node deep in a shaft leaves the vehicle without horizontal position. Use either systemd node services or the GCS launcher for the shaft nodes, not both.

`/etc/systemd/system/micro-xrce-agent.service`:

```ini
[Unit]
Description=Micro XRCE-DDS Agent (Pixhawk TELEM2)
After=network.target

[Service]
User=<user>
ExecStart=/usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/shaft-node@.service` (one template for every node):

```ini
[Unit]
Description=shaft_inspection %i
After=micro-xrce-agent.service

[Service]
User=<user>
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/px4_ros2_ws/install/setup.bash && exec ros2 run shaft_inspection %i --ros-args --params-file ~/px4_ros2_ws/shaft_real.yaml'
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

An equivalent `sllidar.service` runs `ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar`.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent sllidar \
     shaft-node@shaft_perception shaft-node@shaft_mission \
     shaft-node@shaft_mapper shaft-node@shaft_dashboard
journalctl -fu shaft-node@shaft_mission        # follow one node's log
```

A mission node that restarts mid-flight returns to `WAIT` and does not resume the mission. With `COM_OBL_RC_ACT = 1`, PX4 is already in Altitude mode by then (Offboard stream lost for longer than `COM_OF_LOSS_T`).

### 9.4 Onboard logging

```bash
mkdir -p ~/flights && cd ~/flights
ros2 bag record -o flight_$(date +%Y%m%d_%H%M%S) /scan /rosout \
    /shaft/offset /shaft/clearance /shaft/valid /shaft/state /shaft/depth \
    /shaft/cmd_vel_ned /shaft/sector_min /shaft/budget \
    /fmu/out/vehicle_status /fmu/out/vehicle_local_position \
    /fmu/out/vehicle_attitude /fmu/out/failsafe_flags
```

Together with the PX4 ULog on the SD card (`SDLOG_MODE = 1`), this is the only record of an underground flight.

---

## 10. Calibration and Bench Tests (Propellers Removed)

**Remove the propellers for every test in this section.**

### 10.1 Power and links

```bash
vcgencmd get_throttled          # 0x0: no under-voltage or throttling since boot
ros2 run shaft_inspection shaft_preflight
```

Expected: LiDAR at about 10 Hz, PX4 status arriving, `z_valid` true. Run the full stack (or `stress`) for 10 minutes, then repeat `get_throttled` and check `vcgencmd measure_temp`.

### 10.2 Rangefinder through EKF2

`shaft_preflight` → "H-Flow range (dist_bottom)" must report `from_range=True`. When the vehicle is lifted, the dashboard *Floor below (ToF)* value must follow the height.

### 10.3 LiDAR mounting offset

1. Vehicle on the ground with open space around it.
2. Hold a flat board 0.6–1.0 m directly in front of the nose. Run `ros2 run shaft_inspection shaft_mount_check` and set `lidar_yaw_offset_deg` to the printed value.
3. Restart perception. Move the board to the vehicle's left; it must read approximately +90° after the offset. A reading of approximately −90° means the scan is mirrored: set `lidar_upside_down: true`.

### 10.4 Self filter

With nothing within 1 m of the vehicle, inspect the dashboard bore view (or `shaft_preflight` "self-hits"). Returns that remain stationary come from the airframe. Set `self_filter_radius` a few centimetres beyond the farthest one, and well below `critical_dist`.

### 10.5 Sign test (mandatory)

Build a mock bore that can be walked around: a ring of boxes or boards, a tarpaulin enclosure, or the end of a corridor, at least 2 m across. Place the vehicle inside with the stack running.

1. Move the vehicle 20 cm toward the nose. In the dashboard bore view the blue centre marker must move 20 cm behind the vehicle; *Offset* ≈ 0.20.
2. In the dashboard *Estimator* panel, EKF2 north/east position must change by 20 cm in the direction the vehicle moved (NED, using the heading).
3. Rotate the vehicle 90° and repeat.

If any value moves in the wrong direction or by the wrong amount, **stop** and correct the mounting parameters. A sign error drives the vehicle into the wall at full speed.

### 10.6 EKF2 acceptance of the fix

In the mock bore the dashboard must show *horizontal position* valid. In the QGroundControl MAVLink Inspector (or the ULog), the EKF2 external-vision position fusion flag must be active. If `xy_valid` remains false, check `EKF2_EV_CTRL = 1`, the agent link, and that `px4_msgs` is `release/1.14`.

### 10.7 Mission dry run

Set `dry_run: true` and restart the mission node. It computes all commands and publishes nothing to PX4. In the mock bore, displace the vehicle by hand; the yellow command arrow on the dashboard must point back toward the bore centre. Set `dry_run: false` afterwards.

### 10.8 Failsafe behaviour

With propellers removed and the vehicle restrained:

1. Arm in Position mode and switch to Offboard (the mission node streams setpoints).
2. Stop the mission node: `sudo systemctl stop shaft-node@shaft_mission`, or Ctrl+C.
3. Within about 1 s PX4 must switch to **Altitude** (dashboard timeline or QGroundControl).
4. Move the sticks during Offboard. PX4 must switch to **Position** (`COM_RC_OVERRIDE = 3`).
5. Disarm.

### 10.9 Arming and disarming

- Arm only in Position or Altitude mode. Hold, Mission, Return, Takeoff and Land require GPS and are refused.
- After touchdown, hold the throttle down for about 3 s until PX4 declares the vehicle landed; only then will it disarm.

---

## 11. Staged Flight Testing

Do not advance to the next stage until every pass criterion of the current stage is met. Use a tether and propeller guards throughout stages 2 to 5.

| Stage | Location | Test | Pass criteria |
|---|---|---|---|
| 1. Bench | Workshop | All of §10 | Every check passes; sign test correct in two orientations |
| 2. Manual hover | Open space, 1–2 m | Position mode on flow and ToF only (no walls) | Holds position within about 0.3 m; no drift; landing and disarm work |
| 3. Terrain step | Open space | Fly slowly off a table or step edge in Position mode | ULog shows a terrain reset, not a height jump; vehicle remains level |
| 4. Centring hover | Mock bore ≥ 2.5 m, 1–2 m high | Offboard with `auto_descend: false` | Converges to within ±5 cm of centre, no oscillation; stick takeover works |
| 5. Obstacle | As stage 4 | Add a box protruding into the bore | Vehicle steers around or stops; never closer than `stop_dist` |
| 6. Short vertical | Stairwell, tower or elevator shaft, 3–8 m | Full mission, RC in range | Clean descent, turnaround, climb, `DONE`; takeover and landing |
| 7. Shallow shaft | ≤ 10–15 m deep, ≥ 2.5 m wide, tethered | Full mission after site survey | As stage 6; logs reviewed; battery ≥ 30 % at landing |
| 8. Full-depth shaft | After stage 7 is repeatable | Increase `max_depth` gradually | As above |

From stage 6 onward, raise speeds and reduce `bottom_range` toward the simulation values only after reviewing logs from previous flights.

---

## 12. Tuning from Logs

Open the PX4 ULog in PX4 Flight Review or PlotJuggler together with the rosbag.

| Symptom | Signals | Adjustment |
|---|---|---|
| Centring oscillates | `/shaft/offset`, `vehicle_local_position.vx/vy` | Reduce `kp_lateral`; increase `kd_lateral` |
| Centring slow | As above | Increase `kp_lateral` in small steps |
| EKF2 position lags the fix, or innovations spike with motion | `estimator_innovations` (external-vision horizontal position) against velocity | Tune `EKF2_EV_DELAY` (scan and processing latency; 30–100 ms) |
| Vision fix intermittently rejected | `estimator_innovation_test_ratios` for EV position | Increase `EKF2_EVP_GATE` slightly, or `EKF2_EVP_NOISE` |
| Vehicle oscillates vertically in the shaft | `vehicle_local_position.z`, barometer | Improve barometer foam; reduce `descend_speed`; a few decimetres is acceptable |
| Descent repeatedly gated | Dashboard timeline "descent gated (offset …, clearance …)" | The cause is real (off-centre or narrow section). Check the profile before widening any gate. |
| Hover throttle | `vehicle_thrust_setpoint` in steady hover | Set `MPC_THR_HOVER` |

---

## 13. Operating Procedure and Checklists

### 13.1 Site survey (before transporting the vehicle)

- [ ] Shaft diameter at the mouth and, if known, along its length (≥ 2 m preferred)
- [ ] Depth (sets `max_depth`)
- [ ] Obstructions: cables, ladders, platforms, pipes, water, loose material
- [ ] Airflow and ventilation: direction and strength
- [ ] Lighting at the mouth (flow requires > 5 lux for manual position hold)
- [ ] Dust and dripping water (degrade LiDAR and ToF)
- [ ] Safe takeoff and landing area near the mouth, clear of the edge
- [ ] Tether anchor point; exclusion zone for personnel
- [ ] Permissions and site safety briefing

### 13.2 Pre-flight

- [ ] Battery fully charged; endurance known; `flight_time_budget_s` and `mission_timeout` set below it
- [ ] Propellers, guards and tether attachment secure
- [ ] LiDAR lens and H-Flow window clean
- [ ] `max_depth` set for this shaft
- [ ] Power on; wait for all services
- [ ] `ros2 run shaft_inspection shaft_preflight` reports **GO**
- [ ] Dashboard: all links green; no unexpected failsafe flags
- [ ] Rosbag recording started
- [ ] RC: mode switch in Position; kill switch tested and off
- [ ] Briefing: pilot, tether handler, observer; abort call agreed

### 13.3 Flight

1. Arm in Position mode; take off to about 1 m; confirm position hold.
2. Fly over the shaft centre and stay low (walls within LiDAR range, less than about 3 m above the collar). Dashboard: *bore fix: valid*, *clearance* plausible.
3. Switch to Offboard. Dashboard: *In control: MISSION*.
4. Monitor the dashboard (while Wi-Fi is in range) and the tether.
5. **Abort at any time:** move the sticks (switches to Position) or set the mode switch to Altitude.
6. At `DONE` the vehicle hovers at the handover height. Take over with the sticks or mode switch and fly to the landing area.
7. Land; hold the throttle down for about 3 s; disarm.

### 13.4 Post-flight

- [ ] Stop the rosbag; copy the ULog (SD card) and the rosbag from the Pi
- [ ] Copy the shaft maps (profile CSV, point cloud)
- [ ] Record battery remaining, anomalies and dashboard timeline warnings
- [ ] Inspect propellers, guards, LiDAR and H-Flow

### 13.5 Emergencies

| Situation | Action |
|---|---|
| Vehicle drifts toward a wall in Offboard | Move the sticks (switches to Position) and fly away from the wall |
| Offboard lost or companion computer failed | PX4 is in Altitude mode. Fly up and out manually if RC is in range; otherwise recover with the tether. |
| RC lost underground during the mission | No action. The mission continues (`COM_RCL_EXCEPT`) and the vehicle hovers at `DONE` until RC is regained. |
| Low-battery warning in the shaft | Take over and climb out immediately |
| Loss of control near personnel | Kill switch |

---

## 14. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| Agent shows no session | TX/RX swapped, wrong device, wrong baud, `UXRCE_DDS_CFG` not TELEM2 | Re-check §4.2 and §5.2, `/dev/ttyAMA0`, serial console disabled |
| PX4 topics listed but values wrong or not received | `px4_msgs` not `release/1.14` | Rebuild with the correct branch (§7.4) |
| `/scan` missing | LiDAR power, `/dev/rplidar` rule, baud rate | Run `ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=...` and read its errors |
| Bore fix never valid in a real shaft | Coverage < 85 % (blocked sectors, dark or absorbing walls, beyond 12 m range), or self filter too large | Check the dashboard bore view; clear the mount; fly lower |
| Mission aborts on the ground | Airframe returns inside `critical_dist` | Increase `self_filter_radius` (§10.4) |
| Centring pushes toward the wall | Incorrect `lidar_yaw_offset_deg` or `lidar_upside_down` | Repeat §10.3 and §10.5 |
| Arming refused, "no global position" | Mode is Hold | Switch to Position |
| Vehicle does not disarm after landing | Land detector | Hold throttle down for about 3 s |
| Sticks do not override Offboard | `COM_RC_OVERRIDE` still 1 | Set to 3 |
| Switch to Offboard refused | Mission node not streaming, or no valid local position | Check dashboard links; the bore fix or flow must provide horizontal position |
| Height jumps when crossing the collar | `EKF2_HGT_REF = 2` (range) | Set to 0 (barometer) |
| Position mode diverges near the ground | Flow axes incorrect | §6 step 4 |
| Mission never turns around before the floor | No return budget: no battery data, `flight_time_budget_s` and `max_depth` both 0 | Set both (§5.6, §8.1) |
| Pi reboots or throttles in flight | Power or heat | 5 A supply, active cooler, `vcgencmd get_throttled` |

---

## 15. Open Risks Before Unattended Underground Use

| # | Risk | Mitigation |
|---|---|---|
| 1 | No reliable heading reference inside a symmetric bore rich in steel or ore | Centring is unaffected; the 3D map smears azimuthally. Use the profile as the primary output. |
| 2 | Companion-computer failure at depth; PX4 has no safe autonomous recovery without horizontal position | Auto-restarting services, tether, conservative depths until reliability is demonstrated |
| 3 | Downwash in narrow bores, not modelled in simulation | Characterize in stage 6 before entering real shafts; keep bores ≥ 2 m |
| 4 | ToF range on dark, wet rock may be well below 30 m | The vehicle descends on barometric height until the floor is detected; set `slowdown_range` and `bottom_range` for late detection at the chosen descent speed |
| 5 | Dust at the bottom blinds both LiDAR and ToF | The 0.5–0.8 m turnaround (no landing) limits but does not eliminate this |
| 6 | Firmware difference: SITL runs PX4 main, the vehicle runs v1.14 | Estimator and failsafe behaviour may differ; the staged tests are designed to expose this |
| 7 | Battery state not available to the mission on v1.14 | Measured `flight_time_budget_s`, or the firmware change in §5.6 |
