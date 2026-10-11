# PX4 SITL Teleoperation (Keyboard and Joystick)

Simulation environment for developing and testing Offboard velocity control. A PX4 SITL quadrotor in Gazebo is flown from the keyboard or a joystick through ROS 2, with QGroundControl for monitoring and the Micro XRCE-DDS Agent as the PX4 to ROS 2 bridge.

Adapted from [MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick](https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick) (maintainer: [Curious-Utkarsh](https://github.com/Curious-Utkarsh)). The bring-up has been modified for the VENTRA workspace: GPS-denied vehicle model, indoor world, and Gazebo clock and LiDAR bridges.

Demonstration video (upstream): https://github.com/user-attachments/assets/ecb34f04-d7c9-4618-aa5d-2d075a3f07f9

---

## 1. Packages

| Package | Contents |
|---|---|
| `px4_bringup` | `minimal.launch.py`: starts the complete simulation environment |
| `px4_offboard` | `velocity_offboard_control.py` (Offboard controller), `teleop_keyboard.py`, `teleop_joystick.py`, launch files, `config/joy_config.yaml` |

### 1.1 `minimal.launch.py`

| Process | Detail |
|---|---|
| PX4 SITL | Headless; model `gz_x500_gps_denied`; world `hospital`; spawn pose (0, 15, 0) |
| Micro XRCE-DDS Agent | `MicroXRCEAgent udp4 -p 8888` |
| Gazebo GUI | `gz sim -g` |
| QGroundControl | `~/Downloads/QGroundControl-x86_64.AppImage` |
| Clock bridge | `/clock` (Gazebo to ROS) |
| LiDAR bridge | Gazebo `lidar_2d_v2` scan remapped to `/scan` |

PX4 is expected at `~/PX4-Autopilot`. The `gz_x500_gps_denied` model and `hospital` world must exist in that tree. If the LiDAR topic differs, list the available topics with `gz topic -l | grep -i scan` and update `GZ_LIDAR_TOPIC` in the launch file.

### 1.2 Command interface

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/offboard_velocity_cmd` | `geometry_msgs/Twist` | Teleop → controller | Velocity and yaw-rate command |
| `/arm_message` | `std_msgs/Bool` | Teleop → controller | Arm (true) or disarm (false) |
| `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` | `px4_msgs` | Controller → PX4 | Offboard heartbeat, setpoints, commands |
| `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_attitude` | `px4_msgs` | PX4 → controller | State feedback |

The controller subscribes to the versioned status topic `vehicle_status_v1` used by recent PX4 main. Change the topic name in `velocity_offboard_control.py` if the PX4 version publishes a different name (`ros2 topic list | grep vehicle_status`).

---

## 2. Requirements

| Item | Version |
|---|---|
| Operating system | Ubuntu 24.04 (Ubuntu 22.04 with ROS 2 Humble is also supported upstream) |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (installed by the PX4 setup script) |
| PX4 | PX4-Autopilot main |
| Disk | 8–10 GB free |

---

## 3. Installation

### 3.1 ROS 2

Install ROS 2 Jazzy (`ros-jazzy-desktop` and `ros-dev-tools`) by following the [official Ubuntu installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 3.2 PX4 Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot
make px4_sitl
```

Verify with `make px4_sitl gz_x500`. Gazebo should open with an x500 quadrotor.

### 3.3 Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Verify with `MicroXRCEAgent udp4 -p 8888`. The agent reports that it is listening on port 8888.

### 3.4 QGroundControl

Download the AppImage from the [QGroundControl download page](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) to `~/Downloads`, then:

```bash
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

QGroundControl connects automatically when PX4 SITL is running.

### 3.5 Workspace

In the VENTRA workspace these packages are already present under `src/`. Install dependencies and build:

```bash
cd ~/px4_ros2_ws
sudo rosdep init        # first time only
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 4. Operation

### 4.1 Start the simulation

Terminal 1:

```bash
ros2 launch px4_bringup minimal.launch.py
```

Wait until the vehicle appears in Gazebo and QGroundControl shows a connected vehicle.

### 4.2 Start teleoperation

Terminal 2, one of:

```bash
ros2 launch px4_offboard keyboard_teleop.launch.py
ros2 launch px4_offboard joystick_teleop.launch.py
```

### 4.3 Keyboard map

| Key | Action |
|---|---|
| Space | Toggle arm / disarm |
| W / S | Throttle up / down (incremental) |
| Arrow keys | Forward, backward, left, right (while held) |
| A / D | Yaw left / right (while held) |
| Ctrl+C | Exit |

### 4.4 Manual PX4 commands

Entered in the PX4 SITL console for a quick check without ROS:

```
commander arm
commander takeoff
commander land
commander disarm
```

---

## 5. Verification

```bash
ros2 topic list | grep fmu
```

Expected topics include `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` and `/fmu/out/vehicle_status*`. To confirm data flow from PX4:

```bash
ros2 topic echo /fmu/out/vehicle_attitude --once
```

---

## 6. Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| QGroundControl does not start | Missing runtime libraries | Install `libfuse2` and the GStreamer plugins |
| Agent reports port in use | Another agent instance running | `pkill -x MicroXRCEAgent` |
| No `/fmu` topics | PX4 or agent not running, or `ROS_DOMAIN_ID` mismatch | Check both processes; confirm the domain ID |
| No `/scan` | Gazebo LiDAR topic name differs | Update `GZ_LIDAR_TOPIC` in `minimal.launch.py` |
| Vehicle does not respond to commands | Controller subscribed to a status topic PX4 does not publish | Check the `vehicle_status` topic name (§1.2) |

---

## 7. References

- [PX4 development environment (Ubuntu)](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)
- [PX4 ROS 2 user guide and `px4_ros_com`](https://github.com/PX4/px4_ros_com)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- [QGroundControl user guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/index.html)
