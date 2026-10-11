# Front-Obstacle Back-Off Test (MAVSDK + RPLIDAR C1)

Standalone test of LiDAR-triggered avoidance on the Raspberry Pi 5 and Pixhawk 6C, without ROS. When an obstacle enters a cone in front of the vehicle, the script commands a slow backward velocity in Offboard mode until the obstacle is outside the stop distance.

This test predates the ROS 2 `reactive_avoidance` node in `src/odom_bridge`, which supersedes it for normal operation.

---

## 1. Function

| Condition (minimum range in the front cone) | Mode | Commanded body velocity |
|---|---|---|
| No valid return yet | `WAITING` | 0 |
| d < `stop-distance` | `BACKING` | vx = −`back-speed` |
| `stop-distance` ≤ d ≤ `clear-distance` | `HOLD` | 0 |
| d > `clear-distance` | `CLEAR` | 0 |

Returns between 0.05 m and 12 m inside ±`cone-deg` of the LiDAR's 0° bearing are considered. The LiDAR 0° direction must point to the vehicle nose.

On exit (Ctrl+C or any error) the script stops Offboard and **commands PX4 to land**.

### 1.1 Limitations

- The RPLIDAR C1 measures a single horizontal plane. Obstacles above or below that plane, including a hand held at a different height, are not detected.
- Only the forward cone is monitored. The vehicle can back into an obstacle behind it.
- Only forward obstacles produce a response; there is no lateral or vertical avoidance.

---

## 2. Files

| File | Function |
|---|---|
| `lidar_front_test.py` | Reads the LiDAR and prints the minimum front distance. No flight-controller connection. |
| `avoidance_px4.py` | Connects to PX4 through MAVSDK and runs the back-off logic in Offboard mode. |

---

## 3. Installation

On the Raspberry Pi (Ubuntu 24.04):

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
python3 -m venv ~/px4_avoidance
source ~/px4_avoidance/bin/activate
pip install --upgrade pip
pip install mavsdk rplidar pyserial
```

If access to the serial ports is denied, add the user to the `dialout` group, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

---

## 4. Command-Line Arguments

| Argument | Default | Description |
|---|---|---|
| `--lidar-port` | `/dev/ttyUSB0` | LiDAR serial device |
| `--lidar-baudrate` | 115200 | LiDAR baud rate. **The RPLIDAR C1 uses 460800.** |
| `--mavsdk-address` | `serial:///dev/ttyACM0:921600` | PX4 connection |
| `--cone-deg` | 25.0 | Half-angle of the front cone, degrees |
| `--stop-distance` | 1.5 | Range below which the vehicle backs away, m |
| `--clear-distance` | 2.0 | Range above which the path is reported clear, m |
| `--back-speed` | 0.25 | Backward speed, m/s |
| `--control-hz` | 10.0 | Setpoint rate, Hz |

`lidar_front_test.py` accepts `--port`, `--baudrate` and `--cone-deg` with the same defaults.

Common PX4 connection addresses:

| Link | Address |
|---|---|
| USB | `serial:///dev/ttyACM0:921600` |
| GPIO UART (TELEM2) | `serial:///dev/ttyAMA0:921600` |

The TELEM2 UART is normally occupied by the uXRCE-DDS agent. Stop the agent, or use USB, before running this test.

---

## 5. Procedure

### 5.1 LiDAR check

```bash
source ~/px4_avoidance/bin/activate
python3 lidar_front_test.py --port /dev/ttyUSB0 --baudrate 460800
```

Move an object in front of the LiDAR. The printed distance must change accordingly. Do not proceed until this works.

### 5.2 Bench test (propellers removed)

```bash
python3 avoidance_px4.py --lidar-port /dev/ttyUSB0 --lidar-baudrate 460800 \
    --mavsdk-address serial:///dev/ttyACM0:921600
```

1. Wait for `PX4 connected`.
2. Press Enter to start Offboard.
3. Place an object inside the stop distance. Mode must change to `BACKING` with vx = −0.25 m/s.
4. Remove the object. Mode must change to `HOLD` and then `CLEAR`, with vx = 0.

### 5.3 Flight test

1. Install propellers. Confirm that a safety pilot can immediately take over with the RC transmitter.
2. Take off manually and establish a stable hover in an open area.
3. Run the script and press Enter only when the vehicle is stable.
4. Approach the front of the vehicle with a large, flat object at LiDAR height. The vehicle must move backward and stop once the object is beyond the stop distance.
5. Press Ctrl+C. The vehicle will land.

---

## 6. Tuning

Adjust in small steps from the defaults:

| Observation | Adjustment |
|---|---|
| Reacts too early | Reduce `--stop-distance` |
| Reacts too late | Increase `--stop-distance` |
| Backs away too aggressively | Reduce `--back-speed` |
| Responds to the airframe or propeller guards | Reduce `--cone-deg` or check LiDAR mounting |
