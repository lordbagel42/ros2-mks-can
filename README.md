# ros2-mks-can

ESP32 firmware that bridges **MKS SERVO57D** closed-loop stepper drivers to
**ROS 2** over CAN bus, using [micro-ROS](https://micro.ros.org/) for
minimum-latency communication.

Built on top of the
[MKSServoCAN](https://github.com/TheSpaceEgg/MKSServoCAN) Arduino library
(TWAI backend) and the
[micro\_ros\_platformio](https://github.com/micro-ROS/micro_ros_platformio)
integration.

---

## Features

| Area | Details |
|------|---------|
| **Motor control** | Absolute position, velocity, enable/disable, homing, e-stop |
| **Telemetry** | Encoder position, speed, and angle-error published as `sensor_msgs/JointState` at configurable rate (default 100 Hz) |
| **Transport** | Serial (USB, lowest latency) or WiFi — selectable at build time |
| **Latency** | Best-effort QoS, 921600 baud serial, zero-copy message buffers, all-on-one-core tight loop |
| **Multi-motor** | Up to 6 motors on the same CAN bus (configurable via `MAX_MOTORS`) |
| **Reconnection** | Automatic agent detection, connect, and reconnect state machine |

---

## Hardware Requirements

* **ESP32** dev board (ESP32-WROOM tested)
* **CAN transceiver** — Waveshare SN65HVD230 or similar 3.3 V module
* **MKS SERVO42D / SERVO57D** driver(s)
* USB cable (for serial transport) or WiFi network

### Wiring

```
ESP32             CAN Transceiver        MKS SERVO57D
──────            ───────────────        ────────────
GPIO 27 (TX) ──▶  TX                     ┐
GPIO 26 (RX) ◀──  RX           CAN_H ───┤ CAN_H
3V3          ──▶  VCC          CAN_L ───┤ CAN_L
GND          ──▶  GND                    ┘
```

> **Tip:** Add a 120 Ω termination resistor across CAN_H / CAN_L at each end
> of the bus.

---

## Software Prerequisites

| Tool | Version |
|------|---------|
| [PlatformIO CLI](https://platformio.org/install/cli) | ≥ 6.x |
| [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) | desktop or base |
| [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) | matching distro |

Install the micro-ROS agent (ROS 2 host):

```bash
sudo apt install ros-humble-micro-ros-agent   # Debian / Ubuntu
# — or build from source —
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
cd .. && colcon build
source install/setup.bash
```

---

## Build & Flash

```bash
cd firmware

# Serial transport (default, lowest latency)
pio run -e esp32

# WiFi transport (edit SSID / password / agent IP in platformio.ini first)
pio run -e esp32_wifi

# Flash
pio run -e esp32 --target upload
```

---

## Running

### 1. Start the micro-ROS agent on your ROS 2 host

```bash
# Serial transport
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 -b 921600

# WiFi transport (agent listens on UDP)
ros2 run micro_ros_agent micro_ros_agent udp4 \
    --port 8888
```

### 2. Power up the ESP32

The on-board LED indicates connection state:

| LED pattern | Meaning |
|-------------|---------|
| Slow blink (1 s) | Waiting for agent |
| Solid on | Connected |
| Fast blink (200 ms) | Agent lost — reconnecting |

### 3. Verify topics

```bash
ros2 topic list
# /joint_states
# /mks_servo/position_cmd
# /mks_servo/velocity_cmd
# /mks_servo/enable
# /mks_servo/emergency_stop

ros2 topic echo /joint_states
```

---

## ROS 2 Interface

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | 100 Hz | Position (rad), velocity (rad/s), effort (angle error rad) for every motor |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mks_servo/position_cmd` | `std_msgs/msg/Float64MultiArray` | Absolute position targets in **radians** — one element per motor (index order) |
| `/mks_servo/velocity_cmd` | `std_msgs/msg/Float64MultiArray` | Velocity targets in **rad/s** — positive = CW, negative = CCW; send 0 to stop |
| `/mks_servo/enable` | `std_msgs/msg/Bool` | `true` enables all motors, `false` disables |
| `/mks_servo/emergency_stop` | `std_msgs/msg/Empty` | Immediately stops all motors |

### Quick command examples

```bash
# Move motor 0 to π/2 rad, motor 1 to π rad
ros2 topic pub --once /mks_servo/position_cmd \
    std_msgs/msg/Float64MultiArray "{data: [1.5708, 3.1416]}"

# Spin motor 0 at 1 rad/s CW, motor 1 at −0.5 rad/s CCW
ros2 topic pub --once /mks_servo/velocity_cmd \
    std_msgs/msg/Float64MultiArray "{data: [1.0, -0.5]}"

# Enable motors
ros2 topic pub --once /mks_servo/enable std_msgs/msg/Bool "{data: true}"

# Emergency stop
ros2 topic pub --once /mks_servo/emergency_stop std_msgs/msg/Empty "{}"
```

---

## Configuration

All defaults live in `firmware/src/config.h` and can be overridden via
`build_flags` in `platformio.ini`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_MOTORS` | 6 | Maximum number of motors supported |
| `DEFAULT_NUM_MOTORS` | 2 | Number of motors initialised at boot |
| `CAN_TX_GPIO` | 27 | TWAI TX pin |
| `CAN_RX_GPIO` | 26 | TWAI RX pin |
| `TELEMETRY_HZ` | 100 | Telemetry publish rate |
| `UROS_BAUD` | 921600 | Serial baud rate |
| `ENCODER_CPR` | 16384 | Encoder counts per revolution (14-bit) |
| `USE_BEST_EFFORT_QOS` | 1 | 1 = best-effort, 0 = reliable |

To change motor CAN IDs, edit the `default_ids` array in
`firmware/src/main.cpp`.

---

## Latency Notes

The firmware is designed for minimum round-trip latency:

1. **Serial transport at 921600 baud** — keeps USB transfer time below 1 ms
   per message.
2. **Best-effort QoS** — no acknowledgement handshake on the RMW layer.
3. **Zero-copy static message buffers** — no `malloc` in the publish / subscribe
   hot path.
4. **Direct CAN frame parsing** — motor state is updated immediately on
   receive, bypassing the library's `Serial.print`-based `pollResponses()`.
5. **100 Hz default telemetry** — each motor's position, velocity, and angle
   error are requested every 10 ms.  Increase `TELEMETRY_HZ` (up to the CAN
   bus bandwidth limit) for even tighter loops.
6. **Agent-synchronised timestamps** — `rmw_uros_sync_session()` aligns the
   ESP32 clock to the ROS 2 graph so that `JointState` stamps are directly
   comparable to other sensor data.

---

## Project Structure

```
firmware/
├── platformio.ini          # PlatformIO build configuration
└── src/
    ├── config.h            # Pin, timing, and conversion constants
    ├── main.cpp            # Arduino setup() / loop()
    ├── motor_manager.h     # Motor state + CAN command / response layer
    ├── motor_manager.cpp
    ├── uros_interface.h    # micro-ROS node, pub/sub, timer
    └── uros_interface.cpp
```

---

## License

See [MKSServoCAN licence](https://github.com/TheSpaceEgg/MKSServoCAN#license)
(CC BY-NC 4.0) for the upstream library.  This firmware wrapper follows the
same terms unless stated otherwise.
