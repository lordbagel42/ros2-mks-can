# ros2-mks-can

ESP32 firmware that bridges **MKS SERVO42D/57D** closed-loop stepper drivers to
**ROS 2** over CAN bus using [micro-ROS](https://micro.ros.org/).

This project lets you control MKS servo motors from any ROS 2 node — publish a
position or velocity command, and the ESP32 forwards it over CAN. Motor
telemetry (position, speed, angle error) streams back to ROS 2 at 100 Hz as a
standard `sensor_msgs/JointState` message.

Built on the
[MKSServoCAN](https://github.com/TheSpaceEgg/MKSServoCAN) Arduino library
(ESP32 TWAI backend) and the
[micro\_ros\_platformio](https://github.com/micro-ROS/micro_ros_platformio)
integration.

---

## Table of Contents

1. [How It Works](#how-it-works)
2. [What You Need](#what-you-need)
3. [Step 1 — Prepare the MKS Motors](#step-1--prepare-the-mks-motors)
4. [Step 2 — Wire the Hardware](#step-2--wire-the-hardware)
5. [Step 3 — Install Software Prerequisites](#step-3--install-software-prerequisites)
6. [Step 4 — Configure the Firmware](#step-4--configure-the-firmware)
7. [Step 5 — Build & Flash](#step-5--build--flash)
8. [Step 6 — Connect to ROS 2](#step-6--connect-to-ros-2)
9. [Using the Motors from ROS 2](#using-the-motors-from-ros-2)
10. [Writing a ROS 2 Node (Python Example)](#writing-a-ros-2-node-python-example)
11. [Configuration Reference](#configuration-reference)
12. [Latency Notes](#latency-notes)
13. [Project Structure](#project-structure)
14. [Troubleshooting](#troubleshooting)
15. [License](#license)

---

## How It Works

```
┌────────────┐  USB/Serial or WiFi  ┌───────────┐   CAN bus   ┌──────────────┐
│  ROS 2     │◄─────────────────────►│   ESP32   │◄───────────►│ MKS SERVO57D │
│  Host PC   │   micro-ROS agent     │ firmware  │   TWAI      │ (motor 1..N) │
└────────────┘                       └───────────┘             └──────────────┘
```

1. The **ESP32** runs this firmware and connects to a **micro-ROS agent** on
   your ROS 2 host (over USB serial or WiFi).
2. The firmware creates a ROS 2 node (`/mks_servo/mks_servo_node`) that
   publishes motor telemetry and subscribes to motion commands.
3. Motor commands received from ROS 2 are forwarded as CAN frames to the
   **MKS SERVO42D/57D** drivers.
4. The firmware polls motor telemetry (encoder, speed, angle error) over CAN
   and publishes it as `/joint_states`.

---

## What You Need

### Hardware

| Component | Notes |
|-----------|-------|
| **ESP32 dev board** | ESP32-WROOM or DevKitC tested |
| **CAN transceiver** | SN65HVD230 (Waveshare), TJA1050, or similar 3.3 V module |
| **MKS SERVO42D or SERVO57D** | One or more, with CAN connectors |
| **120 Ω resistors** (×2) | Bus termination — one at each end of the CAN bus |
| **USB cable** | Micro-USB / USB-C for flashing and serial transport |
| **Power supply** | For the MKS drivers (see MKS datasheet for voltage requirements) |

### Software (on your host PC)

| Tool | Version | Install |
|------|---------|---------|
| [PlatformIO CLI](https://platformio.org/install/cli) | ≥ 6.x | `pip install platformio` |
| [ROS 2](https://docs.ros.org/en/humble/Installation.html) | Humble (or later) | See ROS 2 docs |
| [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) | Humble | See [Step 3](#step-3--install-software-prerequisites) |

---

## Step 1 — Prepare the MKS Motors

Before connecting to the ESP32, configure each MKS motor driver using its
built-in OLED menu or the MKS PC tool:

1. **Set a unique CAN ID** for each motor (1, 2, 3, …).
   Navigate to the motor's menu → *CAN ID* and assign a number.
   The firmware defaults expect motor IDs `1` and `2`.

2. **Set the CAN baud rate to 500 kbps** (this matches the ESP32 TWAI
   default).  Navigate to *CAN Rate* → select `500K`.

3. **Set the work mode** to your desired mode:
   - `CR_CLOSE` (closed-loop pulse) or `SR_CLOSE` (closed-loop serial) are
     typical for CAN control.

4. **Calibrate the encoder** if this is a new motor.  Navigate to *Calibrate*
   and follow the on-screen instructions.  The shaft will rotate during
   calibration.

5. **Note down each motor's CAN ID** — you will need these when configuring
   the firmware.

> **Tip:** See the
> [MKS SERVO57D documentation](https://github.com/makerbase-motor/MKS-SERVO57D)
> for full menu descriptions.

---

## Step 2 — Wire the Hardware

Connect the ESP32 to the CAN transceiver, and the transceiver to the MKS
motor(s):

```
ESP32                CAN Transceiver         MKS SERVO57D
────────             ───────────────         ────────────
GPIO 27 (TX)  ──▶    TX
GPIO 26 (RX)  ◀──    RX
3V3           ──▶    VCC
GND           ──▶    GND
                     CAN_H  ────────────────  CAN_H
                     CAN_L  ────────────────  CAN_L
```

**Important wiring notes:**

- Add a **120 Ω termination resistor** across CAN_H and CAN_L at each end of
  the bus (one at the transceiver, one at the last motor).
- If you have multiple motors, daisy-chain them on the same CAN bus
  (CAN_H → CAN_H, CAN_L → CAN_L).
- Keep CAN wires short and twisted together to reduce noise.
- **Do not** connect 5 V to a 3.3 V-only transceiver module.

---

## Step 3 — Install Software Prerequisites

### 3a. Install PlatformIO

```bash
pip install platformio
```

Verify:

```bash
pio --version
```

### 3b. Install ROS 2 Humble

Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html)
for your OS.  After installation, make sure your environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

### 3c. Install the micro-ROS Agent

The agent runs on your host PC and bridges between the ESP32's serial/WiFi
link and the ROS 2 network.

**Option A — Install from apt (easiest):**

```bash
sudo apt install ros-humble-micro-ros-agent
```

**Option B — Build from source:**

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/microros_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

---

## Step 4 — Configure the Firmware

### Clone this repo

```bash
git clone https://github.com/lordbagel42/ros2-mks-can.git
cd ros2-mks-can
```

### Set motor CAN IDs

Edit `firmware/src/main.cpp` and change the `default_ids` array to match the
CAN IDs you assigned to your motors in [Step 1](#step-1--prepare-the-mks-motors):

```cpp
// Change these to match your motors' CAN IDs
static const uint32_t default_ids[DEFAULT_NUM_MOTORS] = {1, 2};
```

If you have a different number of motors, also update `DEFAULT_NUM_MOTORS` in
`firmware/platformio.ini`:

```ini
build_flags =
    -D DEFAULT_NUM_MOTORS=3   ; ← change to your motor count
    ...
```

### Set CAN bus pins (optional)

If your ESP32 wiring uses different GPIOs, update the pin defines in
`firmware/platformio.ini`:

```ini
build_flags =
    -D CAN_TX_GPIO=27   ; ← your TX pin
    -D CAN_RX_GPIO=26   ; ← your RX pin
    ...
```

### WiFi transport (optional)

If you want to use WiFi instead of USB serial, edit the `[env:esp32_wifi]`
section in `firmware/platformio.ini`:

```ini
build_flags =
    ...
    -D WIFI_SSID=\"your_wifi_name\"
    -D WIFI_PASS=\"your_wifi_password\"
    -D AGENT_IP=\"192.168.1.100\"    ; ← IP of your ROS 2 host
    -D AGENT_PORT=8888
```

---

## Step 5 — Build & Flash

### Build

```bash
cd firmware

# Serial transport (default — lowest latency)
pio run -e esp32

# WiFi transport
pio run -e esp32_wifi
```

### Flash to the ESP32

Plug in the ESP32 via USB, then:

```bash
# Serial variant
pio run -e esp32 --target upload

# WiFi variant
pio run -e esp32_wifi --target upload
```

> **Note:** If the upload fails, hold the **BOOT** button on the ESP32 during
> upload, or check that the correct serial port is detected (`pio device list`).

### Monitor serial output (optional, for debugging)

```bash
pio device monitor -b 921600
```

---

## Step 6 — Connect to ROS 2

### 6a. Start the micro-ROS agent

Open a terminal on your ROS 2 host machine:

```bash
source /opt/ros/humble/setup.bash

# ── Serial transport ────────────────────────────────────────────────────
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 -b 921600

# ── WiFi transport (if you built the WiFi variant) ─────────────────────
ros2 run micro_ros_agent micro_ros_agent udp4 \
    --port 8888
```

> **Tip:** If your serial device is not `/dev/ttyUSB0`, check with
> `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`.

### 6b. Check the ESP32 LED status

Once the agent is running and the ESP32 is powered:

| LED Pattern | Meaning |
|-------------|---------|
| Slow blink (1 s) | Waiting for agent — agent not yet detected |
| **Solid on** | **Connected** — node is live on the ROS 2 network |
| Fast blink (200 ms) | Agent lost — attempting to reconnect |

When the LED goes **solid**, the ESP32 is connected.

### 6c. Verify the connection

Open a **second terminal** (source ROS 2 first):

```bash
source /opt/ros/humble/setup.bash

# List all topics — you should see the MKS servo topics
ros2 topic list
```

Expected output:

```
/joint_states
/mks_servo/emergency_stop
/mks_servo/enable
/mks_servo/position_cmd
/mks_servo/velocity_cmd
```

Confirm telemetry is flowing:

```bash
ros2 topic echo /joint_states
```

You should see `JointState` messages at 100 Hz with `position`, `velocity`,
and `effort` arrays (one entry per motor).

---

## Using the Motors from ROS 2

Once connected, you control the motors by publishing to the subscribed topics.
All commands use **SI units** (radians and radians/second).

### Enable the motors

Motors are disabled by default after power-on. You must enable them before
sending motion commands:

```bash
ros2 topic pub --once /mks_servo/enable \
    std_msgs/msg/Bool "{data: true}"
```

To disable:

```bash
ros2 topic pub --once /mks_servo/enable \
    std_msgs/msg/Bool "{data: false}"
```

### Move to an absolute position

Send an array of target positions in **radians**, one per motor (in index
order matching your CAN IDs):

```bash
# Motor 0 → 90° (π/2 rad), Motor 1 → 180° (π rad)
ros2 topic pub --once /mks_servo/position_cmd \
    std_msgs/msg/Float64MultiArray "{data: [1.5708, 3.1416]}"

# Motor 0 → 0° (home), Motor 1 → 360° (2π rad)
ros2 topic pub --once /mks_servo/position_cmd \
    std_msgs/msg/Float64MultiArray "{data: [0.0, 6.2832]}"
```

### Spin at a constant velocity

Send an array of velocities in **rad/s**. Positive = clockwise,
negative = counter-clockwise. Send `0.0` to stop:

```bash
# Motor 0 at 1 rad/s CW, Motor 1 at 0.5 rad/s CCW
ros2 topic pub --once /mks_servo/velocity_cmd \
    std_msgs/msg/Float64MultiArray "{data: [1.0, -0.5]}"

# Stop all motors
ros2 topic pub --once /mks_servo/velocity_cmd \
    std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

### Emergency stop

Immediately halts all motors:

```bash
ros2 topic pub --once /mks_servo/emergency_stop \
    std_msgs/msg/Empty "{}"
```

### Monitor telemetry

```bash
# Stream all motor states
ros2 topic echo /joint_states

# Check publish rate
ros2 topic hz /joint_states
```

The `JointState` message fields:

| Field | Unit | Description |
|-------|------|-------------|
| `position[]` | rad | Current shaft position |
| `velocity[]` | rad/s | Current shaft speed |
| `effort[]` | rad | Angle error (difference between target and actual position) |
| `name[]` | — | Motor names (`motor_0`, `motor_1`, …) |

---

## Writing a ROS 2 Node (Python Example)

Below is a minimal Python node that enables the motors, sends a position
command, and prints the current joint state. Save it as
`mks_example_node.py` and run with `python3 mks_example_node.py`:

```python
#!/usr/bin/env python3
"""Minimal example: command MKS servos and read telemetry via ROS 2."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool

import math


class MKSServoExample(Node):
    def __init__(self):
        super().__init__('mks_servo_example')

        # Use best-effort QoS to match the micro-ROS firmware
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers (commands to the motors)
        self.enable_pub = self.create_publisher(Bool, '/mks_servo/enable', qos)
        self.pos_pub = self.create_publisher(
            Float64MultiArray, '/mks_servo/position_cmd', qos
        )
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/mks_servo/velocity_cmd', qos
        )

        # Subscriber (telemetry from the motors)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, qos)

        # Enable motors after a short delay
        self.enable_timer = self.create_timer(1.0, self.enable_motors)

        # Send a position command 3 seconds after startup
        self.position_timer = self.create_timer(3.0, self.send_position)

    def enable_motors(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        self.get_logger().info('Motors enabled')
        # Only fire once
        self.destroy_timer(self.enable_timer)

    def send_position(self):
        msg = Float64MultiArray()
        msg.data = [math.pi / 2, math.pi]  # motor_0 → 90°, motor_1 → 180°
        self.pos_pub.publish(msg)
        self.get_logger().info(f'Position command sent: {msg.data}')
        self.destroy_timer(self.position_timer)

    def on_joint_state(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self.get_logger().info(
                f'{name}: pos={msg.position[i]:.3f} rad, '
                f'vel={msg.velocity[i]:.3f} rad/s, '
                f'effort={msg.effort[i]:.4f} rad'
            )


def main():
    rclpy.init()
    node = MKSServoExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> **Note:** The micro-ROS firmware uses **best-effort QoS** by default. Your
> ROS 2 nodes must use the same QoS profile or messages will not be received.

---

## ROS 2 Interface Reference

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/msg/JointState` | 100 Hz | Position (rad), velocity (rad/s), effort (angle error rad) per motor |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mks_servo/position_cmd` | `std_msgs/msg/Float64MultiArray` | Absolute position targets in **radians** — one element per motor |
| `/mks_servo/velocity_cmd` | `std_msgs/msg/Float64MultiArray` | Velocity targets in **rad/s** — positive = CW, negative = CCW; 0 = stop |
| `/mks_servo/enable` | `std_msgs/msg/Bool` | `true` enables all motors, `false` disables |
| `/mks_servo/emergency_stop` | `std_msgs/msg/Empty` | Immediately stops all motors |

### Node Info

| Property | Value |
|----------|-------|
| Node name | `mks_servo_node` |
| Namespace | `mks_servo` |
| Frame ID | `base_link` |

---

## Configuration Reference

All defaults are defined in `firmware/src/config.h` and can be overridden via
`build_flags` in `firmware/platformio.ini`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_MOTORS` | 6 | Maximum motors the firmware can manage |
| `DEFAULT_NUM_MOTORS` | 2 | Number of motors initialised at boot |
| `CAN_TX_GPIO` | 27 | ESP32 pin for TWAI TX |
| `CAN_RX_GPIO` | 26 | ESP32 pin for TWAI RX |
| `TELEMETRY_HZ` | 100 | JointState publish rate (Hz) |
| `UROS_BAUD` | 921600 | Serial baud rate for micro-ROS transport |
| `ENCODER_CPR` | 16384 | Encoder counts per revolution (14-bit) |
| `USE_BEST_EFFORT_QOS` | 1 | 1 = best-effort QoS, 0 = reliable QoS |

### Changing motor CAN IDs

Edit `firmware/src/main.cpp`:

```cpp
static const uint32_t default_ids[DEFAULT_NUM_MOTORS] = {1, 2};
```

### Adjusting the telemetry rate

In `firmware/platformio.ini`, change the `TELEMETRY_HZ` flag. Higher rates
give fresher data but use more CAN bus bandwidth:

```ini
build_flags =
    -D TELEMETRY_HZ=200   ; 200 Hz telemetry
    ...
```

---

## Latency Notes

This firmware is designed for the lowest possible round-trip latency between
ROS 2 and the motors:

1. **Serial transport at 921600 baud** — USB transfer time stays below 1 ms
   per message.
2. **Best-effort QoS** — no acknowledgement handshake on the RMW layer.
3. **Zero-copy static message buffers** — no `malloc` in the publish/subscribe
   hot path.
4. **Direct CAN frame parsing** — motor state is updated immediately on
   receive, bypassing the library's `Serial.print`-based `pollResponses()`.
5. **100 Hz default telemetry** — position, velocity, and angle error are
   requested every 10 ms.  Increase `TELEMETRY_HZ` for tighter loops.
6. **Agent-synchronised timestamps** — `rmw_uros_sync_session()` aligns the
   ESP32 clock with the ROS 2 graph so `JointState` stamps are directly
   comparable to other sensor data.

---

## Project Structure

```
ros2-mks-can/
├── README.md
├── .gitignore
└── firmware/
    ├── platformio.ini          # PlatformIO build configuration
    └── src/
        ├── config.h            # Pin, timing, and conversion constants
        ├── main.cpp            # Arduino setup() / loop()
        ├── motor_manager.h     # Motor state tracking + CAN commands
        ├── motor_manager.cpp   # CAN frame parsing and motion commands
        ├── uros_interface.h    # micro-ROS node, pub/sub, timer
        └── uros_interface.cpp  # Agent state machine and callbacks
```

---

## Troubleshooting

### ESP32 LED blinks slowly, agent is running

- Check the **USB cable** — some cables are charge-only and lack data lines.
- Verify the correct serial port: `pio device list` or `ls /dev/ttyUSB*`.
- Make sure the baud rate matches: the agent must use `-b 921600`.

### `ros2 topic list` shows no topics

- Confirm the micro-ROS agent printed `[info] Created client` in its terminal.
- If using WiFi, verify the agent IP and port match `platformio.ini`.
- Try restarting the ESP32 (press the **EN** / reset button).

### Motors do not move

- Make sure you **enabled** the motors first:
  `ros2 topic pub --once /mks_servo/enable std_msgs/msg/Bool "{data: true}"`
- Check that the motor CAN IDs in `main.cpp` match the IDs configured on the
  motors themselves.
- Verify the CAN bus wiring and 120 Ω termination resistors.
- Check motor error state: the effort field in `/joint_states` shows the angle
  error — large values may indicate a stall or obstruction.

### `joint_states` shows all zeros

- The firmware needs at least one full telemetry cycle to populate data.
  Wait 1–2 seconds after connection.
- Verify CAN wiring — if the transceiver is not connected, CAN reads will
  time out and return zero.

### QoS mismatch — subscriber receives nothing

The firmware uses **best-effort QoS** by default.  Your ROS 2 subscribers
must also use best-effort:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
```

Or from the command line, add `--qos-reliability best_effort`:

```bash
ros2 topic echo /joint_states --qos-reliability best_effort
```

### Build fails with micro-ROS errors

- Make sure PlatformIO is up to date: `pio upgrade`
- Clean the build: `cd firmware && pio run -e esp32 --target clean`
- Delete `.pio/` and rebuild: `rm -rf .pio && pio run -e esp32`

---

## License

See the
[MKSServoCAN licence](https://github.com/TheSpaceEgg/MKSServoCAN#license)
(CC BY-NC 4.0) for the upstream library.  This firmware wrapper follows the
same terms unless stated otherwise.
