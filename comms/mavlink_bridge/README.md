# mavlink_bridge

ROS 2 nodes that talk MAVLink to the vehicle (Pixhawk hardware or **ArduSub SITL**).

## ArduSub SITL and TCP ports

ArduPilot SITL exposes UARTs as TCP ports. Defaults are documented here:

[UARTs and the Console (ArduPilot Dev)](https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html)

Typical mapping:

| SITL UART   | Default TCP | Role (typical)   |
|------------|-------------|------------------|
| `SERIAL0`  | **5760**    | First link       |
| `SERIAL1`  | **5762**    | Second MAVLink   |

This package uses **two connections** (telemetry publisher + command receiver). **Both must not attach to the same SITL TCP port** — each serial port usually accepts one TCP client. Defaults:

- `mavlink_publisher` → `MAVLINK_PUBLISHER_URL` default `tcp:127.0.0.1:5760`
- `ros2_receiver` → `MAVLINK_RECEIVER_URL` default `tcp:127.0.0.1:5762`

`orca_bringup/cfg/sub.parm` sets `SERIAL1_PROTOCOL 2` so SERIAL1 speaks MAVLink2 on 5762.

Override at runtime if needed:

```bash
export MAVLINK_PUBLISHER_URL=tcp:127.0.0.1:5760
export MAVLINK_RECEIVER_URL=tcp:127.0.0.1:5762
```

## Real hardware

Use a serial device and baud (baud is used for serial; TCP ignores it), for example:

```bash
export MAVLINK_PUBLISHER_URL=/dev/ttyUSB0
export MAVLINK_PUBLISHER_BAUD=57600
export MAVLINK_RECEIVER_URL=/dev/ttyUSB0
export MAVLINK_RECEIVER_BAUD=57600
```

If you only have **one** physical UART, run a **single** MAVLink client or multiplex via MAVProxy; two nodes opening the same serial port will conflict.
