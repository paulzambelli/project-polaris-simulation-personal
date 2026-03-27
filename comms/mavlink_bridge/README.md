# mavlink_bridge

ROS 2 nodes that talk MAVLink to the vehicle (Pixhawk hardware or **ArduSub SITL**).

**Design plan (external nav / replacing MAVROS `vision_pose`):** see [docs/EXTERNAL_NAV_PLAN.md](docs/EXTERNAL_NAV_PLAN.md) — cites [ArduPilot Non-GPS position estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html), [MAVLink](https://mavlink.io/en/messages/common.html), and [MAVROS](https://github.com/mavlink/mavros/tree/ros2/mavros).

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

## Invalid `SCALED_PRESSURE` (~−180000 hPa) in ArduSub JSON SITL

`mavlink_bridge_publisher` forwards `press_abs` from MAVLink (hPa → Pa). If **even `SCALED_PRESSURE`** (primary baro) is a large negative value, ArduPilot’s **SITL barometer backend** is producing bad pressure from **`sitl_fdm.altitude`** (underwater model in `AP_Baro_SITL.cpp`), almost always because **Gazebo’s JSON pose/altitude** does not match what ArduPilot expects for the JSON link—not because the wrong `SCALED_PRESSURE2` slot was chosen.

**In-repo mitigations**

- **`mavlink_publisher`** (default on): `hydrostatic_pressure_fallback` publishes an approximate absolute pressure from **`/odom`** pose `z` (ENU: set `hydrostatic_surface_z_enu` to your free-surface height). High `variance` marks it as an estimate; **EKF/CRITICAL may persist** until the FDM chain is fixed.
- **`orca_bringup/cfg/sub.parm`**: optional `SIM_BARO_DISABLE 1` (disables simulated baro; side effects on health/EKF possible).

**Upstream fix**: align the Gazebo ↔ JSON interface with [SITL with JSON](https://ardupilot.org/dev/docs/sitl-with-JSON.html) / maintained `ardupilot_gazebo` plugins so NED position and home altitude stay consistent.
