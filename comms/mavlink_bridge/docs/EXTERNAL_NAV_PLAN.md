# Plan: External navigation to ArduSub via the custom MAVLink bridge

This document replaces the role of **MAVROS `vision_pose`** (and related FCU-facing plugins) with **explicit MAVLink** from your bridge, aligned with **ArduPilot** and **MAVLink** specifications.

---

## 1. What MAVROS was doing (so you replicate the *interface*, not the package)

In upstream Orca4, `base_controller` publishes a `geometry_msgs/PoseStamped` on a topic that **MAVROS** consumes.

Your `sim_mavros_params.yaml` enables the **`vision_pose`** plugin:

```yaml
plugin_allowlist:
  ...
  - vision_pose
```

The **MAVROS extras** **`vision_pose` / `vision_pose_estimate`** plugin subscribes to vision pose (e.g. `PoseStamped` / `PoseWithCovarianceStamped` or TF) and sends a **MAVLink vision estimate** to the FCU (classically **`VISION_POSITION_ESTIMATE`**; see plugin lineage in [mavlink/mavros#1605](https://github.com/mavlink/mavros/pull/1605)).

**Official MAVROS entry points:**

- ROS 2 package layout and plugins: [mavlink/mavros (ROS 2 branch)](https://github.com/mavlink/mavros/tree/ros2/mavros)  
- Connection URL schemes (same bridge will use `pymavlink`): [mavros `fcu_url` documentation in repo](https://github.com/mavlink/mavros/tree/ros2/mavros) (see parameter docs there)

So yes: **without MAVROS, you must send the equivalent MAVLink yourself** (or reuse a small library that already implements the same message packing as MAVROS).

---

## 2. What ArduPilot expects (authoritative)

### 2.1 Allowed messages and rate

ArduPilot’s developer documentation **“Non-GPS Position Estimation”** states that **any** of the listed MAVLink messages may be used, and they should be sent at **≥ 4 Hz**:

**Primary reference:**  
[Non-GPS Position Estimation — ArduPilot Dev](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)

That page lists (among others):

| Approach | MAVLink message(s) | Notes from ArduPilot |
|----------|-------------------|----------------------|
| **Preferred** | [`ODOMETRY`](https://mavlink.io/en/messages/common.html#ODOMETRY) | Preferred method; `frame_id` / `child_frame_id` per doc |
| Common | [`VISION_POSITION_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) + optional [`VISION_SPEED_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE) | Widely used with vision systems |
| Others | `VISION_POSITION_DELTA`, `VICON_POSITION_ESTIMATE`, `ATT_POS_MOCAP`, … | See same ArduPilot page |

**MAVLink message definitions (official):**  
[MAVLink `common` dialect](https://mavlink.io/en/messages/common.html)

### 2.2 `ODOMETRY` fields (if you choose the preferred path)

ArduPilot documents required semantics in the same page, including:

- `frame_id`: **`MAV_FRAME_LOCAL_FRD` (20)** or **`MAV_FRAME_BODY_FRD` (12)**  
- `child_frame_id`: same allowed set  
- Position **z positive down** (NED-style vertical)  
- Quaternion `q` w,x,y,z; linear velocity; angular rates; covariance slots; `reset_counter`; `quality` vs **`VISO_QUAL_MIN`**

Full field table:  
[Non-GPS Position Estimation — ODOMETRY message section](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html#odometry-message)

### 2.3 EKF origin when GPS is disabled

ArduPilot explicitly states:

> If no GPS is attached to the autopilot then the **EKF origin must be set** before the EKF can start estimating its position.

**Reference:**  
[Non-GPS Position Estimation — ArduPilot Dev](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html) (note in opening section)

How to set it via MAVLink:

**[Setting Home and/or EKF origin — ArduPilot Dev](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html)**

Relevant messages/commands there include:

- [`SET_GPS_GLOBAL_ORIGIN`](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN) for the **EKF origin**  
- Home via [`MAV_CMD_DO_SET_HOME`](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME) in `COMMAND_INT` / `COMMAND_LONG`

Your SITL `--home` and params interact with this; if the EKF never gets an origin, external nav fusion can fail regardless of streaming poses.

### 2.4 Parameters (must match what you actually send)

ArduPilot’s non-GPS page gives a **template** using **VOXL-style** `VISO_*` and **EK3_SRC1_*** settings (ExternalNav = 6, baro for Z, etc.).  

**ArduSub parameter reference (official):**  
[ArduSub parameters](https://ardupilot.org/sub/docs/parameters.html) — search `VISO_TYPE`, `EK3_SRC1_POSXY`, `VISO_QUAL_MIN`, etc.

Your `orca_bringup/cfg/sub.parm` already uses **GPS off + external nav + vision type**. Before implementation, **diff** your parm set against the recommendations on the [Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html) page (e.g. `VISO_TYPE` value and `VISO_POS_*` lever arms for sensor offset on the vehicle).

---

## 3. ROS / Gazebo side vs MAVLink side (correctness)

| Layer | Frame / data | Role |
|-------|----------------|------|
| **Nav2 / TF** | Typically **`map`** + **`odom`** + **`base_link`**, often **ENU** conventions | Planning and control in ROS |
| **MAVLink → ArduPilot** | **`ODOMETRY`**: **local FRD** as per ArduPilot table; **z down** | EKF external navigation |

So you **do** use Gazebo odometry (or SLAM later) as the **source of truth in ROS**, but you must **not** memcpy ROS poses directly into MAVLink without a **documented frame transform**:

1. Subscribe to **`nav_msgs/Odometry`** on `/odom` (or use TF `map`→`base_link` / `odom`→`base_link` — pick one chain and stick to it).  
2. Convert **position + orientation + optionally velocity** from your ROS convention to the **MAVLink frame** required by the message you chose (`LOCAL_FRD` for `ODOMETRY` per ArduPilot doc).  
3. Use **consistent timestamps** (`use_sim_time` in ROS vs `time_usec` in MAVLink — ArduPilot notes boot time vs epoch for some fields; follow the **ODOMETRY** / **VISION_POSITION_ESTIMATE** field descriptions on [mavlink.io](https://mavlink.io/en/messages/common.html)).

**Quality / covariance:** ArduPilot applies **`VISO_QUAL_MIN`** and covariance handling as described in [Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html). For ground truth from simulation you can use high quality and reasonable diagonal covariances; tune for real sensors later.

---

## 4. Where this lives in *your* bridge architecture

| Component | Suggested responsibility |
|-----------|---------------------------|
| **`mavlink_publisher`** | Keep as **FCU → ROS** telemetry (already uses one SITL TCP port). |
| **`ros2_receiver`** | Keep as **ROS → FCU** commands (mode, arm, `cmd_vel` → `SET_POSITION_TARGET_LOCAL_NED`, etc.). |
| **New node (recommended)** e.g. `vision_odom_bridge` | Subscribe `/odom` (or TF), send **`ODOMETRY`** or **`VISION_POSITION_ESTIMATE`** at **≥ 4 Hz** on the **same MAVLink link** that ArduPilot accepts for those messages — typically the **second** SITL serial (e.g. TCP 5762) *or* share one connection (hardware: usually **one** serial → **one** `mavutil` connection multiplexing all outbound messages). |

**Important:** On **SITL**, you already split **two TCP clients** across two ports; the new stream must go out on **a link the FCU is listening on** without breaking the existing command client. Options:

- Send odometry from the **same process** as `ros2_receiver` (single TCP client, single writer) — cleanest on hardware.  
- Or add a third SITL UART via ArduPilot’s serial options (see [UARTs and the Console](https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html)) — only if you really need separate processes.

---

## 5. Implementation phases (recommended order)

1. **Parameters** — Align `sub.parm` with [Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html) + [ArduSub parameters](https://ardupilot.org/sub/docs/parameters.html) for your chosen message type.  
2. **EKF origin** — Verify in SITL/logs that origin/home behavior matches [mavlink-get-set-home-and-origin](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html) expectations when `GPS_TYPE = 0`.  
3. **Message choice** — Implement **`ODOMETRY`** first (ArduPilot’s stated preference) with `frame_id = MAV_FRAME_LOCAL_FRD` per their table; fallback to **`VISION_POSITION_ESTIMATE`** if you need parity with older MAVROS-style paths.  
4. **Transforms** — Unit-test ENU→FRD/NED conversion against known poses (document test vectors).  
5. **Integration** — Run SITL + Gazebo; confirm EKF uses external nav (Mission Planner / QGC EKF status, or ArduPilot logs).  
6. **Hardware** — Same node; change only `MAVLINK_*_URL` to serial; confirm one writer on the UART.

---

## 6. Quick reference link list

| Topic | URL |
|-------|-----|
| ArduPilot non-GPS / external nav | https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html |
| EKF origin / home | https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html |
| MAVLink `ODOMETRY` | https://mavlink.io/en/messages/common.html#ODOMETRY |
| MAVLink `VISION_POSITION_ESTIMATE` | https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE |
| MAVLink `VISION_SPEED_ESTIMATE` | https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE |
| ArduSub parameters | https://ardupilot.org/sub/docs/parameters.html |
| SITL UART layout | https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html |
| MAVROS (ROS 2) | https://github.com/mavlink/mavros/tree/ros2/mavros |

---

*This file is a design plan only; implementation should follow the linked specifications and your vehicle’s safety procedures.*
