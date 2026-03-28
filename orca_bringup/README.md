## Launch files

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo.
Calls [bringup.py](launch/bringup.py).

To see parameters: `ros2 launch --show-args orca_bringup sim_launch.py`

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including ORB_SLAM2 and Nav2.
Calls [navigation_launch.py](launch/navigation_launch.py).

### [navigation_launch.py](launch/navigation_launch.py)

Nav2 navigation launch file, modified to avoid launch the velocity smoother.

## Scenarios

### Full automation

In a terminal run:
~~~
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
~~~

Execute a mission in a 2nd terminal:
~~~
source src/orca4/setup.bash
ros2 run orca_bringup mission_runner.py
~~~

### Using MAVProxy

It is possible to launch Gazebo and ArduSub and control the sub using MAVProxy.

Launch a minimal system:
~~~
ros2 launch orca_bringup sim_launch.py base:=false comms:=false nav:=false rviz:=false
~~~

Launch MAVProxy in a 2nd terminal:
~~~
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out udp:0.0.0.0:14550 --console
~~~

You can use MAVProxy to send commands directly to ArduSub:
~~~
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
disarm
~~~

RC channels:
* RC 3 -- vertical
* RC 4 -- yaw
* RC 5 -- forward
* RC 6 -- strafe

### MAVProxy + SLAM

This will bring up a minimal system with SLAM and RViz:
~~~
ros2 launch orca_bringup sim_launch.py base:=false comms:=false nav:=false
~~~

You can use MAVProxy to drive the sub around the seafloor and build a map.

## Tracking errors: rosbag → CSV (Docker vs host)

The Nav2 plugin `orca_nav2/PurePursuitController3D` can publish three diagnostics
(`std_msgs/msg/Float64`) while it is actively following a plan:

| Topic | Meaning |
|-------|---------|
| `/pure_pursuit_cross_track_xy` | Horizontal distance from the robot to the plan polyline (m) |
| `/pure_pursuit_vertical_error` | `robot_z - path_z` at the closest XY point (m) |
| `/pure_pursuit_yaw_error` | Shortest angle from path heading to robot yaw (rad, about `[-π, π]`) |

Enable/disable with the controller plugin parameter `publish_tracking_error` in
[`params/nav2_params.yaml`](params/nav2_params.yaml) (e.g. under `LongStraightLine`).

**Important:** Paths below use a typical **Orca4 Docker** layout: user `orca4`, workspace
`/home/orca4/colcon_ws`. If your image uses a different user or mount, replace those paths.

### What runs where

| Location | What you do there |
|----------|-------------------|
| **Inside Docker** | Build (`colcon build`), `source install/setup.bash`, launch sim, run missions, run `ros2 bag …` / `export_tracking_bag_csv.py`. Bags and CSVs are created **on the container filesystem** under your workspace (e.g. `~/colcon_ws`). |
| **On the Linux host** | `docker ps`, `docker cp` (and similar) to **copy** folders from the container to the host. The `docker` CLI is **not** available inside the container. |
| **Any machine (no ROS)** | Open the copied **CSV files** with pandas, R, Excel, etc. |

---

### A. Inside the Docker container (step by step)

1. **Open a shell in the container** (however you usually do: `docker exec -it orca4 bash`, VS Code Dev Container, etc.).

2. **Go to your ROS 2 workspace** (example):
   ```bash
   cd ~/colcon_ws
   ```

3. **Build packages that contain the controller and scripts** (after code changes):
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select orca_nav2 orca_bringup
   source install/setup.bash
   ```

4. **Start simulation with bag recording**  
   [`sim_launch.py`](launch/sim_launch.py) with `bag:=True` starts `ros2 bag record` for **only** the three topics above (no `/tf`, `/odom`, etc.):
   ```bash
   ros2 launch orca_bringup sim_launch.py bag:=True
   ```
   Leave this running in that terminal.

5. **Run a mission** (second terminal **inside the same container**, with workspace sourced):
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/colcon_ws/install/setup.bash
   ros2 run orca_bringup WSG84_mission_starter.py
   ```
   (Or your usual mission flow.) The topics are only published while Nav2’s controller is **active** and following a path.

6. **Stop cleanly**  
   In the launch terminal, press **Ctrl+C** once and wait. That stops `ros2 bag record` so the bag is **closed** properly (avoid `kill -9`).

7. **Find the bag directory**  
   The recorder creates a folder in the **current working directory** where you ran `ros2 launch` (usually `~/colcon_ws`):
   ```bash
   cd ~/colcon_ws
   ls -lt
   ```
   Look for `rosbag2_YYYY_MM_DD-HH_MM_SS`. It must contain `metadata.yaml` and a `.db3` (or `.mcap`) file.

8. **Export to CSV** (still **inside** the container):
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/colcon_ws/install/setup.bash
   ros2 run orca_bringup export_tracking_bag_csv.py ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS
   ```
   Optional: write outputs somewhere else:
   ```bash
   ros2 run orca_bringup export_tracking_bag_csv.py ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS \
     -o ~/colcon_ws/exports/my_run
   ```

9. **Exported files**  
   By default the script creates a folder **`csv_export`** next to the bag (or under `-o`):

   - `tracking_errors_long.csv` — columns `time_sec`, `topic`, `data` (all three topics in one file)
   - `tracking_errors_wide.csv` — columns `time_sec`, `cross_track_xy_m`, `vertical_error_m`, `yaw_error_rad`
   - `tracking_export_README.txt` — short column reference

   Optional plot (needs matplotlib): add `--plot` to the `export_tracking_bag_csv.py` command.

---

### B. On the Linux host (copy results out of Docker)

`docker cp` must be run on the **host**, not inside the container (`docker: command not found` inside the container is expected).

1. **On the host**, open a terminal (e.g. `polaris_pz@…`, **not** `orca4@…`).

2. **Confirm the container name**:
   ```bash
   docker ps
   ```
   Example: `NAMES` column shows `orca4`, or use the `CONTAINER ID`.

3. **Copy the `csv_export` folder** (adjust bag name to match yours):
   ```bash
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19/csv_export \
     ~/Downloads/orca_tracking_csv_export
   ```
   Or into your host project:
   ```bash
   mkdir -p ~/project-polaris-simulation-personal/analysis
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19/csv_export \
     ~/project-polaris-simulation-personal/analysis/run_180719
   ```

4. **Optional:** copy the **entire** bag if you want to keep the rosbag2 database on the host:
   ```bash
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19 \
     ~/Downloads/rosbag2_2026_03_28-18_07_19
   ```

5. **Bind mounts**  
   If you started the container with `-v /host/path:/home/orca4/colcon_ws`, the bag and `csv_export` may **already** appear under `/host/path` on the host—no `docker cp` needed.

---

### C. Analysis without ROS (host or other repo)

After you have `csv_export` on the host:

```bash
ls ~/Downloads/orca_tracking_csv_export
```

Example in Python:

```python
import pandas as pd
df = pd.read_csv("tracking_errors_wide.csv")
print(df.describe())
```

No ROS 2 installation is required on that machine.

---

### Troubleshooting

- **No `rosbag2_*` folder in `ls`**  
  You did not use `bag:=True`, or you launched from a **different** current directory—search:  
  `find ~ -maxdepth 3 -name metadata.yaml 2>/dev/null`

- **`ros2 bag info` shows 0 messages** on the three topics  
  Controller was not publishing (`publish_tracking_error: false`, or Nav2 not following a plan long enough).

- **`docker cp` fails**  
  Run it on the **host**; check container name, path inside container, and that the bag/export path exists (`docker exec orca4 ls /home/orca4/colcon_ws/...`).

- **ROS 1 `bagpy`**  
  Targets `.bag` files only; **not** ROS 2 rosbag2. Use `rosbag2_py` / `export_tracking_bag_csv.py` for folders with `metadata.yaml`.

## Video pipeline

The simulation uses Gazebo camera sensors to generate uncompressed 800x600 images in an
ideal stereo configuration. The frame rate is throttled to 5Hz to reduce CPU load in ORB_SLAM2, but
it can easily go higher.
