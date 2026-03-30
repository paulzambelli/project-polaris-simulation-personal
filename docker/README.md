To build the docker image:
~~~
./build.sh
~~~

### Laptops without NVIDIA (Intel / AMD graphics)

Build and run the `orca4:laptop` image (no `nvidia-container-toolkit` and no `--gpus`):

~~~
./build_laptop.sh
./run_laptop.sh
~~~

Inside the container, a typical headless-friendly sim (no Gazebo GUI client) is:

~~~
ros2 launch orca_bringup sim_launch.py gzclient:=False
~~~

The container still passes `/dev/dri` and the `video` group so Mesa can use the integrated GPU when you do open a GUI (RViz or `gzclient:=True`). The main `Dockerfile` / `run.sh` path keeps NVIDIA environment variables and `--gpus all` for discrete NVIDIA GPUs.

`run.sh` starts the container in a `tmux` session named `orca` so you can split panes (e.g. `Ctrl-b %`, `Ctrl-b "`) and run several shells in one terminal. To skip tmux, run `docker run` with the same flags as `run.sh` but end the command with `/bin/bash` instead of the `tmux` line. The same applies to `run_laptop.sh` (container name `orca4-laptop`).

To launch Gazebo, RViz, all nodes:
~~~
./run.sh
ros2 launch orca_bringup sim_launch.py
~~~

To execute a mission:
~~~
docker exec -it orca4 /bin/bash
ros2 run orca_bringup mission_runner.py
~~~
