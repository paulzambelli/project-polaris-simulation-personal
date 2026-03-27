To build the docker image:
~~~
./build.sh
~~~

`run.sh` starts the container in a `tmux` session named `orca` so you can split panes (e.g. `Ctrl-b %`, `Ctrl-b "`) and run several shells in one terminal. To skip tmux, run `docker run` with the same flags as `run.sh` but end the command with `/bin/bash` instead of the `tmux` line.

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
