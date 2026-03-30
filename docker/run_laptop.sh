#!/usr/bin/env bash
# Intel / AMD laptops: no NVIDIA Container Toolkit or --gpus flag required.
# Gazebo server + physics run without a discrete GPU; use gzclient:=False to skip the GUI client.
# For RViz or gzclient:=True, keep X11 forwarding below and xhost +local:docker on the host if needed.

XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]
then
    xauth_list=$(xauth nlist "$DISPLAY")
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ -n "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    else
        touch "$XAUTH"
    fi
    chmod a+r "$XAUTH"
fi

docker run -it \
    --rm \
    --name orca4-laptop \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    --device=/dev/dri \
    --group-add video \
    --privileged \
    --security-opt seccomp=unconfined \
    orca4:laptop \
    tmux new-session -A -s orca
