#!/bin/bash
xhost +local:

docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --privileged \
    --env="DISPLAY" \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --workdir="/home/ue4/colcon_ws" \
    --volume="$(pwd)/../colcon_ws:/home/ue4/colcon_ws" \
    --volume="$(pwd)/../work:/home/ue4/work" \
    unreal-engine-ros2-image:latest
