#!/bin/bash

docker run \
    -it \
    --rm \
    --privileged \
    -v /home/pi/.ros/:/root/.ros/ \
    -v /home/catkin_ws/:/root/catkin_ws/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    test-realsense \
    bash
