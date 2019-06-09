#!/bin/bash

docker run \
    -it \
    --rm \
    --privileged \
    -v /home/pi/.ros/:/root/.ros/ \
    -v /home/pi/catkin_ws/:/root/catkin_ws/ \
    -p 9090:9090 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros-cpp-2.22.0-2.2.6 \
    bash
