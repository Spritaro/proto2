#!/bin/bash

docker run \
    --net=host \
    -it \
    --rm \
    --privileged \
    -v /home/pi/.ros/:/root/.ros/ \
    -v /home/pi/catkin_ws/:/root/catkin_ws/ \
    -v "/usr/src/linux-headers-4.14.98-v7+":"/usr/src/linux-headers-4.14.98-v7+": \
    -v "/lib/modules/4.14.98-v7+":"/lib/modules/4.14.98-v7+" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    test-realsense \
    bash
