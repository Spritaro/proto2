#!/bin/bash

docker run \
    -it \
    --rm \
    --privileged \
    --net=host \
    -v /home/pi/.ros/:/root/.ros/ \
    -v /home/pi/catkin_ws/:/root/catkin_ws/ \
    -p 9090:9090 \
    --env ROS_IP=192.168.13.100 \
    --env ROS_MASTER_URI=http://192.168.13.100:11311 \
    ros-cpp-tf \
    bash
