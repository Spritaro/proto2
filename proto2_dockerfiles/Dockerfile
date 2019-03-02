FROM ros:kinetic

# SHELL ["/bin/bash", "-C"]

RUN apt-get update && apt-get -y install \
    ros-kinetic-desktop
    ros-kinetic-rgbd-launch

RUN apt-get update && apt-get -y install \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    cmake \
    udev

WORKDIR /root/
RUN git clone https://github.com/IntelRealSense/librealsense.git

WORKDIR /root/
RUN cd librealsense && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d/

WORKDIR /root/librealsense/
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

CMD udevadm control --reload-rules && udevadm trigger

