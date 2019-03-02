FROM ros:kinetic

# SHELL ["/bin/bash", "-C"]

RUN apt-get update && apt-get -y install \
    ros-kinetic-desktop

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

RUN cd librealsense && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d/

CMD udevadm control --reload-rules && udevadm trigger

WORKDIR /root/librealsense/

RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4

WORKDIR /root/librealsense/build/

RUN make install

RUN apt-get update && apt-get -y install \
    ros-kinetic-rgbd-launch
