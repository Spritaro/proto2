FROM ros:kinetic

# SHELL ["/bin/bash", "-C"]

RUN apt-get update && apt-get -y install \
    ros-kinetic-desktop \
    ros-kinetic-rgbd-launch

RUN apt-get update && apt-get -y install \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    cmake \
    udev \
    nano \
    kmod

RUN apt-get update && apt-get -y install \
    wget \
    zip \
    unzip

WORKDIR /root/
#RUN git clone https://github.com/IntelRealSense/librealsense.git
RUN wget https://github.com/intelRealSense/librealsense/archive/v2.18.0.zip

WORKDIR /root/
RUN unzip v2.18.0.zip && \
    cd librealsense-2.18.0 && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d/

WORKDIR /root/librealsense-2.18.0/
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

CMD udevadm control --reload-rules && udevadm trigger && \
    source /root/catkin_ws/devel/setup.bash && \
    catkin_make && \
    source /root/catkin_ws/devel/setup.bash
