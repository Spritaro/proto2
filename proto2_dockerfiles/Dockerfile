FROM ros:kinetic

ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rgbd-launch

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
    kmod \
    wget \
    zip \
    unzip \
    usbutils

ENV LIBREALSENSE_VERSION 2.18.0
WORKDIR /root/
RUN wget https://github.com/intelRealSense/librealsense/archive/v${LIBREALSENSE_VERSION}.zip
RUN unzip v${LIBREALSENSE_VERSION}.zip && \
    rm v${LIBREALSENSE_VERSION}.zip && \
    cd librealsense-${LIBREALSENSE_VERSION} && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d
WORKDIR /root/librealsense-${LIBREALSENSE_VERSION}
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

CMD udevadm control --reload-rules && udevadm trigger && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash ; \
                  catkin_make ; \
                  source /root/catkin_ws/devel/setup.bash ;"
