FROM ros:kinetic

ENV ROS_DISTRO kinetic
#ENV LIBREALSENSE_VERSION 2.21.0
ENV ROS_WRAPPER_VERSION 2.2.3

RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rgbd-launch

RUN apt-get update && apt-get -y install \
    build-essential \
    cmake \
    git \
    kmod \
    libglfw3-dev \
    libgtk-3-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    nano \
    pkg-config \
    python \
    python-dev \
    python-pip \
    python3 \
    python3-dev \
    python3-pip \
    udev \
    unzip \
    usbutils \
    wget \
    zip \
&& rm -rf /var/lib/apt/lists/*

# Install LibRealSense
WORKDIR /root/
#RUN git clone https://github.com/IntelRealSense/librealsense.git && 
#    cd librealsense && \
#    git checkout v${LIBREALSENSE_VERSION} && \
#    mkdir -p /etc/udev/rules.d && \
#    cp config/99-realsense-libusb.rules /etc/udev/rules.d
RUN git clone https://github.com/Spritaro/librealsense.git && \
    cd librealsense && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d
RUN cd /root/librealsense && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_PYTHON_BINDINGS=bool:true && \
    make -j4 && \
    make install

# Install ROS wrapper
WORKDIR /root/
RUN mkdir -p catkin_rs/src && \
    cd catkin_rs/src && \
    git clone https://github.com/intel-ros/realsense.git && \
    cd realsense && \
    git checkout ${ROS_WRAPPER_VERSION}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash ; \
    cd catkin_rs ; \
    catkin_make"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_rs/devel/setup.bash" >> ~/.bashrc

RUN pip install numpy

CMD udevadm control --reload-rules && udevadm trigger
