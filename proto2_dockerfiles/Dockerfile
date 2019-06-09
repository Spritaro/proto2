FROM ros:kinetic

ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-tf

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
    sudo \
    udev \
    unzip \
    usbutils \
    wget \
    zip \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*

# Install LibRealSense
ENV LIBREALSENSE_VERSION 2.22.0
WORKDIR /root
COPY patchfile /root
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
   cd librealsense && \
   git checkout v${LIBREALSENSE_VERSION} && \
   patch -p0 < ../patchfile && \
   mkdir -p /etc/udev/rules.d && \
   cp config/99-realsense-libusb.rules /etc/udev/rules.d
RUN cd /root/librealsense && \
    mkdir build && \
    cd build && \
    cmake .. -DFORCE_LIBUVC=true && \
    make -j4 && \
    make install

# Install ROS wrapper
ENV ROS_WRAPPER_VERSION 2.2.6
WORKDIR /root
RUN mkdir -p catkin_rs/src && \
    cd catkin_rs/src && \
    git clone https://github.com/pal-robotics/ddynamic_reconfigure.git && \
    git clone https://github.com/intel-ros/realsense.git && \
    cd realsense && \
    git checkout ${ROS_WRAPPER_VERSION}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash ; \
    cd catkin_rs ; \
    catkin_make"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_rs/devel/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN pip install numpy

# Install WiringPi
RUN git clone git://git.drogon.net/wiringPi && \
    cd wiringPi && \
    ./build

WORKDIR /root/catkin_ws
CMD udevadm control --reload-rules && udevadm trigger && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash ; \
    catkin_make ; \
    source /root/catkin_ws/devel/setup.bash"
#    roslaunch proto2_ros bringup.launch

