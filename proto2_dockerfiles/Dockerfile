FROM ros:kinetic

ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
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
    sudo \
    udev \
    unzip \
    usbutils \
    wget \
    zip \
&& rm -rf /var/lib/apt/lists/*

# Install LibRealSense
#ENV LIBREALSENSE_VERSION 2.21.0
ENV LIBREALSENSE_VERSION disable-hids
WORKDIR /root/
#RUN git clone https://github.com/IntelRealSense/librealsense.git && 
#    cd librealsense && \
#    git checkout v${LIBREALSENSE_VERSION} && \
#    mkdir -p /etc/udev/rules.d && \
#    cp config/99-realsense-libusb.rules /etc/udev/rules.d
RUN git clone https://github.com/Spritaro/librealsense.git && \
    cd librealsense && \
    git checkout ${LIBREALSENSE_VERSION} && \
    mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d
RUN cd /root/librealsense && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_PYTHON_BINDINGS=bool:true && \
    make -j4 && \
    make install

# Install ROS wrapper
ENV ROS_WRAPPER_VERSION 2.2.3
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
    echo "source /root/catkin_rs/devel/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN pip install numpy

# Install WiringPi
RUN git clone git://git.drogon.net/wiringPi && \
    cd wiringPi && \
    ./build

#
RUN apt-get update && apt-get -y install \
    ros-kinetic-compressed-image-transport \
    ros-kinetic-rosbridge-server \
    ros-kinetic-tf \
    ros-kinetic-joy

CMD udevadm control --reload-rules && udevadm trigger && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make && \
    source /root/catkin_ws/devel/setup.bash
#    roslaunch proto2_ros bringup.launch

