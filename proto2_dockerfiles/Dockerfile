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

# Install WiringPi
RUN git clone https://github.com/WiringPi/WiringPi.git && \
    cd WiringPi && \
    ./build

# install tensorflow
WORKDIR /root
RUN apt-get update && apt-get install -y \
    libhdf5-dev \
    software-properties-common
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py
RUN wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v1.10.0/tensorflow-1.10.0-cp27-none-linux_armv7l.whl && \
    pip install --ignore-installed enum34 && \
    pip install tensorflow-1.10.0-cp27-none-linux_armv7l.whl

ENV MATPLOTLIB_VERSION 2.0.2
RUN pip install \
    Cython \
    contextlib2 \
    jupyter \
    matplotlib==$MATPLOTLIB_VERSION

# download Object Detection API
WORKDIR /usr/src
RUN mkdir tensorflow && \
    cd tensorflow && \
    git clone https://github.com/tensorflow/models.git && \
    cd models && \
    git checkout b07b494e3514553633b132178b4c448f994d59df

# install COCO API
WORKDIR /usr/src/tensorflow/models/research
RUN git clone https://github.com/cocodataset/cocoapi.git && \
    cd cocoapi/PythonAPI && \
    make && \
    cp -r pycocotools /usr/src/tensorflow/models/research

# protobuff installation
WORKDIR /usr/src/tensorflow/models/research
#RUN wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_32.zip && \
#    unzip protobuf.zip && \
#    ./bin/protoc object_detection/protos/*.proto --python_out=.
#RUN apt-get update && apt-get install -y \
#    protobuf-compiler \
# && protoc object_detection/protos/*.proto --python_out=.
RUN apt-get update && apt-get install -y autoconf automake libtool curl make g++ unzip
RUN git clone https://github.com/google/protobuf.git && \
    cd protobuf && \
    git checkout v3.6.0 && \
    git submodule update --init --recursive && \
    ./autogen.sh && \
    ./configure && \
    make && \
    make check && \
    make install && \
    ldconfig
RUN protoc object_detection/protos/*.proto --python_out=.

# Upgrading GLIBCXX
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && apt-get install -y libstdc++6

# add libraries to PYTHONPATH
ENV PYTHONPATH=$PYTHONPATH:/usr/src/tensorflow/models/research:/usr/src/tensorflow/models/research/slim

# default command
WORKDIR /root/catkin_ws
#CMD udevadm control --reload-rules && udevadm trigger && \
CMD /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash ; \
    catkin_make ; \
    source /root/catkin_ws/devel/setup.bash ; \
    roslaunch proto2_ros bringup.launch"
