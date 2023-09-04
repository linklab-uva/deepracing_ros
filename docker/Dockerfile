#This base image is quite comprehensive, but there's still a few more things we need
FROM althack/ros2:humble-dev

# install ros2 packages
RUN apt update && apt upgrade -y && apt install -y --no-install-recommends \
    wget \
    clang \
    clang-format \
    htop \
    nano \
    net-tools \
    tmux \
    # libpcl-dev \
    libopencv-dev \
    libgeographic-dev \
    libusb-1.0-0-dev \
    libflann-dev \
    libqhull-dev \
    libalglib-dev \
    libarmadillo-dev \
    libgl1-mesa-dev \
    libpcap-dev \
    libboost-all-dev \
    libjsoncpp-dev \
    libtbb-dev \
    libeigen3-dev \
    libprotobuf-dev \
    libprotoc-dev \
    protobuf-compiler \
    libopenmpi-dev \
    libopenmpi3 \
    python3-setuptools \
    python3-pip \
    python3-numpy \
    python3-yaml \
    python3-pillow \
    python3-tqdm \
    python3-jinja2 \
    python3-matplotlib \
    python3-opencv \
    python3-pandas \
    python3-requests \
    python3-scipy \
    python3-skimage \
    python3-sklearn \
    python3-transforms3d \
    python3-shapely \
    python3-typeguard \
    python3-lmdb \
    python3-tqdm \
    dos2unix \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-laser-geometry \
    ros-humble-robot-state-publisher \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install pkbar easydict torch
ARG CACHEBUST

RUN sudo update-alternatives --config cc
RUN sudo update-alternatives --config c++
RUN mkdir -p /root/.colcon
ADD ./colcon_defaults.yaml /root/.colcon/defaults.yaml

RUN mkdir -p /opt/ros/deepracingdeps/src
ADD ./deepracingdeps.repos /opt/ros/deepracingdeps/src/deepracingdeps.repos
WORKDIR /opt/ros/deepracingdeps/src
RUN vcs import . --input ./deepracingdeps.repos
RUN chmod +x /opt/ros/humble/setup.sh
WORKDIR /opt/ros/deepracingdeps
ADD ./deps.meta /opt/ros/deepracingdeps/colcon.meta
RUN rm -rf build log install && . /opt/ros/humble/setup.sh && colcon build 

run rm -rf /opt/ros/deepracingdeps/build /opt/ros/deepracingdeps/log
ARG CACHEBUST2
RUN rm -rf /home/ros/deepracingws/src
RUN mkdir -p /home/ros/deepracingws/src
ADD ./deepracing.repos /home/ros/deepracingws/src/deepracing.repos 
WORKDIR /home/ros/deepracingws/src
RUN vcs import . --input ./deepracing.repos
WORKDIR /home/ros/deepracingws
RUN rm -rf build log install && . /opt/ros/deepracingdeps/install/setup.sh && colcon build 

RUN mkdir -p /home/ros/.colcon
ADD ./colcon_defaults.yaml /home/ros/.colcon/defaults.yaml
ADD ./deepracingsetup.bash /home/ros/deepracingsetup.bash
RUN chmod a+x /home/ros/deepracingsetup.bash
RUN chown ros:ros -R /opt/ros/deepracingdeps
RUN chown ros:ros -R /home/ros