FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zurich
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && apt-get install -y --no-install-recommends \
    ros-noetic-cmake-modules \
    ros-noetic-control-toolbox \
    ros-noetic-joy \
    ros-noetic-octomap-ros \
    ros-noetic-mavlink \
    ros-noetic-geographic-msgs \
    ros-noetic-eigen-conversions \
    ros-noetic-tf-conversions \
    ros-noetic-angles \
    ros-noetic-tf2-eigen \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-gazebo-msgs \
    ros-noetic-rviz \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-gazebo-plugins \
    qt5-default qt5-qmake qtbase5-dev-tools qt5-doc \
    autoconf \
    libtool \
    libyaml-cpp-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    liblapacke-dev \
    libgeographic-dev \
    python3-pip \
    git \
    wget \
    rsync
RUN python3 -m pip install future unrealcv catkin-tools
RUN mkdir -p /ws/src/mav_active_3d_planning
WORKDIR /ws
RUN catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel
WORKDIR /ws/src
COPY mav_active_3d_planning_https.rosinstall /ws/src/.rosinstall
RUN wstool update
RUN catkin build eigen_catkin grpc yaml_cpp_catkin
COPY . /ws/src/mav_active_3d_planning
RUN . /ws/devel/setup.sh && catkin build mav_active_3d_planning


